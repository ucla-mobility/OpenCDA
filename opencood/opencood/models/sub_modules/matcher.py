import torch
from torch import nn

from opencood.pcdet_utils.iou3d_nms.iou3d_nms_utils import boxes_iou3d_gpu

pi = 3.141592653


def limit_period(val, offset=0.5, period=2 * pi):
    return val - torch.floor(val / period + offset) * period


class Matcher(nn.Module):
    """Correct localization error and use Algorithm 1:
     BBox matching with scores to fuse the proposal BBoxes"""

    def __init__(self, cfg, pc_range):
        super(Matcher, self).__init__()
        self.pc_range = pc_range

    @torch.no_grad()
    def forward(self, data_dict):
        clusters, scores = self.clustering(data_dict)
        data_dict['boxes_fused'], data_dict[
            'scores_fused'] = self.cluster_fusion(clusters, scores)
        self.merge_keypoints(data_dict)
        return data_dict

    def clustering(self, data_dict):
        """
        Assign predicted boxes to clusters according to their ious with each other
        """
        clusters_batch = []
        scores_batch = []
        record_len = [int(l) for l in data_dict['record_len']]
        for i, l in enumerate(record_len):
            cur_boxes_list = data_dict['det_boxes'][sum(record_len[:i]):sum(record_len[:i])+l]
            cur_scores_list = data_dict['det_scores'][sum(record_len[:i]):sum(record_len[:i])+l]
            cur_boxes_list = [b for b in cur_boxes_list if len(b) > 0]
            cur_scores_list = [s for s in cur_scores_list if len(s) > 0]
            if len(cur_scores_list) == 0:
                clusters_batch.append([torch.Tensor([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.57]).
                                      to(torch.device('cuda:0')).view(1, 7)])
                scores_batch.append([torch.Tensor([0.01]).to(torch.device('cuda:0')).view(-1)])
                continue

            pred_boxes_cat = torch.cat(cur_boxes_list, dim=0)
            pred_boxes_cat[:, -1] = limit_period(pred_boxes_cat[:, -1])
            pred_scores_cat = torch.cat(cur_scores_list, dim=0)

            ious = boxes_iou3d_gpu(pred_boxes_cat, pred_boxes_cat)
            cluster_indices = torch.zeros(len(ious)).int() # gt assignments of preds
            cur_cluster_id = 1
            while torch.any(cluster_indices == 0):
                cur_idx = torch.where(cluster_indices == 0)[0][0] # find the idx of the first pred which is not assigned yet
                cluster_indices[torch.where(ious[cur_idx] > 0.1)[0]] = cur_cluster_id
                cur_cluster_id += 1
            clusters = []
            scores = []
            for j in range(1, cur_cluster_id):
                clusters.append(pred_boxes_cat[cluster_indices==j])
                scores.append(pred_scores_cat[cluster_indices==j])
            clusters_batch.append(clusters)
            scores_batch.append(scores)

        return clusters_batch, scores_batch

    def cluster_fusion(self, clusters, scores):
        """
        Merge boxes in each cluster with scores as weights for merging
        """
        boxes_fused = []
        scores_fused = []
        for cl, sl in zip(clusters, scores):
            for c, s in zip(cl, sl):
                # reverse direction for non-dominant direction of boxes
                dirs = c[:, -1]
                max_score_idx = torch.argmax(s)
                dirs_diff = torch.abs(dirs - dirs[max_score_idx].item())
                lt_pi = (dirs_diff > pi).int()
                dirs_diff = dirs_diff * (1 - lt_pi) + (
                            2 * pi - dirs_diff) * lt_pi
                score_lt_half_pi = s[dirs_diff > pi / 2].sum()  # larger than
                score_set_half_pi = s[
                    dirs_diff <= pi / 2].sum()  # small equal than
                # select larger scored direction as final direction
                if score_lt_half_pi <= score_set_half_pi:
                    dirs[dirs_diff > pi / 2] += pi
                else:
                    dirs[dirs_diff <= pi / 2] += pi
                dirs = limit_period(dirs)
                s_normalized = s / s.sum()
                sint = torch.sin(dirs) * s_normalized
                cost = torch.cos(dirs) * s_normalized
                theta = torch.atan2(sint.sum(), cost.sum()).view(1, )
                center_dim = c[:, :-1] * s_normalized[:, None]
                boxes_fused.append(torch.cat([center_dim.sum(dim=0), theta]))
                s_sorted = torch.sort(s, descending=True).values
                s_fused = 0
                for i, ss in enumerate(s_sorted):
                    s_fused += ss ** (i + 1)
                s_fused = torch.tensor([min(s_fused, 1.0)], device=s.device)
                scores_fused.append(s_fused)

        assert len(boxes_fused) > 0
        boxes_fused = torch.stack(boxes_fused, dim=0)
        len_records = [len(c) for c in clusters]
        boxes_fused = [
            boxes_fused[sum(len_records[:i]):sum(len_records[:i]) + l] for i, l
            in enumerate(len_records)]
        scores_fused = torch.stack(scores_fused, dim=0)
        scores_fused = [
            scores_fused[sum(len_records[:i]):sum(len_records[:i]) + l] for
            i, l in enumerate(len_records)]

        return boxes_fused, scores_fused

    def merge_keypoints(self, data_dict):
        # merge keypoints
        kpts_feat_out = []
        kpts_coor_out = []
        keypoints_features = data_dict['point_features']
        keypoints_coords = data_dict['point_coords']
        idx = 0
        for l in data_dict['record_len']:
            kpts_coor_out.append(
                torch.cat(keypoints_coords[idx:l + idx], dim=0))
            kpts_feat_out.append(
                torch.cat(keypoints_features[idx:l + idx], dim=0))
            idx += l
        data_dict['point_features'] = kpts_feat_out
        data_dict['point_coords'] = kpts_coor_out
