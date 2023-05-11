"""
3D Anchor Generator for Voxel
"""
import math
import sys

import numpy as np
import torch
import torch.nn.functional as F

from opencood.data_utils.post_processor.voxel_postprocessor \
    import VoxelPostprocessor
from opencood.utils import box_utils


class CiassdPostprocessor(VoxelPostprocessor):
    def __init__(self, anchor_params, train):
        super(CiassdPostprocessor, self).__init__(anchor_params, train)
        self.train = train
        self.anchor_num = self.params['anchor_args']['num']

    def post_process(self, data_dict, output_dict):
        """
        Process the outputs of the model to 2D/3D bounding box.
        Step1: convert each cav's output to bounding box format
        Step2: project the bounding boxes to ego space.
        Step:3 NMS

        Parameters
        ----------
        data_dict : dict
            The dictionary containing the origin input data of model.

        output_dict :dict
            The dictionary containing the output of the model.

        Returns
        -------
        pred_box3d_tensor : torch.Tensor
            The prediction bounding box tensor after NMS.
        gt_box3d_tensor : torch.Tensor
            The groundtruth bounding box tensor.
        """
        # the final bounding box list
        global batch_num_box_count
        pred_box3d_original_list = []
        pred_box3d_list = []
        pred_box2d_list = []

        for cav_id, cav_content in data_dict.items():
            assert cav_id in output_dict
            # the transformation matrix to ego space
            if 'transformation_matrix' in cav_content:
                transformation_matrix = cav_content['transformation_matrix']
            else:
                transformation_matrix = torch.from_numpy(np.identity(4)).float().\
                    to(cav_content['anchor_box'].device)

            # (H, W, anchor_num, 7)
            anchor_box = cav_content['anchor_box']

            # prediction result
            preds_dict = output_dict[cav_id]['preds_dict_stage1']

            # preds
            prob = preds_dict['cls_preds']
            prob = torch.sigmoid(prob.permute(0, 2, 3, 1).contiguous())
            reg = preds_dict['box_preds'].permute(0, 2, 3, 1).contiguous()
            iou = preds_dict['iou_preds'].permute(0, 2, 3, 1).contiguous().reshape(1, -1)
            dir = preds_dict['dir_cls_preds'].permute(0, 2, 3, 1).contiguous().reshape(1, -1, 2)

            # convert regression map back to bounding box
            # (N, W*L*anchor_num, 7)
            batch_box3d = self.delta_to_boxes3d(reg, anchor_box, False)
            mask = torch.gt(prob, self.params['target_args']['score_threshold'])
            batch_num_box_count = [int(m.sum()) for m in mask]
            mask = mask.view(1, -1)
            mask_reg = mask.unsqueeze(2).repeat(1, 1, 7)

            # during validation/testing, the batch size should be 1
            if not self.train:
                assert batch_box3d.shape[0] == 1

            boxes3d = torch.masked_select(batch_box3d.view(-1, 7), mask_reg[0]).view(-1, 7)
            scores = torch.masked_select(prob.view(-1), mask[0])

            dir_labels = torch.max(dir, dim=-1)[1]
            dir_labels = dir_labels[mask]
            # top_labels = torch.zeros([scores.shape[0]], dtype=torch.long).cuda()
            if scores.shape[0] != 0:
                iou = (iou + 1) * 0.5
                scores = scores * torch.pow(iou.masked_select(mask), 4)
                # correct_direction
                top_labels = (boxes3d[..., -1] > 0) ^ (dir_labels.byte() == 1)
                boxes3d[..., -1] += torch.where(top_labels, torch.tensor(np.pi).type_as(boxes3d),
                                                  torch.tensor(0.0).type_as(boxes3d))
                pred_box3d_original_list.append(boxes3d.detach())

            # convert output to bounding box
            if len(boxes3d) != 0:
                # (N, 8, 3)
                boxes3d_corner = box_utils.boxes_to_corners_3d(boxes3d, order=self.params['order'])
                # (N, 8, 3)
                projected_boxes3d = box_utils.project_box3d(boxes3d_corner, transformation_matrix)
                # convert 3d bbx to 2d, (N,4)
                projected_boxes2d = box_utils.corner_to_standup_box_torch(projected_boxes3d)
                # (N, 5)
                boxes2d_score = torch.cat((projected_boxes2d, scores.unsqueeze(1)), dim=1)

                pred_box2d_list.append(boxes2d_score)
                pred_box3d_list.append(projected_boxes3d)

        if len(pred_box2d_list) ==0 or len(pred_box3d_list) == 0:
            return None, None
        # shape: (N, 5)
        pred_box2d_list = torch.vstack(pred_box2d_list)
        # scores
        scores = pred_box2d_list[:, -1]
        # predicted 3d bbx
        pred_box3d_tensor = torch.vstack(pred_box3d_list)
        pred_box3d_original = torch.vstack(pred_box3d_original_list)

        if not self.train:
            # remove large bbx
            keep_index_1 = box_utils.remove_large_pred_bbx(pred_box3d_tensor)
            keep_index_2 = box_utils.remove_bbx_abnormal_z(pred_box3d_tensor)
            keep_index = torch.logical_and(keep_index_1, keep_index_2)

            pred_box3d_tensor = pred_box3d_tensor[keep_index]
            scores = scores[keep_index]

            # nms
            keep_index = box_utils.nms_rotated(pred_box3d_tensor,
                                               scores,
                                               self.params['nms_thresh']
                                               )

            pred_box3d_tensor = pred_box3d_tensor[keep_index]

            # select cooresponding score
            scores = scores[keep_index]

            # filter out the prediction out of the range.
            mask = \
                box_utils.get_mask_for_boxes_within_range_torch(pred_box3d_tensor)
            pred_box3d_tensor = pred_box3d_tensor[mask, :, :]
            scores = scores[mask]

            assert scores.shape[0] == pred_box3d_tensor.shape[0]
            return pred_box3d_tensor, scores
        else:
            cur_idx = 0
            batch_pred_boxes3d = []
            batch_scores = []
            for n in batch_num_box_count:
                cur_boxes = pred_box3d_tensor[cur_idx:cur_idx+n]
                cur_scores = scores[cur_idx:cur_idx+n]
                # nms
                keep_index = box_utils.nms_rotated(cur_boxes,
                                                   cur_scores,
                                                   self.params['nms_thresh']
                                                   )
                cur_boxes = pred_box3d_original[cur_idx:cur_idx+n] # [:, [0, 1, 2, 5, 4, 3, 6]] # hwl -> lwh
                batch_pred_boxes3d.append(cur_boxes[keep_index])
                batch_scores.append(cur_scores[keep_index])
                cur_idx += n

            return batch_pred_boxes3d, batch_scores

