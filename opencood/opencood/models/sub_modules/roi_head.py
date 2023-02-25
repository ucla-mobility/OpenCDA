import copy

import torch.nn as nn
import torch
import numpy as np
from opencood.pcdet_utils.pointnet2.pointnet2_stack import \
    pointnet2_modules as pointnet2_stack_modules
from opencood.utils import common_utils
from opencood.pcdet_utils.iou3d_nms.iou3d_nms_utils import boxes_iou3d_gpu
from opencood.utils import box_utils


class RoIHead(nn.Module):
    def __init__(self, model_cfg):
        super().__init__()
        self.model_cfg = model_cfg
        input_channels = model_cfg['in_channels']
        self.code_size = 7

        mlps = copy.copy(self.model_cfg['roi_grid_pool']['mlps'])
        for k in range(len(mlps)):
            mlps[k] = [input_channels] + mlps[k]

        self.roi_grid_pool_layer = pointnet2_stack_modules.StackSAModuleMSG(
            radii=self.model_cfg['roi_grid_pool']['pool_radius'],
            nsamples=self.model_cfg['roi_grid_pool']['n_sample'],
            mlps=mlps,
            use_xyz=True,
            pool_method=self.model_cfg['roi_grid_pool']['pool_method'],
        )

        grid_size = self.model_cfg['roi_grid_pool']['grid_size']
        self.grid_size = grid_size
        c_out = sum([x[-1] for x in mlps])
        pre_channel = grid_size * grid_size * grid_size * c_out
        fc_layers = [self.model_cfg['n_fc_neurons']] * 2
        self.shared_fc_layers, pre_channel = self._make_fc_layers(pre_channel,
                                                                  fc_layers)

        self.cls_layers, pre_channel = self._make_fc_layers(pre_channel,
                                                            fc_layers,
                                                            output_channels=
                                                            self.model_cfg[
                                                                'num_cls'])
        self.iou_layers, _ = self._make_fc_layers(pre_channel, fc_layers,
                                                  output_channels=
                                                  self.model_cfg['num_cls'])
        self.reg_layers, _ = self._make_fc_layers(pre_channel, fc_layers,
                                                  output_channels=
                                                  self.model_cfg[
                                                      'num_cls'] * 7)

        self._init_weights(weight_init='xavier')

    def _init_weights(self, weight_init='xavier'):
        if weight_init == 'kaiming':
            init_func = nn.init.kaiming_normal_
        elif weight_init == 'xavier':
            init_func = nn.init.xavier_normal_
        elif weight_init == 'normal':
            init_func = nn.init.normal_
        else:
            raise NotImplementedError

        for m in self.modules():
            if isinstance(m, nn.Conv2d) or isinstance(m, nn.Conv1d):
                if weight_init == 'normal':
                    init_func(m.weight, mean=0, std=0.001)
                else:
                    init_func(m.weight)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
        nn.init.normal_(self.reg_layers[-1].weight, mean=0, std=0.001)

    def _make_fc_layers(self, input_channels, fc_list, output_channels=None):
        fc_layers = []
        pre_channel = input_channels
        for k in range(len(fc_list)):
            fc_layers.extend([
                nn.Conv1d(pre_channel, fc_list[k], kernel_size=1, bias=False),
                # nn.BatchNorm1d(fc_list[k]),
                nn.ReLU()
            ])
            pre_channel = fc_list[k]
            if self.model_cfg['dp_ratio'] > 0:
                fc_layers.append(nn.Dropout(self.model_cfg['dp_ratio']))
        if output_channels is not None:
            fc_layers.append(
                nn.Conv1d(pre_channel, output_channels, kernel_size=1,
                          bias=True))
        fc_layers = nn.Sequential(*fc_layers)
        return fc_layers, pre_channel

    def get_global_grid_points_of_roi(self, rois):
        rois = rois.view(-1, rois.shape[-1])
        batch_size_rcnn = rois.shape[0]

        # (B, 6x6x6, 3)
        local_roi_grid_points = self.get_dense_grid_points(rois,
                                                           batch_size_rcnn,
                                                           self.grid_size)
        global_roi_grid_points = common_utils.rotate_points_along_z(
            local_roi_grid_points.clone(), rois[:, 6]
        ).squeeze(dim=1)
        global_center = rois[:, 0:3].clone()
        global_roi_grid_points += global_center.unsqueeze(dim=1)
        return global_roi_grid_points, local_roi_grid_points

    @staticmethod
    def get_dense_grid_points(rois, batch_size_rcnn, grid_size):
        """
        Get the local coordinates of each grid point of a roi in the coordinate
        system of the roi(origin lies in the center of this roi.
        """
        faked_features = rois.new_ones((grid_size, grid_size, grid_size))
        dense_idx = torch.stack(torch.where(faked_features),
                                dim=1)  # (N, 3) [x_idx, y_idx, z_idx]
        dense_idx = dense_idx.repeat(batch_size_rcnn, 1,
                                     1).float()  # (B, 6x6x6, 3)

        local_roi_size = rois.view(batch_size_rcnn, -1)[:, 3:6]
        roi_grid_points = (
                                  dense_idx + 0.5) / grid_size * local_roi_size.unsqueeze(
            dim=1) \
                          - (local_roi_size.unsqueeze(
            dim=1) / 2)  # (B, 6x6x6, 3)
        return roi_grid_points

    def assign_targets(self, batch_dict):
        batch_dict['rcnn_label_dict'] = {
            'rois': [],
            'gt_of_rois': [],
            'gt_of_rois_src': [],
            'cls_tgt': [],
            'reg_tgt': [],
            'iou_tgt': [],
            'rois_anchor': [],
            'record_len': []
        }
        # pred_boxes = [boxes[:, [0, 1, 2, 5, 4, 3, 6]] for boxes in batch_dict['boxes_fused']]
        pred_boxes = batch_dict['boxes_fused']
        gt_boxes = [b[m][:, [0, 1, 2, 5, 4, 3, 6]].float() for b, m in
                    zip(batch_dict['object_bbx_center'],
                        batch_dict['object_bbx_mask'].bool())]
        for rois, gts in zip(pred_boxes, gt_boxes):
            gts[:, -1] *= 1
            ious = boxes_iou3d_gpu(rois, gts)
            max_ious, gt_inds = ious.max(dim=1)
            gt_of_rois = gts[gt_inds]
            rcnn_labels = (max_ious > 0.3).float()
            mask = torch.logical_not(rcnn_labels.bool())

            # set negative samples back to rois, no correction in stage2 for them
            gt_of_rois[mask] = rois[mask]
            gt_of_rois_src = gt_of_rois.clone().detach()

            # canoical transformation
            roi_center = rois[:, 0:3]
            # TODO: roi_ry > 0 in pcdet
            roi_ry = rois[:, 6] % (2 * np.pi)
            gt_of_rois[:, 0:3] = gt_of_rois[:, 0:3] - roi_center
            gt_of_rois[:, 6] = gt_of_rois[:, 6] - roi_ry

            # transfer LiDAR coords to local coords
            gt_of_rois = common_utils.rotate_points_along_z(
                points=gt_of_rois.view(-1, 1, gt_of_rois.shape[-1]),
                angle=-roi_ry.view(-1)
            ).view(-1, gt_of_rois.shape[-1])

            # flip orientation if rois have opposite orientation
            heading_label = (gt_of_rois[:, 6] + (
                    torch.div(torch.abs(gt_of_rois[:, 6].min()),
                              (2 * np.pi), rounding_mode='trunc')
                    + 1) * 2 * np.pi) % (2 * np.pi)  # 0 ~ 2pi
            opposite_flag = (heading_label > np.pi * 0.5) & (
                    heading_label < np.pi * 1.5)

            # (0 ~ pi/2, 3pi/2 ~ 2pi)
            heading_label[opposite_flag] = (heading_label[
                                                opposite_flag] + np.pi) % (
                                                   2 * np.pi)
            flag = heading_label > np.pi
            heading_label[flag] = heading_label[
                                      flag] - np.pi * 2  # (-pi/2, pi/2)
            heading_label = torch.clamp(heading_label, min=-np.pi / 2,
                                        max=np.pi / 2)
            gt_of_rois[:, 6] = heading_label

            # generate regression target
            rois_anchor = rois.clone().detach().view(-1, self.code_size)
            rois_anchor[:, 0:3] = 0
            rois_anchor[:, 6] = 0

            reg_targets = box_utils.box_encode(
                gt_of_rois.view(-1, self.code_size), rois_anchor
            )

            batch_dict['rcnn_label_dict']['rois'].append(rois)
            batch_dict['rcnn_label_dict']['gt_of_rois'].append(gt_of_rois)
            batch_dict['rcnn_label_dict']['gt_of_rois_src'].append(
                gt_of_rois_src)
            batch_dict['rcnn_label_dict']['cls_tgt'].append(rcnn_labels)
            batch_dict['rcnn_label_dict']['reg_tgt'].append(reg_targets)
            batch_dict['rcnn_label_dict']['iou_tgt'].append(max_ious)
            batch_dict['rcnn_label_dict']['rois_anchor'].append(rois_anchor)
            batch_dict['rcnn_label_dict']['record_len'].append(rois.shape[0])

        # cat list to tensor
        for k, v in batch_dict['rcnn_label_dict'].items():
            if k == 'record_len':
                continue
            batch_dict['rcnn_label_dict'][k] = torch.cat(v, dim=0)

        return batch_dict

    def roi_grid_pool(self, batch_dict):
        batch_size = len(batch_dict['record_len'])
        rois = batch_dict['rcnn_label_dict']['rois']
        point_coords = batch_dict['point_coords']
        point_features = batch_dict['point_features']
        label_record_len = batch_dict['rcnn_label_dict']['record_len']

        point_features = torch.cat(point_features, dim=0)
        # (BxN, 6x6x6, 3)
        global_roi_grid_points, local_roi_grid_points = \
            self.get_global_grid_points_of_roi(rois)
        # (B, Nx6x6x6, 3)
        global_roi_grid_points = global_roi_grid_points.view(batch_size, -1, 3)

        xyz = torch.cat(point_coords, dim=0)
        xyz_batch_cnt = xyz.new_zeros(batch_size).int()
        for bs_idx in range(batch_size):
            xyz_batch_cnt[bs_idx] = len(point_coords[bs_idx])
        new_xyz = global_roi_grid_points.view(-1, 3)
        new_xyz_batch_cnt = xyz.new_zeros(batch_size).int()
        for bs_idx in range(batch_size):
            new_xyz_batch_cnt[bs_idx] = label_record_len[
                                            bs_idx] * self.grid_size ** 3

        pooled_points, pooled_features = self.roi_grid_pool_layer(
            xyz=xyz[:, :3].contiguous(),
            xyz_batch_cnt=xyz_batch_cnt,
            new_xyz=new_xyz[:, :3].contiguous(),
            new_xyz_batch_cnt=new_xyz_batch_cnt,
            features=point_features.contiguous(),  # weighted point features
        )  # (M1 + M2 ..., C)
        # (BxN, 6x6x6, C)
        pooled_features = pooled_features.view(-1, self.grid_size ** 3,
                                               pooled_features.shape[-1])

        return pooled_features

    def forward(self, batch_dict):
        batch_dict = self.assign_targets(batch_dict)
        # RoI aware pooling
        pooled_features = self.roi_grid_pool(batch_dict)  # (BxN, 6x6x6, C)

        batch_size_rcnn = pooled_features.shape[0]
        pooled_features = pooled_features.permute(0, 2, 1). \
            contiguous().view(batch_size_rcnn, -1, self.grid_size,
                              self.grid_size,
                              self.grid_size)  # (BxN, C, 6, 6, 6)
        shared_features = self.shared_fc_layers(
            pooled_features.view(batch_size_rcnn, -1, 1))
        rcnn_cls = self.cls_layers(shared_features).transpose(1,
                                                              2).contiguous().squeeze(
            dim=1)  # (B, 1 or 2)
        rcnn_iou = self.iou_layers(shared_features).transpose(1,
                                                              2).contiguous().squeeze(
            dim=1)  # (B, 1)
        rcnn_reg = self.reg_layers(shared_features).transpose(1,
                                                              2).contiguous().squeeze(
            dim=1)  # (B, C)

        batch_dict['fpvrcnn_out'] = {
            'rcnn_cls': rcnn_cls,
            'rcnn_iou': rcnn_iou,
            'rcnn_reg': rcnn_reg,
        }
        return batch_dict
