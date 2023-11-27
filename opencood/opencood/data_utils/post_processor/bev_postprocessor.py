# -*- coding: utf-8 -*-
# Author: Hao Xiang haxiang@g.ucla.edu
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Anchor-free 2d Generator
"""

import numpy as np
import torch
import torch.nn.functional as F

from opencood.utils.transformation_utils import dist_to_continuous
from opencood.data_utils.post_processor.base_postprocessor \
    import BasePostprocessor
from opencood.utils import box_utils
from opencood.visualization import vis_utils


class BevPostprocessor(BasePostprocessor):
    def __init__(self, anchor_params, train):
        super(BevPostprocessor, self).__init__(anchor_params, train)
        # self.geometry_param = anchor_params["geometry"]
        self.geometry_param = anchor_params["geometry_param"]

        # TODO
        # Hard coded for now. Need to calculate for our own training dataset
        self.target_mean = np.array([0.008, 0.001, 0.202, 0.2, 0.43, 1.368])
        self.target_std_dev = np.array([0.866, 0.5, 0.954, 0.668, 0.09, 0.111])

    def generate_anchor_box(self):
        return None

    def generate_label(self, **kwargs):
        """
        Generate targets for training.

        Parameters
        ----------
        kwargs : list
            gt_box_center:(max_num, 7)

        Returns
        -------
        label_dict : dict
            Dictionary that contains all target related info.
        """
        assert self.params['order'] == 'lwh', \
            'Currently BEV only support lwh bbx order.'
        # (max_num, 7)
        gt_box_center = kwargs['gt_box_center']

        # (max_num)
        masks = kwargs['mask']

        # (n, 7)
        gt_box_center_valid = gt_box_center[masks == 1]
        # (n, 4, 3)
        bev_corners = box_utils.boxes_to_corners2d(gt_box_center_valid,
                                                   self.params['order'])

        n = gt_box_center_valid.shape[0]
        # (n, 4, 2)
        bev_corners = bev_corners[:, :, :2]
        yaw = gt_box_center_valid[:, -1]
        x, y = gt_box_center_valid[:, 0], gt_box_center_valid[:, 1]
        dx, dy = gt_box_center_valid[:, 3], gt_box_center_valid[:, 4]
        # (n, 6)
        reg_targets = np.column_stack([np.cos(yaw), np.sin(yaw), x, y, dx, dy])

        # target label map including classification and regression targets
        label_map = np.zeros(self.geometry_param["label_shape"])
        self.update_label_map(label_map, bev_corners, reg_targets)
        label_map = self.normalize_targets(label_map)
        label_dict = {
            # (7, label_shape[0], label_shape[1])
            "label_map": np.transpose(label_map, (2, 0, 1)).astype(np.float32),
            "bev_corners": bev_corners
        }
        return label_dict

    def update_label_map(self, label_map, bev_corners, reg_targets):
        """
        Update label_map based on bbx and regression targets.

        Parameters
        ----------
        label_map : numpy.array
            Targets array for classification and regression tasks with
            the shape of label_shape.

        bev_corners : numpy.array
            The bbx corners in lidar frame with shape (n, 4, 2)

        reg_targets : numpy.array
            Array containing the regression targets information. It need to be
            further processed.

        """
        res = self.geometry_param["res"]
        downsample_rate = self.geometry_param["downsample_rate"]

        bev_origin = np.array([self.geometry_param["L1"],
                               self.geometry_param["W1"]]).reshape(1, -1)

        # discretized bbx corner representations -- (n, 4, 2)
        bev_corners_dist = (bev_corners - bev_origin) / res / downsample_rate
        # generate the coordinates of m
        x = np.arange(self.geometry_param["label_shape"][0])
        y = np.arange(self.geometry_param["label_shape"][1])
        xx, yy = np.meshgrid(x, y)

        # (label_shape[0]*label_shape[1], 2)
        points = np.concatenate([xx.reshape(-1, 1), yy.reshape(-1, 1)],
                                axis=-1)
        bev_origin_dist = bev_origin / res / downsample_rate

        # loop over each bbx, find the points within the bbx.
        for i in range(bev_corners.shape[0]):
            reg_target = reg_targets[i, :]

            # find discredited points in bbx
            points_in_box = \
                box_utils.get_points_in_rotated_box(points,
                                                    bev_corners_dist[i, ...])
            # convert points to continuous space
            points_continuous = dist_to_continuous(points_in_box,
                                                   bev_origin_dist,
                                                   res,
                                                   downsample_rate)
            actual_reg_target = np.repeat(reg_target.reshape(1, -1),
                                          points_continuous.shape[0],
                                          axis=0)
            # build learning targets
            actual_reg_target[:, 2:4] = \
                actual_reg_target[:, 2:4] - points_continuous
            actual_reg_target[:, 4:] = np.log(actual_reg_target[:, 4:])

            # update label map
            label_map[points_in_box[:, 0], points_in_box[:, 1], 0] = 1.0
            label_map[points_in_box[:, 0], points_in_box[:, 1], 1:] = \
                actual_reg_target

    def normalize_targets(self, label_map):
        """
        Normalize label_map

        Parameters
        ----------
        label_map : numpy.array
            Targets array for classification and regression tasks with the
            shape of label_shape.

        Returns
        -------
        label_map: numpy.array
            Nromalized label_map.

        """
        label_map[..., 1:] = \
            (label_map[..., 1:] - self.target_mean) / self.target_std_dev
        return label_map

    def denormalize_reg_map(self, reg_map):
        """
        Denormalize the regression map

        Parameters
        ----------
        reg_map : np.ndarray / torch.Tensor
            Regression output mapwith the shape of (label_shape[0],
            label_shape[1], 6).

        Returns
        -------
        reg_map : np.ndarray / torch.Tensor
            Denormalized regression map.

        """
        if isinstance(reg_map, np.ndarray):
            target_mean = self.target_mean
            target_std_dev = self.target_std_dev

        else:
            target_mean = \
                torch.from_numpy(self.target_mean).to(reg_map.device)
            target_std_dev = \
                torch.from_numpy(self.target_std_dev).to(reg_map.device)
        reg_map = reg_map * target_std_dev + target_mean
        return reg_map

    @staticmethod
    def collate_batch(label_batch_list):
        """
        Customized collate function for target label generation.

        Parameters
        ----------
        label_batch_list : list
            The list of dictionary  that contains all labels for several
            frames.

        Returns
        -------
        processed_batch : dict
            Reformatted labels in torch tensor.
        """
        label_map_list = [x["label_map"][np.newaxis, ...] for x in
                          label_batch_list]
        processed_batch = {
            # (batch_size, 7, label_shape[0], label_shape[1])
            "label_map": torch.from_numpy(np.concatenate(label_map_list,
                                                         axis=0)),
            "bev_corners": [torch.from_numpy(x["bev_corners"]) for x in
                            label_batch_list]
        }
        return processed_batch

    def post_process(self, data_dict, output_dict):
        """
        Process the outputs of the model to 2D bounding box.
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
        pred_box2d_tensor : torch.Tensor
            The prediction bounding box tensor after NMS.

        gt_box2d_tensor : torch.Tensor
            The groundtruth bounding box tensor.
        """

        # the final bounding box list
        pred_box2d_list = []
        pred_score_list = []

        for cav_id, cav_content in data_dict.items():
            assert cav_id in output_dict
            # the transformation matrix to ego space
            transformation_matrix = cav_content['transformation_matrix']

            # classification probability -- (label_shape[0], label_shape[1])
            prob = output_dict[cav_id]['cls'].squeeze(0).squeeze(0)
            prob = torch.sigmoid(prob)
            # regression map -- (label_shape[0], label_shape[1], 6)
            reg_map = output_dict[cav_id]['reg'].squeeze(0).permute(1, 2, 0)
            reg_map = self.denormalize_reg_map(reg_map)
            threshold = self.params['target_args']['score_threshold']
            mask = torch.gt(prob, threshold)

            if mask.sum() > 0:
                # (number of high confidence bbx, 4, 2)
                corners2d = self.reg_map_to_bbx_corners(reg_map, mask)
                # assume the z-diviation in transformation_matrix is small,
                # thus we can pad zeros to simulate the 3d transformation.
                # (number of high confidence bbx, 4, 3)
                box3d = F.pad(corners2d, (0, 1))
                # (number of high confidence bbx, 4, 2)
                projected_boxes2d = \
                    box_utils.project_points_by_matrix_torch(box3d.view(-1, 3),
                                                             transformation_matrix)[
                    :, :2]

                projected_boxes2d = projected_boxes2d.view(-1, 4, 2)
                scores = prob[mask]
                pred_box2d_list.append(projected_boxes2d)
                pred_score_list.append(scores)

        if len(pred_box2d_list):
            pred_box2ds = torch.cat(pred_box2d_list, dim=0)
            pred_scores = torch.cat(pred_score_list, dim=0)
        else:
            return None, None

        keep_index = box_utils.nms_rotated(pred_box2ds, pred_scores,
                                           self.params['nms_thresh'])
        if len(keep_index):
            pred_box2ds = pred_box2ds[keep_index]
            pred_scores = pred_scores[keep_index]

        # filter out the prediction out of the range.
        mask = box_utils.get_mask_for_boxes_within_range_torch(pred_box2ds)
        pred_box2ds = pred_box2ds[mask, :, :]
        pred_scores = pred_scores[mask]
        assert pred_scores.shape[0] == pred_box2ds.shape[0]
        return pred_box2ds, pred_scores

    def reg_map_to_bbx_corners(self, reg_map, mask):
        """
        Construct bbx from the regression output of the model.

        Parameters
        ----------
        reg_map : torch.Tensor
            Regression output of neural networks.

        mask : torch.Tensor
            Masks used to filter bbx.

        Returns
        -------
        corners : torch.Tensor
            Bbx output with shape (N, 4, 2).

        """

        assert len(reg_map.shape) == 3, \
            "only support shape of label_shape i.e. (*, *, 6)"
        device = reg_map.device

        cos_t, sin_t, x, y, log_dx, log_dy = \
            [tt.squeeze(-1) for tt in torch.chunk(reg_map, 6, dim=-1)]
        yaw = torch.atan2(sin_t, cos_t)
        dx, dy = log_dx.exp(), log_dy.exp()

        grid_size = self.geometry_param["res"] * \
                    self.geometry_param["downsample_rate"]
        grid_x = torch.arange(self.geometry_param["L1"],
                              self.geometry_param["L2"],
                              grid_size, dtype=torch.float32, device=device)
        grid_y = torch.arange(self.geometry_param["W1"],
                              self.geometry_param["W2"],
                              grid_size,
                              dtype=torch.float32,
                              device=device)

        xx, yy = torch.meshgrid([grid_x, grid_y])
        center_x = xx + x
        center_y = yy + y

        bbx2d = torch.stack([center_x, center_y, dx, dy, yaw], dim=-1)
        bbx2d = bbx2d[mask, :]
        corners = box_utils.boxes2d_to_corners2d(bbx2d)

        return corners

    def post_process_debug(self, data_dict, output_dict):
        """
        Process the outputs of the model to 2D bounding box for debug purpose.
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
        pred_box2d_tensor : torch.Tensor
            The prediction bounding box tensor after NMS.
        gt_box2d_tensor : torch.Tensor
            The groundtruth bounding box tensor.
        """
        # the final bounding box list
        pred_box2d_list = []
        pred_score_list = []

        # the transformation matrix to ego space
        transformation_matrix = data_dict['transformation_matrix']

        # classification probability -- (label_shape[0], label_shape[1])
        prob = output_dict['cls'].squeeze(0).squeeze(0)
        prob = torch.sigmoid(prob)

        # regression map -- (label_shape[0], label_shape[1], 6)
        reg_map = output_dict['reg'].squeeze(0).permute(1, 2, 0)
        reg_map = self.denormalize_reg_map(reg_map)

        threshold = 0.5
        mask = torch.gt(prob, threshold)

        if mask.sum() > 0:
            # (number of high confidence bbx, 4, 2)
            corners2d = self.reg_map_to_bbx_corners(reg_map, mask)
            # assume the z-diviation in transformation_matrix is small,
            # thus we can pad zeros to simulate the 3d transformation.
            # (number of high confidence bbx, 4, 3)
            box3d = F.pad(corners2d, (0, 1))

            # (number of high confidence bbx, 4, 2)
            projected_boxes2d = \
                box_utils.project_points_by_matrix_torch(box3d.view(-1, 3),
                                                         transformation_matrix)[:, :2]
            projected_boxes2d = projected_boxes2d.view(-1, 4, 2)
            scores = prob[mask]
            pred_box2d_list.append(projected_boxes2d)
            pred_score_list.append(scores)

        pred_box2ds = torch.cat(pred_box2d_list, dim=0)
        pred_scores = torch.cat(pred_score_list, dim=0)

        keep_index = box_utils.nms_rotated(pred_box2ds,
                                           pred_scores,
                                           self.params['nms_thresh'])
        pred_box2ds = pred_box2ds[keep_index]

        # filter out the prediction out of the range.
        mask = box_utils.get_mask_for_boxes_within_range_torch(pred_box2ds)
        pred_box2ds = pred_box2ds[mask, :, :]
        return pred_box2ds

    @staticmethod
    def visualize(pred_box_tensor, gt_tensor, pcd, show_vis, save_path,
                  dataset=None):
        """
        Visualize the BEV 2D prediction, ground truth with point cloud together.

        Parameters
        ----------
        pred_box_tensor : torch.Tensor
            (N, 8, 3) prediction.

        gt_tensor : torch.Tensor
            (N, 8, 3) groundtruth bbx

        pcd : torch.Tensor
            PointCloud, (N, 4).

        show_vis : bool
            Whether to show visualization.

        save_path : str
            Save the visualization results to given path.

        dataset : BaseDataset
            opencood dataset object.
        """
        assert dataset is not None, "dataset argument can't be None"
        vis_utils.visualize_single_sample_output_bev(pred_box_tensor,
                                                     gt_tensor,
                                                     pcd,
                                                     dataset,
                                                     show_vis,
                                                     save_path)
