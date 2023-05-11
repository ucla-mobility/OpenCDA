# -*- coding: utf-8 -*-
# Author: Yunshuang Yuan <yunshuang.yuan@ikg.uni-hannover.de>
# Modified by: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Dataset class for 2-stage backbone intermediate fusion
"""
import math
from collections import OrderedDict

import numpy as np
import torch

import opencood.data_utils.datasets
import opencood.data_utils.post_processor as post_processor
from opencood.utils import box_utils
from opencood.data_utils.datasets import basedataset
from opencood.data_utils.pre_processor import build_preprocessor
from opencood.utils.pcd_utils import \
    mask_points_by_range, mask_ego_points, shuffle_points
from opencood.utils.transformation_utils import x1_to_x2


class IntermediateFusionDatasetV2(basedataset.BaseDataset):
    """
    This class is for intermediate fusion where each vehicle transmit the
    deep features to ego.
    """

    def __init__(self, params, visualize, train=True):
        super(IntermediateFusionDatasetV2, self). \
            __init__(params, visualize, train)
        self.pre_processor = \
            build_preprocessor(params['preprocess'], train)
        self.post_processor = \
            post_processor.build_postprocessor(params['postprocess'], train)

    def __getitem__(self, idx):
        # put here to avoid initialization error
        from opencood.pcdet_utils.roiaware_pool3d.roiaware_pool3d_utils \
            import points_in_boxes_cpu
        base_data_dict = self.retrieve_base_data(idx)

        processed_data_dict = OrderedDict()
        processed_data_dict['ego'] = {}

        ego_id = -1
        ego_lidar_pose = []

        # first find the ego vehicle's lidar pose
        for cav_id, cav_content in base_data_dict.items():
            if cav_content['ego']:
                ego_id = cav_id
                ego_lidar_pose = cav_content['params']['lidar_pose']
                break
        assert cav_id == list(base_data_dict.keys())[
            0], "The first element in the OrderedDict must be ego"
        assert ego_id != -1
        assert len(ego_lidar_pose) > 0

        processed_features = []
        object_stack = []
        object_id_stack = []

        projected_lidar_stack = []

        # loop over all CAVs to process information
        for cav_id, selected_cav_base in base_data_dict.items():
            # check if the cav is within the communication range with ego
            distance = \
                math.sqrt((selected_cav_base['params']['lidar_pose'][0] -
                           ego_lidar_pose[0]) ** 2 + (
                                  selected_cav_base['params'][
                                      'lidar_pose'][1] - ego_lidar_pose[
                                      1]) ** 2)
            if distance > opencood.data_utils.datasets.COM_RANGE:
                continue

            selected_cav_processed = self.get_item_single_car(
                selected_cav_base,
                ego_lidar_pose)
            if len(selected_cav_processed['projected_lidar']) > 10:
                object_stack.append(
                    selected_cav_processed['object_bbx_center'])
                object_id_stack += selected_cav_processed['object_ids']
                processed_features.append(
                    selected_cav_processed['processed_features'])

                projected_lidar_stack.append(
                    selected_cav_processed['projected_lidar'])

        # exclude all repetitive objects
        unique_indices = \
            [object_id_stack.index(x) for x in set(object_id_stack)]
        object_stack_all = np.vstack(object_stack)
        object_stack_all = object_stack_all[unique_indices]

        # make sure bounding boxes across all frames have the same number
        object_bbx_center = \
            np.zeros((self.params['postprocess']['max_num'], 7))
        mask = np.zeros(self.params['postprocess']['max_num'])
        object_bbx_center[:object_stack_all.shape[0], :] = object_stack_all
        mask[:object_stack_all.shape[0]] = 1

        # merge preprocessed features from different cavs into the same dict
        cav_num = len(processed_features)
        merged_feature_dict = self.merge_features_to_dict(processed_features)

        # generate the anchor boxes
        anchor_box = self.post_processor.generate_anchor_box()

        # generate targets label
        label_dict = \
            self.post_processor.generate_label(
                gt_box_center=object_bbx_center, # hwl
                anchors=anchor_box,
                mask=mask)

        # Filter empty boxes
        object_stack_filtered = []
        label_dict_no_coop = []
        for boxes, points in zip(object_stack, projected_lidar_stack):
            point_indices = points_in_boxes_cpu(points[:, :3], boxes[:,
                                                               [0, 1, 2, 5, 4,
                                                                3, 6]])
            cur_mask = point_indices.sum(axis=1) > 0
            if cur_mask.sum() == 0:
                label_dict_no_coop.append({
                    'pos_equal_one': np.zeros((*anchor_box.shape[:2],
                                               self.post_processor.anchor_num)),
                    'neg_equal_one': np.ones((*anchor_box.shape[:2],
                                              self.post_processor.anchor_num)),
                    'targets': np.zeros((*anchor_box.shape[:2],
                                         self.post_processor.anchor_num * 7))
                })
                continue
            object_stack_filtered.append(boxes[cur_mask])
            bbx_center = \
                np.zeros((self.params['postprocess']['max_num'], 7))
            bbx_mask = np.zeros(self.params['postprocess']['max_num'])
            bbx_center[:boxes[cur_mask].shape[0], :] = boxes[cur_mask]
            bbx_mask[:boxes[cur_mask].shape[0]] = 1
            label_dict_no_coop.append(
                self.post_processor.generate_label(
                    gt_box_center=bbx_center, # hwl
                    anchors=anchor_box,
                    mask=bbx_mask)
            )
        label_dict = {
            'stage1': label_dict_no_coop,
            'stage2': label_dict
        }
        processed_data_dict['ego'].update(
            {'object_bbx_center': object_bbx_center,
             'object_bbx_mask': mask,
             'object_ids': [object_id_stack[i] for i in unique_indices],
             'anchor_box': anchor_box,
             'processed_lidar': merged_feature_dict,
             'label_dict': label_dict,
             'cav_num': cav_num})

        processed_data_dict['ego'].update({'origin_lidar':
                                               projected_lidar_stack})
        return processed_data_dict

    def get_item_single_car(self, selected_cav_base, ego_pose):
        """
        Project the lidar and bbx to ego space first, and then do clipping.

        Parameters
        ----------
        selected_cav_base : dict
            The dictionary contains a single CAV's raw information.
        ego_pose : list
            The ego vehicle lidar pose under world coordinate.

        Returns
        -------
        selected_cav_processed : dict
            The dictionary contains the cav's processed information.
        """
        selected_cav_processed = {}

        # calculate the transformation matrix
        transformation_matrix = \
            x1_to_x2(selected_cav_base['params']['lidar_pose'],
                     ego_pose)

        # retrieve objects under ego coordinates
        object_bbx_center, object_bbx_mask, object_ids = \
            self.post_processor.generate_object_center([selected_cav_base],
                                                       ego_pose)

        # filter lidar
        lidar_np = selected_cav_base['lidar_np']
        lidar_np = shuffle_points(lidar_np)
        # remove points that hit itself
        lidar_np = mask_ego_points(lidar_np)
        # project the lidar to ego space
        lidar_np[:, :3] = \
            box_utils.project_points_by_matrix_torch(lidar_np[:, :3],
                                                     transformation_matrix)
        lidar_np = mask_points_by_range(lidar_np,
                                        self.params['preprocess'][
                                            'cav_lidar_range'])
        processed_lidar = self.pre_processor.preprocess(lidar_np)

        selected_cav_processed.update(
            {'object_bbx_center': object_bbx_center[object_bbx_mask == 1],
             'object_ids': object_ids,
             'projected_lidar': lidar_np,
             'processed_features': processed_lidar})

        return selected_cav_processed

    @staticmethod
    def merge_features_to_dict(processed_feature_list):
        """
        Merge the preprocessed features from different cavs to the same
        dictionary.

        Parameters
        ----------
        processed_feature_list : list
            A list of dictionary containing all processed features from
            different cavs.

        Returns
        -------
        merged_feature_dict: dict
            key: feature names, value: list of features.
        """

        merged_feature_dict = OrderedDict()

        for i in range(len(processed_feature_list)):
            for feature_name, feature in processed_feature_list[i].items():
                if feature_name not in merged_feature_dict:
                    merged_feature_dict[feature_name] = []
                if isinstance(feature, list):
                    merged_feature_dict[feature_name] += feature
                else:
                    merged_feature_dict[feature_name].append(feature)

        return merged_feature_dict

    def collate_batch_train(self, batch):
        # Intermediate fusion is different the other two
        output_dict = {'ego': {}}

        object_bbx_center = []
        object_bbx_mask = []
        object_ids = []
        processed_lidar_list = []
        # used to record different scenario
        record_len = []
        label_dict_list = []
        label_dict_no_coop_list = []

        origin_lidar = []

        # added by yys, fpvrcnn needs anchors for
        # first stage proposal generation
        if batch[0]['ego']['anchor_box'] is not None:
            output_dict['ego'].update({'anchor_box':
                torch.from_numpy(np.array(
                    batch[0]['ego'][
                        'anchor_box']))})

        for i in range(len(batch)):
            ego_dict = batch[i]['ego']
            object_bbx_center.append(ego_dict['object_bbx_center'])
            object_bbx_mask.append(ego_dict['object_bbx_mask'])
            object_ids.append(ego_dict['object_ids'])

            processed_lidar_list.append(ego_dict['processed_lidar'])
            record_len.append(ego_dict['cav_num'])
            label_dict_no_coop_list.append(ego_dict['label_dict']['stage1'])
            label_dict_list.append(ego_dict['label_dict']['stage2'])

            origin_lidar.append(ego_dict['origin_lidar'])

        # convert to numpy, (B, max_num, 7)
        object_bbx_center = torch.from_numpy(np.array(object_bbx_center))
        object_bbx_mask = torch.from_numpy(np.array(object_bbx_mask))

        # example: {'voxel_features':[np.array([1,2,3]]),
        # np.array([3,5,6]), ...]}
        merged_feature_dict = self.merge_features_to_dict(processed_lidar_list)
        processed_lidar_torch_dict = \
            self.pre_processor.collate_batch(merged_feature_dict)
        # [2, 3, 4, ..., M], M <= 5
        record_len = torch.from_numpy(np.array(record_len, dtype=int))
        label_torch_dict = \
            self.post_processor.collate_batch(label_dict_list)
        label_dict_no_coop_list_ = [label_dict for label_list in
                                    label_dict_no_coop_list for label_dict in
                                    label_list]
        for i in range(len(label_dict_no_coop_list_)):
            if isinstance(label_dict_no_coop_list_[i], list):
                print('debug')
        label_no_coop_torch_dict = \
            self.post_processor.collate_batch(label_dict_no_coop_list_)
        # object id is only used during inference, where batch size is 1.
        # so here we only get the first element.
        output_dict['ego'].update({'object_bbx_center': object_bbx_center,
                                   'object_bbx_mask': object_bbx_mask,
                                   'processed_lidar': processed_lidar_torch_dict,
                                   'record_len': record_len,
                                   'label_dict': {
                                       'stage1': label_no_coop_torch_dict,
                                       'stage2': label_torch_dict},
                                   'object_ids': object_ids[0]})

        coords = []
        idx = 0
        for b in range(len(batch)):
            for points in origin_lidar[b]:
                assert len(points) != 0
                coor_pad = np.pad(points, ((0, 0), (1, 0)),
                                  mode="constant", constant_values=idx)
                coords.append(coor_pad)
                idx += 1
        origin_lidar = np.concatenate(coords, axis=0)

        origin_lidar = torch.from_numpy(origin_lidar)
        output_dict['ego'].update({'origin_lidar': origin_lidar})

        return output_dict

    def collate_batch_test(self, batch):
        assert len(batch) <= 1, "Batch size 1 is required during testing!"
        output_dict = self.collate_batch_train(batch)

        # check if anchor box in the batch
        if batch[0]['ego']['anchor_box'] is not None:
            output_dict['ego'].update({'anchor_box':
                torch.from_numpy(np.array(
                    batch[0]['ego'][
                        'anchor_box']))})

        # save the transformation matrix (4, 4) to ego vehicle
        transformation_matrix_torch = \
            torch.from_numpy(np.identity(4)).float()
        output_dict['ego'].update({'transformation_matrix':
                                       transformation_matrix_torch})

        return output_dict

    def post_process(self, data_dict, output_dict):
        """
        Process the outputs of the model to 2D/3D bounding box.

        Parameters
        ----------
        data_dict : dict
            The dictionary containing the origin input data of model.

        output_dict :dict
            The dictionary containing the output of the model.

        Returns
        -------
        pred_box_tensor : torch.Tensor
            The tensor of prediction bounding box after NMS.
        gt_box_tensor : torch.Tensor
            The tensor of gt bounding box.
        """
        pred_box_tensor, pred_score = \
            self.post_processor.post_process(data_dict, output_dict)
        gt_box_tensor = self.post_processor.generate_gt_bbx(data_dict)

        return pred_box_tensor, pred_score, gt_box_tensor

    def visualize_result(self, pred_box_tensor,
                         gt_tensor,
                         pcd,
                         show_vis,
                         save_path,
                         dataset=None):
        # we need to convert the pcd from [n, 5] -> [n, 4]
        pcd = pcd[:, 1:]
        # visualize the model output
        self.post_processor.visualize(pred_box_tensor,
                                      gt_tensor,
                                      pcd,
                                      show_vis,
                                      save_path,
                                      dataset=dataset)
