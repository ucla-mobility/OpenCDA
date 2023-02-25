# -*- coding: utf-8 -*-
# Author: Hao Xiang <haxiang@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


"""
Convert lidar to bev
"""

import numpy as np
import torch
from opencood.data_utils.pre_processor.base_preprocessor import \
    BasePreprocessor


class BevPreprocessor(BasePreprocessor):
    def __init__(self, preprocess_params, train):
        super(BevPreprocessor, self).__init__(preprocess_params, train)
        self.lidar_range = self.params['cav_lidar_range']
        self.geometry_param = preprocess_params["geometry_param"]

    def preprocess(self, pcd_raw):
        """
        Preprocess the lidar points to BEV representations.

        Parameters
        ----------
        pcd_raw : np.ndarray
            The raw lidar.

        Returns
        -------
        data_dict : the structured output dictionary.
        """
        bev = np.zeros(self.geometry_param['input_shape'], dtype=np.float32)
        intensity_map_count = np.zeros((bev.shape[0], bev.shape[1]),
                                       dtype=np.int)
        bev_origin = np.array(
            [self.geometry_param["L1"], self.geometry_param["W1"],
             self.geometry_param["H1"]]).reshape(1, -1)

        indices = ((pcd_raw[:, :3] - bev_origin) / self.geometry_param[
            "res"]).astype(int)

        for i in range(indices.shape[0]):
            bev[indices[i, 0], indices[i, 1], indices[i, 2]] = 1
            bev[indices[i, 0], indices[i, 1], -1] += pcd_raw[i, 3]
            intensity_map_count[indices[i, 0], indices[i, 1]] += 1
        divide_mask = intensity_map_count != 0
        bev[divide_mask, -1] = np.divide(bev[divide_mask, -1],
                                         intensity_map_count[divide_mask])

        data_dict = {
            "bev_input": np.transpose(bev, (2, 0, 1))
        }
        return data_dict

    @staticmethod
    def collate_batch_list(batch):
        """
        Customized pytorch data loader collate function.

        Parameters
        ----------
        batch : list
            List of dictionary. Each dictionary represent a single frame.

        Returns
        -------
        processed_batch : dict
            Updated lidar batch.
        """
        bev_input_list = [
            x["bev_input"][np.newaxis, ...] for x in batch
        ]
        processed_batch = {
            "bev_input": torch.from_numpy(
                np.concatenate(bev_input_list, axis=0))
        }
        return processed_batch

    @staticmethod
    def collate_batch_dict(batch):
        """
        Customized pytorch data loader collate function.

        Parameters
        ----------
        batch : dict
            Dict of list. Each element represents a CAV.

        Returns
        -------
        processed_batch : dict
            Updated lidar batch.
        """
        bev_input_list = [
            x[np.newaxis, ...] for x in batch["bev_input"]
        ]
        processed_batch = {
            "bev_input": torch.from_numpy(
                np.concatenate(bev_input_list, axis=0))
        }
        return processed_batch

    def collate_batch(self, batch):
        """
        Customized pytorch data loader collate function.

        Parameters
        ----------
        batch : list / dict
            Batched data.
        Returns
        -------
        processed_batch : dict
            Updated lidar batch.
        """
        if isinstance(batch, list):
            return self.collate_batch_list(batch)
        elif isinstance(batch, dict):
            return self.collate_batch_dict(batch)
        else:
            raise NotImplemented
