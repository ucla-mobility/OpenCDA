# -*- coding: utf-8 -*-
# Author: Hao Xiang <haxiang@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


"""
Implementation of V2VNet Fusion
"""

import torch
import torch.nn as nn

from opencood.models.sub_modules.torch_transformation_utils import \
    get_discretized_transformation_matrix, get_transformation_matrix, \
    warp_affine, get_rotated_roi
from opencood.models.sub_modules.convgru import ConvGRU


class V2VNetFusion(nn.Module):
    def __init__(self, args):
        super(V2VNetFusion, self).__init__()
        
        in_channels = args['in_channels']
        H, W = args['conv_gru']['H'], args['conv_gru']['W']
        kernel_size = args['conv_gru']['kernel_size']
        num_gru_layers = args['conv_gru']['num_layers']

        self.discrete_ratio = args['voxel_size'][0]
        self.downsample_rate = args['downsample_rate']
        self.num_iteration = args['num_iteration']
        self.gru_flag = args['gru_flag']
        self.agg_operator = args['agg_operator']

        self.msg_cnn = nn.Conv2d(in_channels * 2, in_channels, kernel_size=3,
                                 stride=1, padding=1)
        self.conv_gru = ConvGRU(input_size=(H, W),
                                input_dim=in_channels * 2,
                                hidden_dim=[in_channels],
                                kernel_size=kernel_size,
                                num_layers=num_gru_layers,
                                batch_first=True,
                                bias=True,
                                return_all_layers=False)
        self.mlp = nn.Linear(in_channels, in_channels)

    def regroup(self, x, record_len):
        cum_sum_len = torch.cumsum(record_len, dim=0)
        split_x = torch.tensor_split(x, cum_sum_len[:-1].cpu())
        return split_x

    def forward(self, x, record_len, pairwise_t_matrix):
        """
        Fusion forwarding.
        
        Parameters
        ----------
        x : torch.Tensor
            input data, (B, C, H, W)
            
        record_len : list
            shape: (B)
            
        pairwise_t_matrix : torch.Tensor
            The transformation matrix from each cav to ego, 
            shape: (B, L, L, 4, 4) 
            
        Returns
        -------
        Fused feature.
        """
        _, C, H, W = x.shape
        B, L = pairwise_t_matrix.shape[:2]

        # split x:[(L1, C, H, W), (L2, C, H, W)]
        split_x = self.regroup(x, record_len)
        # (B,L,L,2,3)
        pairwise_t_matrix = get_discretized_transformation_matrix(
            pairwise_t_matrix.reshape(-1, L, 4, 4), self.discrete_ratio,
            self.downsample_rate).reshape(B, L, L, 2, 3)
        # (B*L,L,1,H,W)
        roi_mask = get_rotated_roi((B * L, L, 1, H, W),
                                   pairwise_t_matrix.reshape(B * L * L, 2, 3))
        roi_mask = roi_mask.reshape(B, L, L, 1, H, W)
        batch_node_features = split_x
        
        # iteratively update the features for num_iteration times
        for l in range(self.num_iteration):

            batch_updated_node_features = []
            # iterate each batch
            for b in range(B):

                # number of valid agent
                N = record_len[b]
                # (N,N,4,4)
                # t_matrix[i, j]-> from i to j
                t_matrix = pairwise_t_matrix[b][:N, :N, :, :]
                updated_node_features = []
                # update each node i
                for i in range(N):
                    # (N,1,H,W)
                    mask = roi_mask[b, :N, i, ...]

                    current_t_matrix = t_matrix[:, i, :, :]
                    current_t_matrix = get_transformation_matrix(
                        current_t_matrix, (H, W))

                    # (N,C,H,W)
                    neighbor_feature = warp_affine(batch_node_features[b],
                                                   current_t_matrix,
                                                   (H, W))
                    # (N,C,H,W)
                    ego_agent_feature = batch_node_features[b][i].unsqueeze(
                        0).repeat(N, 1, 1, 1)
                    #(N,2C,H,W)
                    neighbor_feature = torch.cat(
                        [neighbor_feature, ego_agent_feature], dim=1)
                    # (N,C,H,W)
                    message = self.msg_cnn(neighbor_feature) * mask

                    # (C,H,W)
                    if self.agg_operator=="avg":
                        agg_feature = torch.mean(message, dim=0)
                    elif self.agg_operator=="max":
                        agg_feature = torch.max(message, dim=0)[0]
                    else:
                        raise ValueError("agg_operator has wrong value")
                    # (2C, H, W)
                    cat_feature = torch.cat(
                        [batch_node_features[b][i, ...], agg_feature], dim=0)
                    # (C,H,W)
                    if self.gru_flag:
                        gru_out = \
                            self.conv_gru(cat_feature.unsqueeze(0).unsqueeze(0))[
                                0][
                                0].squeeze(0).squeeze(0)
                    else:
                        gru_out = batch_node_features[b][i, ...] + agg_feature
                    updated_node_features.append(gru_out.unsqueeze(0))
                # (N,C,H,W)
                batch_updated_node_features.append(
                    torch.cat(updated_node_features, dim=0))
            batch_node_features = batch_updated_node_features
        # (B,C,H,W)
        out = torch.cat(
            [itm[0, ...].unsqueeze(0) for itm in batch_node_features], dim=0)
        # (B,C,H,W)
        out = self.mlp(out.permute(0, 2, 3, 1)).permute(0, 3, 1, 2)

        return out
