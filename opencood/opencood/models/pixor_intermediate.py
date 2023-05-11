# -*- coding: utf-8 -*-
# Author: Hao Xiang <haxiang@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import math

import torch.nn as nn

from opencood.models.fuse_modules.self_attn import AttFusion
from opencood.models.pixor import Bottleneck, BackBone, Header


class BackBoneIntermediate(BackBone):

    def __init__(self, block, num_block, geom, use_bn=True):
        super(BackBoneIntermediate, self).__init__(block,
                                                   num_block,
                                                   geom, use_bn)

        self.fusion_net3 = AttFusion(192)
        self.fusion_net4 = AttFusion(256)
        self.fusion_net5 = AttFusion(384)

    def forward(self, x, record_len):
        c3, c4, c5 = self.encode(x)

        c5 = self.fusion_net5(c5, record_len)
        c4 = self.fusion_net4(c4, record_len)
        c3 = self.fusion_net3(c3, record_len)

        p4 = self.decode(c3, c4, c5)
        return p4


class PIXORIntermediate(nn.Module):
    """
    The Pixor backbone. The input of PIXOR nn module is a tensor of
    [batch_size, height, weight, channel], The output of PIXOR nn module
    is also a tensor of [batch_size, height/4, weight/4, channel].  Note that
     we convert the dimensions to [C, H, W] for PyTorch's nn.Conv2d functions

    Parameters
    ----------
    args : dict
        The arguments of the model.

    Attributes
    ----------
    backbone : opencood.object
        The backbone used to extract features.
    header : opencood.object
        Header used to predict the classification and coordinates.
    """

    def __init__(self, args):
        super(PIXORIntermediate, self).__init__()
        geom = args["geometry_param"]
        use_bn = args["use_bn"]
        self.backbone = BackBoneIntermediate(Bottleneck, [3, 6, 6, 3],
                                             geom,
                                             use_bn)
        self.header = Header(use_bn)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
                m.weight.data.normal_(0, math.sqrt(2. / n))
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()

        prior = 0.01
        self.header.clshead.weight.data.fill_(-math.log((1.0 - prior) / prior))
        self.header.clshead.bias.data.fill_(0)
        self.header.reghead.weight.data.fill_(0)
        self.header.reghead.bias.data.fill_(0)

    def forward(self, data_dict):
        bev_input = data_dict['processed_lidar']["bev_input"]
        record_len = data_dict['record_len']

        features = self.backbone(bev_input, record_len)
        # cls -- (N, 1, W/4, L/4)
        # reg -- (N, 6, W/4, L/4)
        cls, reg = self.header(features)

        output_dict = {
            "cls": cls,
            "reg": reg
        }

        return output_dict
