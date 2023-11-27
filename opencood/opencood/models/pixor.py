# -*- coding: utf-8 -*-
# Author: Hao Xiang <haxiang@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import math

import torch
import torch.nn as nn
import torch.nn.functional as F


def conv3x3(in_planes, out_planes, stride=1, bias=False):
    """3x3 convolution with padding"""
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding=1, bias=bias)


class BasicBlock(nn.Module):
    expansion = 1

    def __init__(self, in_planes, planes, stride=1, downsample=None):
        super(BasicBlock, self).__init__()
        self.conv1 = conv3x3(in_planes, planes, stride, bias=True)
        self.bn1 = nn.BatchNorm2d(planes)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = conv3x3(planes, planes, bias=True)
        self.bn2 = nn.BatchNorm2d(planes)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        residual = x

        out = self.conv1(x)
        # out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        # out = self.bn2(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)

        return out


class Bottleneck(nn.Module):
    expansion = 4

    def __init__(self, in_planes, planes, stride=1, downsample=None,
                 use_bn=True):
        super(Bottleneck, self).__init__()
        bias = not use_bn
        self.use_bn = use_bn
        self.conv1 = nn.Conv2d(in_planes, planes, kernel_size=1, bias=bias)
        self.bn1 = nn.BatchNorm2d(planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=stride,
                               padding=1, bias=bias)
        self.bn2 = nn.BatchNorm2d(planes)
        self.conv3 = nn.Conv2d(planes, self.expansion * planes, kernel_size=1,
                               bias=bias)
        self.bn3 = nn.BatchNorm2d(self.expansion * planes)
        self.downsample = downsample
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        """
        Forward pass of residual block.
        Parameters
        ----------
        x : torch.Tensor
            Shape (N, C, W, L).

        Returns
        -------
        out : torch.Tensor
            Shape (N, self.expansion*planes, W/stride, L/stride).
        """
        residual = x
        # (N, planes, W, L)
        out = self.conv1(x)
        if self.use_bn:
            out = self.bn1(out)
        out = self.relu(out)
        # (N, planes, W/stride, L/stride)
        out = self.conv2(out)
        if self.use_bn:
            out = self.bn2(out)
        out = self.relu(out)
        # (N, self.expansion*planes, W/stride, L/stride)
        out = self.conv3(out)
        if self.use_bn:
            out = self.bn3(out)

        if self.downsample is not None:
            # (N, self.expansion*planes, W/2, L/2)
            residual = self.downsample(x)
        out = self.relu(residual + out)
        return out


class BackBone(nn.Module):

    def __init__(self, block, num_block, geom, use_bn=True):
        super(BackBone, self).__init__()

        self.use_bn = use_bn

        # Block 1
        self.conv1 = conv3x3(geom["input_shape"][-1], 32)
        self.conv2 = conv3x3(32, 32)
        self.bn1 = nn.BatchNorm2d(32)
        self.bn2 = nn.BatchNorm2d(32)
        self.relu = nn.ReLU(inplace=True)

        # Block 2-5
        self.in_planes = 32
        self.block2 = self._make_layer(block, 24, num_blocks=num_block[0])
        self.block3 = self._make_layer(block, 48, num_blocks=num_block[1])
        self.block4 = self._make_layer(block, 64, num_blocks=num_block[2])
        self.block5 = self._make_layer(block, 96, num_blocks=num_block[3])

        # Lateral layers
        self.latlayer1 = nn.Conv2d(384, 196, kernel_size=1, stride=1,
                                   padding=0)
        self.latlayer2 = nn.Conv2d(256, 128, kernel_size=1, stride=1,
                                   padding=0)
        self.latlayer3 = nn.Conv2d(192, 96, kernel_size=1, stride=1, padding=0)

        # Top-down layers
        self.deconv1 = nn.ConvTranspose2d(196, 128, kernel_size=3, stride=2,
                                          padding=1, output_padding=1)
        p = 0 if geom['label_shape'][1] == 175 else 1
        self.deconv2 = nn.ConvTranspose2d(128, 96, kernel_size=3, stride=2,
                                          padding=1, output_padding=(1, p))

    def encode(self, x):
        x = self.conv1(x)
        if self.use_bn:
            x = self.bn1(x)
        x = self.relu(x)

        x = self.conv2(x)
        if self.use_bn:
            x = self.bn2(x)
        c1 = self.relu(x)

        # bottom up layers
        c2 = self.block2(c1)
        c3 = self.block3(c2)
        c4 = self.block4(c3)
        c5 = self.block5(c4)

        return c3, c4, c5

    def decode(self, c3, c4, c5):
        l5 = self.latlayer1(c5)
        l4 = self.latlayer2(c4)
        p5 = l4 + self.deconv1(l5)
        l3 = self.latlayer3(c3)
        p4 = l3 + self.deconv2(p5)

        return p4

    def forward(self, x):
        c3, c4, c5 = self.encode(x)
        p4 = self.decode(c3, c4, c5)

        return p4

    def _make_layer(self, block, planes, num_blocks):

        if self.use_bn:
            # downsample the H*W by 1/2
            downsample = nn.Sequential(
                nn.Conv2d(self.in_planes, planes * block.expansion,
                          kernel_size=1, stride=2, bias=False),
                nn.BatchNorm2d(planes * block.expansion)
            )
        else:
            downsample = nn.Conv2d(self.in_planes, planes * block.expansion,
                                   kernel_size=1, stride=2, bias=True)

        layers = [
            block(self.in_planes, planes, stride=2, downsample=downsample)]

        self.in_planes = planes * block.expansion
        for i in range(1, num_blocks):
            layers.append(block(self.in_planes, planes, stride=1))
            self.in_planes = planes * block.expansion
        return nn.Sequential(*layers)

    def _upsample_add(self, x, y):
        """Upsample and add two feature maps.
        Args:
          x: (Variable) top feature map to be upsampled.
          y: (Variable) lateral feature map.
        Returns:
          (Variable) added feature map.
        Note in PyTorch, when input size is odd, the upsampled feature map
        with `F.upsample(..., scale_factor=2, mode='nearest')`
        maybe not equal to the lateral feature map size.
        e.g.
        original input size: [N,_,15,15] ->
        conv2d feature map size: [N,_,8,8] ->
        upsampled feature map size: [N,_,16,16]
        So we choose bilinear upsample which supports arbitrary output sizes.
        """
        _, _, H, W = y.size()
        return F.upsample(x, size=(H, W), mode='bilinear') + y


class Header(nn.Module):

    def __init__(self, use_bn=True):
        super(Header, self).__init__()

        self.use_bn = use_bn
        bias = not use_bn
        self.conv1 = conv3x3(96, 96, bias=bias)
        self.bn1 = nn.BatchNorm2d(96)
        self.conv2 = conv3x3(96, 96, bias=bias)
        self.bn2 = nn.BatchNorm2d(96)
        self.conv3 = conv3x3(96, 96, bias=bias)
        self.bn3 = nn.BatchNorm2d(96)
        self.conv4 = conv3x3(96, 96, bias=bias)
        self.bn4 = nn.BatchNorm2d(96)

        self.clshead = conv3x3(96, 1, bias=True)
        self.reghead = conv3x3(96, 6, bias=True)

    def forward(self, x):
        x = self.conv1(x)
        if self.use_bn:
            x = self.bn1(x)
        x = self.conv2(x)
        if self.use_bn:
            x = self.bn2(x)
        x = self.conv3(x)
        if self.use_bn:
            x = self.bn3(x)
        x = self.conv4(x)
        if self.use_bn:
            x = self.bn4(x)

        cls = self.clshead(x)
        reg = self.reghead(x)

        return cls, reg


class PIXOR(nn.Module):
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
        super(PIXOR, self).__init__()
        geom = args["geometry_param"]
        use_bn = args["use_bn"]
        self.backbone = BackBone(Bottleneck, [3, 6, 6, 3], geom, use_bn)
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

        features = self.backbone(bev_input)
        # cls -- (N, 1, W/4, L/4)
        # reg -- (N, 6, W/4, L/4)
        cls, reg = self.header(features)

        output_dict = {
            "cls": cls,
            "reg": reg
        }

        return output_dict
