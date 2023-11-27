"""
Class used to downsample features by 3*3 conv
"""
import torch.nn as nn


class DoubleConv(nn.Module):
    """
    Double convoltuion
    Args:
        in_channels: input channel num
        out_channels: output channel num
    """

    def __init__(self, in_channels, out_channels, kernel_size,
                 stride, padding):
        super().__init__()
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=kernel_size,
                      stride=stride, padding=padding),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.double_conv(x)


class DownsampleConv(nn.Module):
    def __init__(self, config):
        super(DownsampleConv, self).__init__()
        self.layers = nn.ModuleList([])
        input_dim = config['input_dim']

        for (ksize, dim, stride, padding) in zip(config['kernal_size'],
                                                 config['dim'],
                                                 config['stride'],
                                                 config['padding']):
            self.layers.append(DoubleConv(input_dim,
                                          dim,
                                          kernel_size=ksize,
                                          stride=stride,
                                          padding=padding))
            input_dim = dim

    def forward(self, x):
        for i in range(len(self.layers)):
            x = self.layers[i](x)
        return x