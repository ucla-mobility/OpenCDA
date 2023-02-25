import torch
import torch.nn as nn


class AutoEncoder(nn.Module):
    def __init__(self, feature_num, layer_num):
        super().__init__()
        self.feature_num = feature_num
        self.feature_stride = 2

        self.encoder = nn.ModuleList()
        self.decoder = nn.ModuleList()

        for i in range(layer_num):
            cur_layers = [
                nn.ZeroPad2d(1),
                nn.Conv2d(
                    feature_num, feature_num, kernel_size=3,
                    stride=2, padding=0, bias=False
                ),
                nn.BatchNorm2d(feature_num, eps=1e-3, momentum=0.01),
                nn.ReLU()]

            cur_layers.extend([
                nn.Conv2d(feature_num, feature_num // self.feature_stride,
                          kernel_size=3, padding=1, bias=False),
                nn.BatchNorm2d(feature_num // self.feature_stride,
                               eps=1e-3, momentum=0.01),
                nn.ReLU()
            ])

            self.encoder.append(nn.Sequential(*cur_layers))
            feature_num = feature_num // self.feature_stride

        feature_num = self.feature_num
        for i in range(layer_num):
            cur_layers = [nn.Sequential(
                nn.ConvTranspose2d(
                    feature_num // 2, feature_num,
                    kernel_size=2,
                    stride=2, bias=False
                ),
                nn.BatchNorm2d(feature_num,
                               eps=1e-3, momentum=0.01),
                nn.ReLU()
            )]

            cur_layers.extend([nn.Sequential(
                nn.Conv2d(
                    feature_num, feature_num, kernel_size=3,
                    stride=1, bias=False, padding=1
                ),
                nn.BatchNorm2d(feature_num, eps=1e-3,
                               momentum=0.01),
                nn.ReLU()
            )])
            self.decoder.append(nn.Sequential(*cur_layers))
            feature_num //= 2

    def forward(self, x):
        for i in range(len(self.encoder)):
            x = self.encoder[i](x)

        for i in range(len(self.decoder)-1, -1, -1):
            x = self.decoder[i](x)

        return x