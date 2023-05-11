import torch
from torch import nn



class SSFA(nn.Module):
    def __init__(self, args):
        super(SSFA, self).__init__()
        self._num_input_features = args['feature_num']  # 128

        seq = [nn.ZeroPad2d(1)] + get_conv_layers('Conv2d', 128, 128, n_layers=3, kernel_size=[3, 3, 3],
                                                  stride=[1, 1, 1], padding=[0, 1, 1], sequential=False)
        self.bottom_up_block_0 = nn.Sequential(*seq)
        self.bottom_up_block_1 = get_conv_layers('Conv2d', 128, 256, n_layers=3, kernel_size=[3, 3, 3],
                                                  stride=[2, 1, 1], padding=[1, 1, 1])

        self.trans_0 = get_conv_layers('Conv2d', 128, 128, n_layers=1, kernel_size=[1], stride=[1], padding=[0])
        self.trans_1 = get_conv_layers('Conv2d', 256, 256, n_layers=1, kernel_size=[1], stride=[1], padding=[0])

        self.deconv_block_0 = get_conv_layers('ConvTranspose2d', 256, 128, n_layers=1, kernel_size=[3], stride=[2],
                                              padding=[1], output_padding=[1])
        self.deconv_block_1 = get_conv_layers('ConvTranspose2d', 256, 128, n_layers=1, kernel_size=[3], stride=[2],
                                              padding=[1], output_padding=[1])

        self.conv_0 = get_conv_layers('Conv2d', 128, 128, n_layers=1, kernel_size=[3], stride=[1], padding=[1])
        self.conv_1 = get_conv_layers('Conv2d', 128, 128, n_layers=1, kernel_size=[3], stride=[1], padding=[1])

        self.w_0 = get_conv_layers('Conv2d', 128, 1, n_layers=1, kernel_size=[1], stride=[1], padding=[0], relu_last=False)
        self.w_1 = get_conv_layers('Conv2d', 128, 1, n_layers=1, kernel_size=[1], stride=[1], padding=[0], relu_last=False)

    # default init_weights for conv(msra) and norm in ConvModule
    def init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.xavier_normal_(m.weight, gain=1)
                if hasattr(m, "bias") and m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, x):
        x_0 = self.bottom_up_block_0(x)
        x_1 = self.bottom_up_block_1(x_0)
        x_trans_0 = self.trans_0(x_0)
        x_trans_1 = self.trans_1(x_1)
        x_middle_0 = self.deconv_block_0(x_trans_1) + x_trans_0
        x_middle_1 = self.deconv_block_1(x_trans_1)
        x_output_0 = self.conv_0(x_middle_0)
        x_output_1 = self.conv_1(x_middle_1)

        x_weight_0 = self.w_0(x_output_0)
        x_weight_1 = self.w_1(x_output_1)
        x_weight = torch.softmax(torch.cat([x_weight_0, x_weight_1], dim=1), dim=1)
        x_output = x_output_0 * x_weight[:, 0:1, :, :] + x_output_1 * x_weight[:, 1:, :, :]

        return x_output.contiguous()


def get_conv_layers(conv_name, in_channels, out_channels, n_layers, kernel_size, stride,
                    padding, relu_last=True, sequential=True, **kwargs):
    """
    Build convolutional layers. kernel_size, stride and padding should be a list with the lengths that match n_layers
    """
    seq = []
    for i in range(n_layers):
        seq.extend([getattr(nn, conv_name)(in_channels, out_channels, kernel_size[i], stride=stride[i],
                                           padding=padding[i], bias=False, **{k: v[i] for k, v in kwargs.items()}),
                    nn.BatchNorm2d(out_channels, eps=1e-3, momentum=0.01)])
        if i < n_layers - 1 or relu_last:
            seq.append(nn.ReLU())
        in_channels = out_channels
    if sequential:
        return nn.Sequential(*seq)
    else:
        return seq


class Head(nn.Module):
    def __init__(self, num_input, num_pred, num_cls, num_iou=2, use_dir=False, num_dir=1):
        super(Head, self).__init__()
        self.use_dir = use_dir

        self.conv_box = nn.Conv2d(num_input, num_pred, 1)  # 128 -> 14
        self.conv_cls = nn.Conv2d(num_input, num_cls, 1)   # 128 -> 2
        self.conv_iou = nn.Conv2d(num_input, num_iou, 1, bias=False)

        if self.use_dir:
            self.conv_dir = nn.Conv2d(num_input, num_dir, 1)  # 128 -> 4

    def forward(self, x):
        box_preds = self.conv_box(x)
        cls_preds = self.conv_cls(x)
        ret_dict = {"box_preds": box_preds, "cls_preds": cls_preds}
        if self.use_dir:
            dir_preds = self.conv_dir(x)  # dir_preds.shape=[8, w, h, 4]
            ret_dict["dir_cls_preds"] = dir_preds
        else:
            ret_dict["dir_cls_preds"] = torch.zeros((len(box_preds), 1, 2))

        ret_dict["iou_preds"] = self.conv_iou(x)

        return ret_dict