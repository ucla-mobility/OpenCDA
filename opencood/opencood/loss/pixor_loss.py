# -*- coding: utf-8 -*-
# Author: Hao Xiang <haxiang@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


from functools import reduce

import torch
import torch.nn as nn
import torch.nn.functional as F


class PixorLoss(nn.Module):
    def __init__(self, args):
        super(PixorLoss, self).__init__()
        self.alpha = args["alpha"]
        self.beta = args["beta"]
        self.loss_dict = {}

    def forward(self, output_dict, target_dict):
        """
        Compute loss for pixor network
        Parameters
        ----------
        output_dict : dict
           The dictionary that contains the output.

        target_dict : dict
           The dictionary that contains the target.

        Returns
        -------
        total_loss : torch.Tensor
            Total loss.

        """
        targets = target_dict["label_map"]
        cls_preds, loc_preds = output_dict["cls"], output_dict["reg"]

        cls_targets, loc_targets = targets.split([1, 6], dim=1)
        pos_count = cls_targets.sum()
        neg_count = (cls_targets == 0).sum()
        w1, w2 = neg_count / (pos_count + neg_count), pos_count / (
                    pos_count + neg_count)
        weights = torch.ones_like(cls_preds.reshape(-1))
        weights[cls_targets.reshape(-1) == 1] = w1
        weights[cls_targets.reshape(-1) == 0] = w2
        # cls_targets = cls_targets.float()
        # cls_loss = F.binary_cross_entropy_with_logits(input=cls_preds.reshape(-1), target=cls_targets.reshape(-1), weight=weights,
        #                                               reduction='mean')
        cls_loss = F.binary_cross_entropy_with_logits(
            input=cls_preds, target=cls_targets,
            reduction='mean')
        pos_pixels = cls_targets.sum()

        loc_loss = F.smooth_l1_loss(cls_targets * loc_preds,
                                    cls_targets * loc_targets,
                                    reduction='sum')
        loc_loss = loc_loss / pos_pixels if pos_pixels > 0 else loc_loss

        total_loss = self.alpha * cls_loss + self.beta * loc_loss

        self.loss_dict.update({'total_loss': total_loss,
                               'reg_loss': loc_loss,
                               'cls_loss': cls_loss})

        return total_loss

    def logging(self, epoch, batch_id, batch_len, writer, pbar=None):
        """
        Print out  the loss function for current iteration.

        Parameters
        ----------
        epoch : int
            Current epoch for training.
        batch_id : int
            The current batch.
        batch_len : int
            Total batch length in one iteration of training,
        writer : SummaryWriter
            Used to visualize on tensorboard
        """
        total_loss = self.loss_dict['total_loss']
        reg_loss = self.loss_dict['reg_loss']
        cls_loss = self.loss_dict['cls_loss']

        if pbar is None:
            print("[epoch %d][%d/%d], || Loss: %.4f || Conf Loss: %.4f"
                " || Loc Loss: %.4f" % (
                    epoch, batch_id + 1, batch_len,
                    total_loss.item(), cls_loss.item(), reg_loss.item()))
        else:
            pbar.set_description("[epoch %d][%d/%d], || Loss: %.4f || Conf Loss: %.4f"
                  " || Loc Loss: %.4f" % (
                      epoch, batch_id + 1, batch_len,
                      total_loss.item(), cls_loss.item(), reg_loss.item()))

        writer.add_scalar('Regression_loss', reg_loss.item(),
                          epoch * batch_len + batch_id)
        writer.add_scalar('Confidence_loss', cls_loss.item(),
                          epoch * batch_len + batch_id)


def test():
    torch.manual_seed(0)
    loss = PixorLoss(None)
    pred = torch.sigmoid(torch.randn(1, 7, 2, 3))
    label = torch.zeros(1, 7, 2, 3)
    loss = loss(pred, label)
    print(loss)


if __name__ == "__main__":
    test()
