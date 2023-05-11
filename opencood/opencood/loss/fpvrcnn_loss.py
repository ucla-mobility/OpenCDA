import torch
from torch import nn
import numpy as np
from opencood.loss.ciassd_loss import CiassdLoss, weighted_smooth_l1_loss


class FpvrcnnLoss(nn.Module):
    def __init__(self, args):
        super(FpvrcnnLoss, self).__init__()
        self.ciassd_loss = CiassdLoss(args['stage1'])
        self.cls = args['stage2']['cls']
        self.reg = args['stage2']['reg']
        self.iou = args['stage2']['iou']
        self.loss_dict = {}

    def forward(self, output_dict, label_dict):
        """
        Parameters
        ----------
        output_dict : dict
        target_dict : dict
        """
        ciassd_loss = self.ciassd_loss(output_dict, label_dict)

        # only update ciassd if no bbox is detected in the first stage
        if 'fpvrcnn_out' not in output_dict:
            self.loss_dict = {
                'loss': ciassd_loss,
            }
            return ciassd_loss

        # rcnn out
        rcnn_cls = output_dict['fpvrcnn_out']['rcnn_cls'].view(1, -1, 1)
        rcnn_iou = output_dict['fpvrcnn_out']['rcnn_iou'].view(1, -1, 1)
        rcnn_reg = output_dict['fpvrcnn_out']['rcnn_reg'].view(1, -1, 7)

        tgt_cls = output_dict['rcnn_label_dict']['cls_tgt'].view(1, -1, 1)
        tgt_iou = output_dict['rcnn_label_dict']['iou_tgt'].view(1, -1, 1)
        tgt_reg = output_dict['rcnn_label_dict']['reg_tgt'].view(1, -1, 7)

        pos_norm = tgt_cls.sum()
        # cls loss
        loss_cls = weighted_sigmoid_binary_cross_entropy(rcnn_cls, tgt_cls)

        # iou loss
        # TODO: also count the negative samples
        loss_iou = weighted_smooth_l1_loss(rcnn_iou, tgt_iou,
                                           weights=tgt_cls).mean()

        # regression loss
        # Target resampling : Generate a weights mask to force the regressor concentrate on low iou predictions
        # sample 50% with iou>0.7 and 50% < 0.7
        weights = torch.ones(tgt_iou.shape, device=tgt_iou.device)
        weights[tgt_cls == 0] = 0
        neg = torch.logical_and(tgt_iou < 0.7, tgt_cls != 0)
        pos = torch.logical_and(tgt_iou >= 0.7, tgt_cls != 0)
        num_neg = int(neg.sum(dim=1))
        num_pos = int(pos.sum(dim=1))
        num_pos_smps = max(num_neg, 2)
        pos_indices = torch.where(pos)[1]
        not_selsected = torch.randperm(num_pos)[:num_pos - num_pos_smps]
        # not_selsected_indices = pos_indices[not_selsected]
        weights[:, pos_indices[not_selsected]] = 0
        loss_reg = weighted_smooth_l1_loss(rcnn_reg, tgt_reg,
                                           weights=weights / max(weights.sum(),
                                                                 1)).sum()

        loss_cls_reduced = loss_cls * self.cls['weight']
        loss_iou_reduced = loss_iou * self.iou['weight']
        loss_reg_reduced = loss_reg * self.reg['weight']

        # if torch.isnan(loss_reg_reduced):
        #     print('debug')

        rcnn_loss = loss_cls_reduced + loss_iou_reduced + loss_reg_reduced
        loss = rcnn_loss + ciassd_loss

        self.loss_dict.update({
            'loss': loss,
            'rcnn_loss': rcnn_loss,
            'cls_loss': loss_cls_reduced,
            'iou_loss': loss_iou_reduced,
            'reg_loss': loss_reg_reduced,
        })

        return loss

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
        ciassd_loss_dict = self.ciassd_loss.loss_dict
        ciassd_total_loss = ciassd_loss_dict['total_loss']
        reg_loss = ciassd_loss_dict['reg_loss']
        cls_loss = ciassd_loss_dict['cls_loss']
        dir_loss = ciassd_loss_dict['dir_loss']
        iou_loss = ciassd_loss_dict['iou_loss']

        if (batch_id + 1) % 10 == 0:
            str_to_print = "[epoch %d][%d/%d], || Loss: %.4f || Ciassd: %.4f " \
                           "|| Cls1: %.4f || Loc1: %.4f || Dir1: %.4f || Iou1: %.4f" % (
                               epoch, batch_id + 1, batch_len, self.loss_dict['loss'],
                               ciassd_total_loss.item(), cls_loss.item(), reg_loss.item(),
                               dir_loss.item(), iou_loss.item(),
                               )
            if 'rcnn_loss' in self.loss_dict:
                str_to_print += " || Rcnn: %.4f || Cls2: %.4f || Loc2: %.4f || Iou2: %.4f" % (
                        self.loss_dict['rcnn_loss'],
                        self.loss_dict['cls_loss'].item(),
                        self.loss_dict['reg_loss'].item(),
                        self.loss_dict['iou_loss'].item(),
                    )
            print(str_to_print)

        writer.add_scalar('Ciassd_regression_loss', reg_loss.item(),
                          epoch * batch_len + batch_id)
        writer.add_scalar('Ciassd_Confidence_loss', cls_loss.item(),
                          epoch * batch_len + batch_id)
        writer.add_scalar('Ciassd_Direction_loss', dir_loss.item(),
                          epoch * batch_len + batch_id)
        writer.add_scalar('Ciassd_Iou_loss', iou_loss.item(),
                          epoch * batch_len + batch_id)
        writer.add_scalar('Ciassd_loss', ciassd_total_loss.item(),
                          epoch * batch_len + batch_id)
        if 'rcnn_loss' in self.loss_dict:
            writer.add_scalar('Rcnn_regression_loss',
                              self.loss_dict['reg_loss'].item(),
                              epoch * batch_len + batch_id)
            writer.add_scalar('Rcnn_Confidence_loss',
                              self.loss_dict['cls_loss'].item(),
                              epoch * batch_len + batch_id)
            writer.add_scalar('Rcnn_Iou_loss',
                              self.loss_dict['iou_loss'].item(),
                              epoch * batch_len + batch_id)
            writer.add_scalar('Rcnn_loss', self.loss_dict['rcnn_loss'].item(),
                              epoch * batch_len + batch_id)
            writer.add_scalar('Total_loss', self.loss_dict['loss'].item(),
                              epoch * batch_len + batch_id)


def weighted_sigmoid_binary_cross_entropy(preds, tgts, weights=None,
                                          class_indices=None):
    if weights is not None:
        weights = weights.unsqueeze(-1)
    if class_indices is not None:
        weights *= (
            indices_to_dense_vector(class_indices, preds.shape[2])
                .view(1, 1, -1)
                .type_as(preds)
        )
    per_entry_cross_ent = nn.functional.binary_cross_entropy_with_logits(preds,
                                                                         tgts,
                                                                         weights)
    return per_entry_cross_ent


def indices_to_dense_vector(
        indices, size, indices_value=1.0, default_value=0, dtype=np.float32
):
    """Creates dense vector with indices set to specific value and rest to zeros.

    This function exists because it is unclear if it is safe to use
        tf.sparse_to_dense(indices, [size], 1, validate_indices=False)
    with indices which are not ordered.
    This function accepts a dynamic size (e.g. tf.shape(tensor)[0])

    Args:
        indices: 1d Tensor with integer indices which are to be set to
            indices_values.
        size: scalar with size (integer) of output Tensor.
        indices_value: values of elements specified by indices in the output vector
        default_value: values of other elements in the output vector.
        dtype: data type.

    Returns:
        dense 1D Tensor of shape [size] with indices set to indices_values and the
            rest set to default_value.
    """
    dense = torch.zeros(size).fill_(default_value)
    dense[indices] = indices_value

    return dense
