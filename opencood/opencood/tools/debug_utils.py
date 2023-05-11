# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>, Hao Xiang <haxiang@g.ucla.edu>,
# License: TDG-Attribution-NonCommercial-NoDistrib


import argparse

import torch
from torch.utils.data import DataLoader

import opencood.hypes_yaml.yaml_utils as yaml_utils
from opencood.tools import train_utils
from opencood.data_utils.datasets import build_dataset
from opencood.visualization import vis_utils


def test_parser():
    parser = argparse.ArgumentParser(description="synthetic data generation")
    parser.add_argument('--model_dir', type=str, required=True,
                        help='Continued training path')
    parser.add_argument('--fusion_method', type=str, default='late',
                        help='late, early or intermediate')
    opt = parser.parse_args()
    return opt


def test_bev_post_processing():
    opt = test_parser()
    assert opt.fusion_method in ['late', 'early', 'intermediate']

    hypes = yaml_utils.load_yaml(None, opt)

    print('Dataset Building')
    opencood_dataset = build_dataset(hypes, visualize=True, train=False)
    data_loader = DataLoader(opencood_dataset,
                             batch_size=1,
                             num_workers=0,
                             collate_fn=opencood_dataset.collate_batch_test,
                             shuffle=False,
                             pin_memory=False,
                             drop_last=False)

    print('Creating Model')
    model = train_utils.create_model(hypes)
    # we assume gpu is necessary
    if torch.cuda.is_available():
        model.cuda()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    print('Loading Model from checkpoint')
    saved_path = opt.model_dir
    _, model = train_utils.load_saved_model(saved_path, model)
    model.eval()
    for i, batch_data in enumerate(data_loader):
        batch_data = train_utils.to_device(batch_data, device)
        label_map = batch_data["ego"]["label_dict"]["label_map"]
        output_dict = {
            "cls": label_map[:, 0, :, :],
            "reg": label_map[:, 1:, :, :]
        }
        gt_box_tensor, _ = opencood_dataset.post_processor.post_process_debug(
            batch_data["ego"], output_dict)
        vis_utils.visualize_single_sample_output_bev(gt_box_tensor,
                                                     batch_data['ego'][
                                                         'origin_lidar'].squeeze(
                                                         0),
                                                     opencood_dataset)


if __name__ == '__main__':
    test_bev_post_processing()
