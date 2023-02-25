# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencood.data_utils.pre_processor.base_preprocessor import BasePreprocessor
from opencood.data_utils.pre_processor.voxel_preprocessor import VoxelPreprocessor
from opencood.data_utils.pre_processor.bev_preprocessor import BevPreprocessor
from opencood.data_utils.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor

__all__ = {
    'BasePreprocessor': BasePreprocessor,
    'VoxelPreprocessor': VoxelPreprocessor,
    'BevPreprocessor': BevPreprocessor,
    'SpVoxelPreprocessor': SpVoxelPreprocessor
}


def build_preprocessor(preprocess_cfg, train):
    process_method_name = preprocess_cfg['core_method']
    error_message = f"{process_method_name} is not found. " \
                     f"Please add your processor file's name in opencood/" \
                     f"data_utils/processor/init.py"
    assert process_method_name in ['BasePreprocessor', 'VoxelPreprocessor',
                                   'BevPreprocessor', 'SpVoxelPreprocessor'], \
        error_message

    processor = __all__[process_method_name](
        preprocess_params=preprocess_cfg,
        train=train
    )

    return processor
