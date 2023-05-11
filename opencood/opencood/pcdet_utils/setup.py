import os

from setuptools import find_packages, setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension


def make_cuda_ext(name, module, sources):
    cuda_ext = CUDAExtension(
        name='%s.%s' % (module, name),
        sources=[os.path.join(*module.split('.'), src) for src in sources]
    )
    return cuda_ext


setup(
    name='pcd utils',
    cmdclass={'build_ext': BuildExtension},
    ext_modules=[make_cuda_ext(
                name='iou3d_nms_cuda',
                module='opencood.pcdet_utils.iou3d_nms',
                sources=[
                    'src/iou3d_cpu.cpp',
                    'src/iou3d_nms_api.cpp',
                    'src/iou3d_nms.cpp',
                    'src/iou3d_nms_kernel.cu'
                ]),
        make_cuda_ext(
            name='roiaware_pool3d_cuda',
            module='opencood.pcdet_utils.roiaware_pool3d',
            sources=[
                'src/roiaware_pool3d.cpp',
                'src/roiaware_pool3d_kernel.cu',
            ]
        ),
        make_cuda_ext(
            name='pointnet2_stack_cuda',
            module='opencood.pcdet_utils.pointnet2.pointnet2_stack',
            sources=[
                'src/pointnet2_api.cpp',
                'src/ball_query.cpp',
                'src/ball_query_gpu.cu',
                'src/group_points.cpp',
                'src/group_points_gpu.cu',
                'src/sampling.cpp',
                'src/sampling_gpu.cu',
                'src/interpolate.cpp',
                'src/interpolate_gpu.cu',
            ],
        ),
        make_cuda_ext(
            name='pointnet2_batch_cuda',
            module='opencood.pcdet_utils.pointnet2.pointnet2_batch',
            sources=[
                'src/pointnet2_api.cpp',
                'src/ball_query.cpp',
                'src/ball_query_gpu.cu',
                'src/group_points.cpp',
                'src/group_points_gpu.cu',
                'src/interpolate.cpp',
                'src/interpolate_gpu.cu',
                'src/sampling.cpp',
                'src/sampling_gpu.cu',
            ],
        )]

)