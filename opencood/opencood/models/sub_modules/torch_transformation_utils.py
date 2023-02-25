"""
torch_transformation_utils.py
"""
import os

import torch
import torch.nn.functional as F
import numpy as np
import matplotlib.pyplot as plt

def get_roi_and_cav_mask(shape, cav_mask, spatial_correction_matrix,
                         discrete_ratio, downsample_rate):
    """
    Get mask for the combination of cav_mask and rorated ROI mask.
    Parameters
    ----------
    shape : tuple
        Shape of (B, L, H, W, C).
    cav_mask : torch.Tensor
        Shape of (B, L).
    spatial_correction_matrix : torch.Tensor
        Shape of (B, L, 4, 4)
    discrete_ratio : float
        Discrete ratio.
    downsample_rate : float
        Downsample rate.

    Returns
    -------
    com_mask : torch.Tensor
        Combined mask with shape (B, H, W, L, 1).

    """
    B, L, H, W, C = shape
    C = 1
    # (B,L,4,4)
    dist_correction_matrix = get_discretized_transformation_matrix(
        spatial_correction_matrix, discrete_ratio,
        downsample_rate)
    # (B*L,2,3)
    T = get_transformation_matrix(
        dist_correction_matrix.reshape(-1, 2, 3), (H, W))
    # (B,L,1,H,W)
    roi_mask = get_rotated_roi((B, L, C, H, W), T)
    # (B,L,1,H,W)
    com_mask = combine_roi_and_cav_mask(roi_mask, cav_mask)
    # (B,H,W,1,L)
    com_mask = com_mask.permute(0,3,4,2,1)
    return com_mask


def combine_roi_and_cav_mask(roi_mask, cav_mask):
    """
    Combine ROI mask and CAV mask

    Parameters
    ----------
    roi_mask : torch.Tensor
        Mask for ROI region after considering the spatial transformation/correction.
    cav_mask : torch.Tensor
        Mask for CAV to remove padded 0.

    Returns
    -------
    com_mask : torch.Tensor
        Combined mask.
    """
    # (B, L, 1, 1, 1)
    cav_mask = cav_mask.unsqueeze(2).unsqueeze(3).unsqueeze(4)
    # (B, L, C, H, W)
    cav_mask = cav_mask.expand(roi_mask.shape)
    # (B, L, C, H, W)
    com_mask = roi_mask * cav_mask
    return com_mask


def get_rotated_roi(shape, correction_matrix):
    """
    Get rorated ROI mask.

    Parameters
    ----------
    shape : tuple
        Shape of (B,L,C,H,W).
    correction_matrix : torch.Tensor
        Correction matrix with shape (N,2,3).

    Returns
    -------
    roi_mask : torch.Tensor
        Roated ROI mask with shape (N,2,3).

    """
    B, L, C, H, W = shape
    # To reduce the computation, we only need to calculate the
    # mask for the first channel.
    # (B,L,1,H,W)
    x = torch.ones((B, L, 1, H, W)).to(correction_matrix.dtype).to(
        correction_matrix.device)
    # (B*L,1,H,W)
    roi_mask = warp_affine(x.reshape(-1, 1, H, W), correction_matrix,
                           dsize=(H, W), mode="nearest")
    # (B,L,C,H,W)
    roi_mask = torch.repeat_interleave(roi_mask, C, dim=1).reshape(B, L, C, H,
                                                                   W)
    return roi_mask


def get_discretized_transformation_matrix(matrix, discrete_ratio,
                                          downsample_rate):
    """
    Get disretized transformation matrix.
    Parameters
    ----------
    matrix : torch.Tensor
        Shape -- (B, L, 4, 4) where B is the batch size, L is the max cav
        number.
    discrete_ratio : float
        Discrete ratio.
    downsample_rate : float/int
        downsample_rate

    Returns
    -------
    matrix : torch.Tensor
        Output transformation matrix in 2D with shape (B, L, 2, 3),
        including 2D transformation and 2D rotation.

    """
    matrix = matrix[:, :, [0, 1], :][:, :, :, [0, 1, 3]]
    # normalize the x,y transformation
    matrix[:, :, :, -1] = matrix[:, :, :, -1] \
                          / (discrete_ratio * downsample_rate)

    return matrix.type(dtype=torch.float)


def _torch_inverse_cast(input):
    r"""
    Helper function to make torch.inverse work with other than fp32/64.
    The function torch.inverse is only implemented for fp32/64 which makes
    impossible to be used by fp16 or others. What this function does,
    is cast input data type to fp32, apply torch.inverse,
    and cast back to the input dtype.
    Args:
        input : torch.Tensor
            Tensor to be inversed.

    Returns:
        out : torch.Tensor
            Inversed Tensor.

    """
    dtype = input.dtype
    if dtype not in (torch.float32, torch.float64):
        dtype = torch.float32
    out = torch.inverse(input.to(dtype)).to(input.dtype)
    return out


def normal_transform_pixel(
        height, width, device, dtype, eps=1e-14):
    r"""
    Compute the normalization matrix from image size in pixels to [-1, 1].
    Args:
        height : int
            Image height.
        width : int
            Image width.
        device : torch.device
            Output tensor devices.
        dtype : torch.dtype
            Output tensor data type.
        eps : float
            Epsilon to prevent divide-by-zero errors.

    Returns:
        tr_mat : torch.Tensor
            Normalized transform with shape :math:`(1, 3, 3)`.
    """
    tr_mat = torch.tensor(
        [[1.0, 0.0, -1.0], [0.0, 1.0, -1.0], [0.0, 0.0, 1.0]], device=device,
        dtype=dtype)  # 3x3

    # prevent divide by zero bugs
    width_denom = eps if width == 1 else width - 1.0
    height_denom = eps if height == 1 else height - 1.0

    tr_mat[0, 0] = tr_mat[0, 0] * 2.0 / width_denom
    tr_mat[1, 1] = tr_mat[1, 1] * 2.0 / height_denom

    return tr_mat.unsqueeze(0)  # 1x3x3


def eye_like(n, B, device, dtype):
    r"""
    Return a 2-D tensor with ones on the diagonal and
    zeros elsewhere with the same batch size as the input.
    Args:
        n : int
            The number of rows :math:`(n)`.
        B : int
            Btach size.
        device : torch.device
            Devices of the output tensor.
        dtype : torch.dtype
            Data type of the output tensor.

    Returns:
       The identity matrix with the shape :math:`(B, n, n)`.
    """

    identity = torch.eye(n, device=device, dtype=dtype)
    return identity[None].repeat(B, 1, 1)


def normalize_homography(dst_pix_trans_src_pix, dsize_src, dsize_dst=None):
    r"""
    Normalize a given homography in pixels to [-1, 1].
    Args:
        dst_pix_trans_src_pix : torch.Tensor
            Homography/ies from source to destination to be normalized with
            shape :math:`(B, 3, 3)`.
        dsize_src : Tuple[int, int]
            Size of the source image (height, width).
        dsize_dst : Tuple[int, int]
            Size of the destination image (height, width).

    Returns:
        dst_norm_trans_src_norm : torch.Tensor
            The normalized homography of shape :math:`(B, 3, 3)`.
    """
    if dsize_dst is None:
        dsize_dst = dsize_src
    # source and destination sizes
    src_h, src_w = dsize_src
    dst_h, dst_w = dsize_dst
    device = dst_pix_trans_src_pix.device
    dtype = dst_pix_trans_src_pix.dtype
    # compute the transformation pixel/norm for src/dst
    src_norm_trans_src_pix = normal_transform_pixel(src_h, src_w, device,
                                                    dtype).to(
        dst_pix_trans_src_pix)

    src_pix_trans_src_norm = _torch_inverse_cast(src_norm_trans_src_pix)
    dst_norm_trans_dst_pix = normal_transform_pixel(dst_h, dst_w, device,
                                                    dtype).to(
        dst_pix_trans_src_pix)
    # compute chain transformations
    dst_norm_trans_src_norm: torch.Tensor = dst_norm_trans_dst_pix @ (
            dst_pix_trans_src_pix @ src_pix_trans_src_norm)
    return dst_norm_trans_src_norm


def get_rotation_matrix2d(M, dsize):
    r"""
    Return rotation matrix for torch.affine_grid based on transformation matrix.
    Args:
        M : torch.Tensor
            Transformation matrix with shape :math:`(B, 2, 3)`.
        dsize : Tuple[int, int]
            Size of the source image (height, width).

    Returns:
        R : torch.Tensor
            Rotation matrix with shape :math:`(B, 2, 3)`.
    """
    H, W = dsize
    B = M.shape[0]
    center = torch.Tensor([W / 2, H / 2]).to(M.dtype).to(M.device).unsqueeze(0)
    shift_m = eye_like(3, B, M.device, M.dtype)
    shift_m[:, :2, 2] = center

    shift_m_inv = eye_like(3, B, M.device, M.dtype)
    shift_m_inv[:, :2, 2] = -center

    rotat_m = eye_like(3, B, M.device, M.dtype)
    rotat_m[:, :2, :2] = M[:, :2, :2]
    affine_m = shift_m @ rotat_m @ shift_m_inv
    return affine_m[:, :2, :]  # Bx2x3


def get_transformation_matrix(M, dsize):
    r"""
    Return transformation matrix for torch.affine_grid.
    Args:
        M : torch.Tensor
            Transformation matrix with shape :math:`(N, 2, 3)`.
        dsize : Tuple[int, int]
            Size of the source image (height, width).

    Returns:
        T : torch.Tensor
            Transformation matrix with shape :math:`(N, 2, 3)`.
    """
    T = get_rotation_matrix2d(M, dsize)
    T[..., 2] += M[..., 2]
    return T


def convert_affinematrix_to_homography(A):
    r"""
    Convert to homography coordinates
    Args:
        A : torch.Tensor
            The affine matrix with shape :math:`(B,2,3)`.

    Returns:
        H : torch.Tensor
            The homography matrix with shape of :math:`(B,3,3)`.
    """
    H: torch.Tensor = torch.nn.functional.pad(A, [0, 0, 0, 1], "constant",
                                              value=0.0)
    H[..., -1, -1] += 1.0
    return H


def warp_affine(
        src, M, dsize,
        mode='bilinear',
        padding_mode='zeros',
        align_corners=True):
    r"""
    Transform the src based on transformation matrix M.
    Args:
        src : torch.Tensor
            Input feature map with shape :math:`(B,C,H,W)`.
        M : torch.Tensor
            Transformation matrix with shape :math:`(B,2,3)`.
        dsize : tuple
            Tuple of output image H_out and W_out.
        mode : str
            Interpolation methods for F.grid_sample.
        padding_mode : str
            Padding methods for F.grid_sample.
        align_corners : boolean
            Parameter of F.affine_grid.

    Returns:
        Transformed features with shape :math:`(B,C,H,W)`.
    """

    B, C, H, W = src.size()

    # we generate a 3x3 transformation matrix from 2x3 affine
    M_3x3 = convert_affinematrix_to_homography(M)
    dst_norm_trans_src_norm = normalize_homography(M_3x3, (H, W), dsize)

    # src_norm_trans_dst_norm = torch.inverse(dst_norm_trans_src_norm)
    src_norm_trans_dst_norm = _torch_inverse_cast(dst_norm_trans_src_norm)
    grid = F.affine_grid(src_norm_trans_dst_norm[:, :2, :],
                         [B, C, dsize[0], dsize[1]],
                         align_corners=align_corners)

    return F.grid_sample(src.half() if grid.dtype==torch.half else src, 
                         grid, align_corners=align_corners, mode=mode,
                         padding_mode=padding_mode)


class Test:
    """
    Test the transformation in this file.
    The methods in this class are not supposed to be used outside of this file.
    """

    def __init__(self):
        pass

    @staticmethod
    def load_img():
        torch.manual_seed(0)
        x = torch.randn(1, 5, 16, 400, 200) * 100
        # x = torch.ones(1, 5, 16, 400, 200)
        return x

    @staticmethod
    def load_raw_transformation_matrix(N):
        a = 90 / 180 * np.pi
        matrix = torch.Tensor([[np.cos(a), -np.sin(a), 10],
                               [np.sin(a), np.cos(a), 10]])
        matrix = torch.repeat_interleave(matrix.unsqueeze(0).unsqueeze(0), N,
                                         dim=1)
        return matrix

    @staticmethod
    def load_raw_transformation_matrix2(N, alpha):
        a = alpha / 180 * np.pi
        matrix = torch.Tensor([[np.cos(a), -np.sin(a), 0, 0],
                               [np.sin(a), np.cos(a), 0, 0]])
        matrix = torch.repeat_interleave(matrix.unsqueeze(0).unsqueeze(0), N,
                                         dim=1)
        return matrix

    @staticmethod
    def test():
        img = Test.load_img()
        B, L, C, H, W = img.shape
        raw_T = Test.load_raw_transformation_matrix(5)
        T = get_transformation_matrix(raw_T.reshape(-1, 2, 3), (H, W))
        img_rot = warp_affine(img.reshape(-1, C, H, W), T, (H, W))
        print(img_rot[0, 0, :, :])
        plt.matshow(img_rot[0, 0, :, :])
        plt.show()

    @staticmethod
    def test_combine_roi_and_cav_mask():
        B = 2
        L = 5
        C = 16
        H = 300
        W = 400
        # 2, 5
        cav_mask = torch.Tensor([[1, 1, 1, 0, 0], [1, 0, 0, 0, 0]])
        x = torch.zeros(B, L, C, H, W)
        correction_matrix = Test.load_raw_transformation_matrix2(5, 10)
        correction_matrix = torch.cat([correction_matrix, correction_matrix],
                                      dim=0)
        mask = get_roi_and_cav_mask((B, L, H, W, C), cav_mask, 
                                    correction_matrix, 0.4, 4)
        plt.matshow(mask[0, :, :, 0, 0])
        plt.show()


if __name__ == "__main__":
    os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'
    Test.test_combine_roi_and_cav_mask()
