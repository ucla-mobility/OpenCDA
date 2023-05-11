# --------------------------------------------------------
# Fast R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Sergey Karayev
# --------------------------------------------------------

import numpy as np
cimport numpy as np
from cython.parallel import prange, parallel


DTYPE = np.float32
ctypedef float DTYPE_t


def bbox_overlaps(
        np.ndarray[DTYPE_t, ndim=2] boxes,
        np.ndarray[DTYPE_t, ndim=2] query_boxes):
    """
    Parameters
    ----------
    boxes: (N, 4) ndarray of float
    query_boxes: (K, 4) ndarray of float
    Returns
    -------
    overlaps: (N, K) ndarray of overlap between boxes and query_boxes
    """
    cdef unsigned int N = boxes.shape[0]
    cdef unsigned int K = query_boxes.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=2] overlaps = np.zeros((N, K), dtype=DTYPE)
    cdef DTYPE_t iw, ih, box_area
    cdef DTYPE_t ua
    cdef unsigned int k, n
    for k in range(K):
        box_area = (
            (query_boxes[k, 2] - query_boxes[k, 0] + 1) *
            (query_boxes[k, 3] - query_boxes[k, 1] + 1)
        )
        for n in range(N):
            iw = (
                min(boxes[n, 2], query_boxes[k, 2]) -
                max(boxes[n, 0], query_boxes[k, 0]) + 1
            )
            if iw > 0:
                ih = (
                    min(boxes[n, 3], query_boxes[k, 3]) -
                    max(boxes[n, 1], query_boxes[k, 1]) + 1
                )
                if ih > 0:
                    ua = float(
                        (boxes[n, 2] - boxes[n, 0] + 1) *
                        (boxes[n, 3] - boxes[n, 1] + 1) +
                        box_area - iw * ih
                    )
                    overlaps[n, k] = iw * ih / ua
    return overlaps

def bbox_intersections(
        np.ndarray[DTYPE_t, ndim=2] boxes,
        np.ndarray[DTYPE_t, ndim=2] query_boxes):
    """
    For each query box compute the intersection ratio covered by boxes
    ----------
    Parameters
    ----------
    boxes: (N, 4) ndarray of float
    query_boxes: (K, 4) ndarray of float
    Returns
    -------
    overlaps: (N, K) ndarray of intersec between boxes and query_boxes
    """
    cdef unsigned int N = boxes.shape[0]
    cdef unsigned int K = query_boxes.shape[0]
    cdef np.ndarray[DTYPE_t, ndim=2] intersec = np.zeros((N, K), dtype=DTYPE)
    cdef DTYPE_t iw, ih, box_area
    cdef DTYPE_t ua
    cdef unsigned int k, n
    for k in range(K):
        box_area = (
            (query_boxes[k, 2] - query_boxes[k, 0] + 1) *
            (query_boxes[k, 3] - query_boxes[k, 1] + 1)
        )
        for n in range(N):
            iw = (
                min(boxes[n, 2], query_boxes[k, 2]) -
                max(boxes[n, 0], query_boxes[k, 0]) + 1
            )
            if iw > 0:
                ih = (
                    min(boxes[n, 3], query_boxes[k, 3]) -
                    max(boxes[n, 1], query_boxes[k, 1]) + 1
                )
                if ih > 0:
                    intersec[n, k] = iw * ih / box_area
    return intersec

# Compute bounding box voting
def box_vote(
        np.ndarray[float, ndim=2] dets_NMS,
        np.ndarray[float, ndim=2] dets_all):
    cdef np.ndarray[float, ndim=2] dets_voted = np.zeros((dets_NMS.shape[0], dets_NMS.shape[1]), dtype=np.float32)
    cdef unsigned int N = dets_NMS.shape[0]
    cdef unsigned int M = dets_all.shape[0]

    cdef np.ndarray[float, ndim=1] det
    cdef np.ndarray[float, ndim=1] acc_box
    cdef float acc_score

    cdef np.ndarray[float, ndim=1] det2
    cdef float bi0, bi1, bit2, bi3
    cdef float iw, ih, ua

    cdef float thresh=0.5

    for i in range(N):
        det = dets_NMS[i, :]
        acc_box = np.zeros((4), dtype=np.float32)
        acc_score = 0.0

        for m in range(M):
            det2 = dets_all[m, :]

            bi0 = max(det[0], det2[0])
            bi1 = max(det[1], det2[1])
            bi2 = min(det[2], det2[2])
            bi3 = min(det[3], det2[3])

            iw = bi2 - bi0 + 1
            ih = bi3 - bi1 + 1

            if not (iw > 0 and ih > 0):
                continue

            ua = (det[2] - det[0] + 1) * (det[3] - det[1] + 1) + (det2[2] - det2[0] + 1) * (det2[3] - det2[1] + 1) - iw * ih
            ov = iw * ih / ua

            if (ov < thresh):
                continue

            acc_box += det2[4] * det2[0:4]
            acc_score += det2[4]

        dets_voted[i][0:4] = acc_box / acc_score
        dets_voted[i][4] = det[4]       # Keep the original score

    return dets_voted
