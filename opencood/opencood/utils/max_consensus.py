import numpy as np
from sklearn.neighbors import NearestNeighbors


def max_consunsus_hierarchical(pointsl, pointsr, loc_l, loc_r, resolution=None, radius=1, point_labels=None, label_weights=None, **kwargs):
    max_err = kwargs['search_range']# np.array([1, 1, 6])
    min_cons = kwargs['min_cons']
    min_match_acc_points = kwargs['min_match_acc_points']
    pointsl_out, pointsr_out, T, tf_local, cons, matched_pointsl, matched_pointsr = max_consensus2(pointsl, pointsr, -max_err, max_err,
                                                                           resolution, radius, loc_l, loc_r,
                                                                          point_labels=point_labels, label_weights=label_weights)

    if matched_pointsl is not None and len(matched_pointsl) > min_match_acc_points:
        T, tf = estimate_tf_2d(matched_pointsl, matched_pointsr, pointsl, pointsr_out)
        tf_local = tf
        tf_local[:2] = tf_local[:2] = tf_local[:2] - loc_r[0, :2] + loc_l[0, :2]
        pointsr_homo = np.concatenate([pointsr, np.ones((len(pointsr), 1))], axis=1).T
        pointsr_out = (T @ pointsr_homo).T
    else:
        return None, None, None

    if cons < min_cons:
        return None, None, None
    return T, tf_local, pointsr_out


def max_consensus2(pointsl, pointsr, xyr_min, xyr_max, resolotion, radius, loc_l=None, loc_r=None, point_labels=None, label_weights=None):
    tf_matrices, tf_params, tf_params_local = construct_tfs(xyr_min, xyr_max, resolotion, loc_l, loc_r)
    rotl, _, _ = construct_tfs(xyr_min[2:], xyr_max[2:], resolotion[2:])
    pointr_homo = np.concatenate([pointsr, np.ones((len(pointsr), 1))], axis=1).T
    # pointl_homo = np.concatenate([pointsl, np.ones((len(pointsl), 1))], axis=1).T
    pointr_transformed = np.einsum('...ij, ...jk', tf_matrices, np.tile(pointr_homo,(len(tf_matrices), 1, 1))).transpose(0, 2, 1)
    pointr_transformed_s = pointr_transformed.reshape(-1, 3)[:, :2]
    cur_cons = 0
    pointl_out = pointsl
    pointr_out = pointsr
    match_T, match_tf_local, matched_pointsl, matched_pointsr = None, None, None, None
    # r1 = 0
    for R in rotl[:, :2, :2]:
        pointl_transformed = np.einsum('ij, jk', R, pointsl.T).T
        nbrs = NearestNeighbors(n_neighbors=1, radius=radius, algorithm='auto').fit(pointl_transformed)
        distances, indices = nbrs.kneighbors(pointr_transformed_s)
        mask = (distances < radius)
        lbll, lblr = point_labels
        plus = (np.logical_and(lbll[indices] > 2, mask)).reshape(len(tf_matrices), len(pointsr))
        mask = mask.reshape(len(tf_matrices), len(pointsr))
        pointr_consensus = mask.sum(axis=1) + plus.sum(axis=1) * label_weights[-1]
        best_match = np.argmax(pointr_consensus)
        match_consensus = pointr_consensus[best_match]
        if match_consensus > cur_cons:
            pointr_out = pointr_transformed[best_match]
            match_T = tf_matrices[best_match]
            match_tf_local = tf_params_local[best_match]
            accurate_points_mask = plus[best_match]
            selected_indices = indices.reshape(len(tf_matrices), len(pointsr))[best_match][accurate_points_mask]
            matched_pointsl = pointsl[selected_indices]
            matched_pointsr = pointsr[accurate_points_mask]
            # r1 = np.arctan2(R[1, 0], R[0, 0])
            pointl_out = pointl_transformed
            cur_cons = match_consensus
    return pointl_out, pointr_out, match_T, match_tf_local, cur_cons, matched_pointsl, matched_pointsr


def max_consensus1(pointsl, pointsr, xyr_min, xyr_max, resolotion, radius, loc_l=None, loc_r=None, point_labels=None, label_weights=None):
    tf_matrices, tf_params, tf_params_local = construct_tfs(xyr_min, xyr_max, resolotion, loc_l, loc_r)
    pointr_homo = np.concatenate([pointsr, np.ones((len(pointsr), 1))], axis=1).T
    pointr_transformed = np.einsum('...ij, ...jk', tf_matrices, np.tile(pointr_homo,(len(tf_matrices), 1, 1))).transpose(0, 2, 1)
    pointr_transformed_s = pointr_transformed.reshape(-1, 3)[:, :2]

    nbrs = NearestNeighbors(n_neighbors=1, radius=radius, algorithm='auto').fit(pointsl)
    distances, indices = nbrs.kneighbors(pointr_transformed_s)
    mask = (distances < radius)
    lbll, lblr = point_labels
    plus = (np.logical_and(lbll[indices] > 2, mask)).reshape(len(tf_matrices), len(pointsr))
    mask = mask.reshape(len(tf_matrices), len(pointsr))
    pointr_consensus = mask.sum(axis=1) + plus.sum(axis=1) * label_weights[-1]
    best_match = np.argmax(pointr_consensus)
    match_consensus = pointr_consensus[best_match]
    pointr_out = pointr_transformed[best_match]
    match_tf = tf_params[best_match]
    match_T = tf_matrices[best_match]
    match_tf_local = tf_params_local[best_match]
    accurate_points_mask = plus[best_match]
    selected_indices = indices.reshape(len(tf_matrices), len(pointsr))[best_match][accurate_points_mask]
    matched_pointsl = pointsl[selected_indices]
    matched_pointsr = pointsr[accurate_points_mask]
    return pointr_out, match_T, match_tf_local, match_consensus, matched_pointsl, matched_pointsr


def construct_tfs(xyr_min, xyr_max, resolution, loc_l=None, loc_r=None):
    input = [np.arange(xyr_min[i], xyr_max[i], resolution[i]) for i in range(len(xyr_min))]
    grid = np.meshgrid(*input)
    grid = [a.reshape(-1) for a in grid]
    tf_parames_local = np.stack(grid, axis=1)
    tf_parames_local[:, -1] = tf_parames_local[:, -1] / 180 * np.pi
    tf_parames = np.copy(tf_parames_local)
    if loc_r is not None:
        tf_parames[:, :-1] = tf_parames_local[:, :2] + loc_r[:, :2] - loc_l[:, :2]
    sina = np.sin(tf_parames[:, -1])
    cosa = np.cos(tf_parames[:, -1])
    zeros = np.zeros(len(tf_parames), dtype=sina.dtype)
    ones = np.ones(len(tf_parames), dtype=sina.dtype)
    x = tf_parames[:, 0] if len(xyr_min)>1 else zeros
    y = tf_parames[:, 1] if len(xyr_min)>1 else zeros
    tfs = np.array([[cosa, -sina, x],
                    [sina, cosa, y],
                    [zeros, zeros, ones]]).transpose(2, 0, 1)
    return tfs, tf_parames, tf_parames_local


def estimate_tf_2d(pointsr, pointsl, pointsl_all, pointsr_all):
    # 1 reduce by the center of mass
    l_mean = pointsl.mean(axis=0)
    r_mean = pointsr.mean(axis=0)
    l_reduced = pointsl - l_mean
    r_reduced = pointsr - r_mean
    # 2 compute the rotation
    Sxx = (l_reduced[:, 0] * r_reduced[:, 0]).sum()
    Syy = (l_reduced[:, 1] * r_reduced[:, 1]).sum()
    Sxy = (l_reduced[:, 0] * r_reduced[:, 1]).sum()
    Syx = (l_reduced[:, 1] * r_reduced[:, 0]).sum()
    theta = np.arctan2(Sxy - Syx, Sxx + Syy)  # / np.pi * 180
    sa = np.sin(theta)
    ca = np.cos(theta)
    T = np.array([[ca, -sa, 0],
                  [sa, ca, 0],
                  [0, 0, 1]])
    t = r_mean.reshape(2, 1) - T[:2, :2] @ l_mean.reshape(2, 1)
    # T = T.T
    T[:2, 2:] = t
    return T, np.array([*t.squeeze(), theta])
