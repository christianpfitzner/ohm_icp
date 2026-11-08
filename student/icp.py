"""ICP after Besl and McKay: the transform that maps the data set D onto the model set M.

    M   model, N_m x 2 numpy array, one point (x, y) per row      here: the older scan
    D   data,  N_d x 2 numpy array                                 here: the newer scan
    T   3 x 3 homogeneous transform (eq. 4.18): a point d of D lands at T (d, 1), close to its partner in M

The four steps of the script, section 4.1.3, and where they are:

    1  closest_points    for every d_j the closest m_i (brute force)                 STUDENT 1.2
    2  best_transform    R and t from the pairs: centroids, eq. 4.14 and 4.17         STUDENT 1.3, 1.4
    3  icp               apply the step to D, put it on top of T                      STUDENT 1.5
    4  icp               stop when the error no longer gets smaller                   given
       pairs_kept        the weights of eq. 4.3: a pair too far apart does not count  STUDENT 1.7
       closest_points_kdtree   step 1 with a kd-tree                                  extra task Z.1

Replace each `raise NotImplementedError("STUDENT ...")` by your code: the comment above it says in words
what goes there, the task in the handout gives the equation. numpy: `A @ B` is the matrix product,
`M.mean(axis=0)` the mean of every column, `M[:, 0]` the first column, `np.sum(a * b)` the sum of the
element-wise products.
"""
import math

import numpy as np


def closest_points(D, M):
    """Step 1, brute force: for every row d_j of D the index of the closest row of M, and its distance.

    Returns (index, distance): two arrays of N_d numbers. M[index[j]] is the partner of D[j].
    """
    diff = D[:, None, :] - M[None, :, :]               # every d_j minus every m_i: N_d x N_m x 2
    # the squared distance of every d_j to every m_i (sum over the last axis of diff); then per row the
    # column of the smallest one (np.argmin), and that smallest distance itself, not squared
    # TODO STUDENT 1.2
    raise NotImplementedError("STUDENT 1.2")
    return index, distance


def best_transform(M, D):
    """Step 2: the T that maps the pairs (D[i], M[i]) onto each other best, eq. 4.4 (row i is pair i)."""
    # the two centroids, and both sets shifted by their own centroid (eq. 4.5 to 4.8)
    # TODO STUDENT 1.3
    raise NotImplementedError("STUDENT 1.3")
    # the angle from the two sums of eq. 4.14 with math.atan2; then the translation of eq. 4.17
    # TODO STUDENT 1.4
    raise NotImplementedError("STUDENT 1.4")
    return se2(t[0], t[1], phi)


def icp(M, D, T_init=None, max_iter=50, eps=1e-7, d_max=0.0, kdtree=False):
    """The ICP loop. Returns (T, info): T maps D onto M, info says how it went.

    T_init   where D starts (task 2: the guess of the odometry); None = the identity, D as it is
    eps      step 4: stop when the error (eq. 4.4, m^2) changed by less than this
    d_max    pairs farther apart than this do not count (STUDENT 1.7); 0 = every pair counts
    kdtree   step 1 with a kd-tree instead of brute force (extra task Z.1)
    """
    T = np.eye(3) if T_init is None else np.array(T_init, dtype=float)
    D_now = transform(T, D)                            # D, moved by the transform so far
    if kdtree:
        from scipy.spatial import cKDTree
        tree = cKDTree(M)                              # built once, asked in every iteration
        search = lambda P: closest_points_kdtree(P, tree)              # noqa: E731
    else:
        search = lambda P: closest_points(P, M)                        # noqa: E731
    error_prev, error, used, iteration = math.inf, math.inf, 0, 0
    for iteration in range(1, max_iter + 1):
        index, distance = search(D_now)                                    # step 1
        keep = pairs_kept(distance, d_max)
        used = int(np.count_nonzero(keep))
        if used < 2:                                                       # nothing left to match
            break
        T_step = best_transform(M[index[keep]], D_now[keep])               # step 2
        # step 3: move D_now by T_step, and put T_step on top of T (the newest step on the left)
        # TODO STUDENT 1.5
        raise NotImplementedError("STUDENT 1.5")
        error = float(np.mean(np.sum((M[index[keep]] - D_now[keep]) ** 2, axis=1)))   # eq. 4.4
        if abs(error_prev - error) < eps:                                  # step 4
            break
        error_prev = error
    return T, {"iterations": iteration, "error": error, "pairs": used, "points": len(D)}


def pairs_kept(distance, d_max):
    """The weights of eq. 4.3: True for a pair that counts, False for one farther apart than d_max.

    d_max = 0 switches it off: every pair counts, as in eq. 4.3 with every weight 1.
    """
    if d_max <= 0.0:
        return np.ones(len(distance), dtype=bool)
    # True for every pair whose distance is at most d_max: one comparison of the whole array
    # TODO STUDENT 1.7
    raise NotImplementedError("STUDENT 1.7")


def closest_points_kdtree(D, tree):
    """Extra task Z.1: step 1 with a kd-tree of M (scipy.spatial.cKDTree), the same (index, distance)."""
    # one call of tree.query for all rows of D; mind the order of the two arrays it gives back
    # TODO STUDENT Z.1
    raise NotImplementedError("STUDENT Z.1")
    return index, distance


# ------------------------------------------------------------------ given: transforms in 2D

def rotation(phi):
    """R(phi), 2 x 2."""
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s], [s, c]])


def se2(x, y, phi):
    """T(phi, t) of eq. 4.18, 3 x 3."""
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def transform(T, P):
    """Every row p of P (N x 2) moved by T: R p + t."""
    return P @ T[:2, :2].T + T[:2, 2]


def pose_of(T):
    """(x, y, phi) of a 3 x 3 transform."""
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])
