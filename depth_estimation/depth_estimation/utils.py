import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import least_squares

def downsample_voxel(points: np.ndarray, voxel_size: float):
    """Voxel-grid downsample; keeps points at centroid of voxels via Open3D."""
    if points.shape[0] == 0:
        return points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return np.asarray(pcd_down.points)

def crop_to_camera_fov(points_cam: np.ndarray, K: np.ndarray, image_shape):
    """
    Keep only points that project inside the camera image.
    points_cam: (N,3) in camera frame (X right, Y up, Z forward).
    K: 3x3 intrinsics.
    image_shape: (h, w)
    """
    zs = points_cam[:, 2]
    # only positive depth
    valid = zs > 0
    if not np.any(valid):
        return np.empty((0,3))
    pts = points_cam[valid]
    proj = (K @ (pts.T)).T  # (N,3)
    u = proj[:, 0] / proj[:, 2]
    v = proj[:, 1] / proj[:, 2]
    h, w = image_shape
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return pts[inside]

def compute_nn_metrics(pred_pts: np.ndarray, gt_pts: np.ndarray):
    """
    Compute nearest-neighbor distances and summary stats.
    Returns dict with per-gt->pred distances and per-pred->gt distances, and stats.
    """
    out = {}
    if gt_pts.shape[0] == 0 or pred_pts.shape[0] == 0:
        out['error'] = 'empty_pred_or_gt'
        return out

    # GT -> Pred (for completeness / recall)
    nn_pred = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(pred_pts)
    d_gt2pred, _ = nn_pred.kneighbors(gt_pts, return_distance=True)
    d_gt2pred = d_gt2pred.ravel()

    # Pred -> GT (for precision)
    nn_gt = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(gt_pts)
    d_pred2gt, _ = nn_gt.kneighbors(pred_pts, return_distance=True)
    d_pred2gt = d_pred2gt.ravel()

    def stats(distances):
        return {
            'mean': float(np.mean(distances)),
            'median': float(np.median(distances)),
            'rmse': float(np.sqrt(np.mean(distances**2))),
            'p90': float(np.percentile(distances, 90)),
            'max': float(np.max(distances)),
            'count': int(distances.size)
        }

    out['gt2pred_dists'] = d_gt2pred
    out['pred2gt_dists'] = d_pred2gt
    out['gt2pred_stats'] = stats(d_gt2pred)
    out['pred2gt_stats'] = stats(d_pred2gt)
    return out

def precision_recall_fscore(d_gt2pred, d_pred2gt, thresholds):
    """
    Compute precision, recall, F-score for thresholds (meters).
    d_gt2pred: array of distances from each gt point to nearest pred point
    d_pred2gt: array of distances from each pred point to nearest gt point
    """
    results = {}
    n_gt = d_gt2pred.size
    n_pred = d_pred2gt.size
    for t in thresholds:
        recall = float(np.count_nonzero(d_gt2pred <= t) / max(1, n_gt))
        precision = float(np.count_nonzero(d_pred2gt <= t) / max(1, n_pred))
        if precision + recall == 0:
            fscore = 0.0
        else:
            fscore = 2 * precision * recall / (precision + recall)
        results[t] = {'precision': precision, 'recall': recall, 'fscore': fscore}
    return results

def chamfer_distance(d_gt2pred, d_pred2gt):
    """Average symmetric nearest neighbor distance (Chamfer)."""
    chamfer = 0.5 * (np.mean(d_gt2pred) + np.mean(d_pred2gt))
    return float(chamfer)

def evaluate_once(pred_pts_cam, gt_pts_cam, thresholds):
    """
    pred_pts_cam: Nx3 stereo points already in camera frame (meters)
    gt_pts_cam: Mx3 Velodyne points in camera frame (meters)
    """
    #Compute NN metrics
    nn_res = compute_nn_metrics(pred_pts_cam, gt_pts_cam)
    if 'error' in nn_res:
        return nn_res

    #Compute PR/F1 and Chamfer
    pr = precision_recall_fscore(nn_res['gt2pred_dists'], nn_res['pred2gt_dists'], thresholds)
    chamfer = chamfer_distance(nn_res['gt2pred_dists'], nn_res['pred2gt_dists'])

    #Bundle results
    results = {
        'nn_stats_gt2pred': nn_res['gt2pred_stats'],
        'nn_stats_pred2gt': nn_res['pred2gt_stats'],
        'precision_recall': pr,
        'chamfer': chamfer,
    }
    return results


# ----------------------------
# Utility: compute normals
# ----------------------------
def compute_normals(points: np.ndarray, k: int = 32):
    """
    Compute normals for points using Open3D. Returns Nx3 normals (unit).
    If points is empty -> returns empty (0,3) array.
    """
    if points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k))
    pcd.normalize_normals()
    normals = np.asarray(pcd.normals, dtype=np.float64)
    return normals


# ----------------------------
# GraphSIM / Normal-based similarity
# ----------------------------
def graphsim(pred_pts: np.ndarray, gt_pts: np.ndarray, k: int = 32):
    """
    GraphSIM-like score based on local normal alignment.
    Returns dict: { 'graphsim_score': float, 'cos_sim': np.ndarray }
    Score in [0,1] (1 is perfect).
    """
    out = {}
    if pred_pts.shape[0] == 0 or gt_pts.shape[0] == 0:
        out["error"] = "empty_pred_or_gt"
        return out

    normals_pred = compute_normals(pred_pts, k=k)
    normals_gt = compute_normals(gt_pts, k=k)

    # Find nearest gt for each pred (match pred -> gt)
    nn = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(gt_pts)
    dists, idxs = nn.kneighbors(pred_pts, return_distance=True)
    idxs = idxs.ravel()

    # Cosine similarity between matched normals
    matched_normals_gt = normals_gt[idxs]
    cos_sim = np.sum(normals_pred * matched_normals_gt, axis=1)
    cos_sim = np.clip(cos_sim, -1.0, 1.0)
    # Map to [0,1]
    sim01 = (cos_sim + 1.0) / 2.0
    score = float(np.mean(sim01))

    out["graphsim_score"] = score
    out["cos_sim"] = sim01  # per-predicted-point similarity
    return out


# ----------------------------
# PointSSIM (neighborhood-level SSIM analog)
# ----------------------------
def _local_stats(points: np.ndarray, k: int = 32):
    """
    For each point compute: local mean (3,), eigenvalue-vector (3 eigenvals),
    and a structure index derived from eigenvalues.
    Returns (means, eigvals, struct_index)
    - means: Nx3
    - eigvals: Nx3 (sorted ascending)
    - struct_index: Nx (float)
    """
    n_pts = points.shape[0]
    if n_pts == 0:
        return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0,))

    nn = NearestNeighbors(n_neighbors=min(k, n_pts), algorithm="kd_tree").fit(points)
    dists, idxs = nn.kneighbors(points)
    means = np.zeros((n_pts, 3), dtype=np.float64)
    eigs = np.zeros((n_pts, 3), dtype=np.float64)
    struct = np.zeros((n_pts,), dtype=np.float64)

    for i in range(n_pts):
        neigh = points[idxs[i]]
        mu = np.mean(neigh, axis=0)
        cov = np.cov((neigh - mu).T) if neigh.shape[0] > 1 else np.zeros((3, 3))
        # Ensure symmetric
        cov = (cov + cov.T) / 2.0
        ev = np.linalg.eigvalsh(cov)
        ev = np.sort(ev)  # ascending
        means[i] = mu
        eigs[i] = ev
        # structure index: largest eigenvalue / sum(eig)
        struct[i] = ev[-1] / (np.sum(ev) + 1e-12)

    return means, eigs, struct


def pointssim(pred_pts: np.ndarray, gt_pts: np.ndarray, k: int = 32):
    """
    Compute PointSSIM at neighborhood size k.
    Returns float in [0,1].
    """
    if pred_pts.shape[0] == 0 or gt_pts.shape[0] == 0:
        return 0.0

    # Match GT -> Pred (for per-GT neighborhood comparison)
    matcher = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(pred_pts)
    _, idx_pred = matcher.kneighbors(gt_pts)
    idx_pred = idx_pred.ravel()

    gt_mean, gt_eig, gt_struct = _local_stats(gt_pts, k=k)
    pred_mean, pred_eig, pred_struct = _local_stats(pred_pts, k=k)

    # Extract matched pred stats aligned to GT points
    pred_mean_matched = pred_mean[idx_pred]
    pred_eig_matched = pred_eig[idx_pred]
    pred_struct_matched = pred_struct[idx_pred]

    # SSIM-like components (use robust constants)
    C1 = 1e-6
    C2 = 1e-6
    C3 = C2 / 2.0

    # Luminance: compare local means (use dot-product normalized)
    lm_num = 2.0 * np.sum(gt_mean * pred_mean_matched, axis=1) + C1
    lm_den = np.sum(gt_mean**2, axis=1) + np.sum(pred_mean_matched**2, axis=1) + C1
    L = lm_num / lm_den

    # Contrast: compare eigenvalue magnitudes
    ct_num = 2.0 * np.sum(gt_eig * pred_eig_matched, axis=1) + C2
    ct_den = np.sum(gt_eig**2, axis=1) + np.sum(pred_eig_matched**2, axis=1) + C2
    C = ct_num / ct_den

    # Structure: compare structure indices
    S = (2.0 * gt_struct * pred_struct_matched + C3) / (gt_struct**2 + pred_struct_matched**2 + C3)

    pssim_map = L * C * S
    pssim = float(np.clip(np.mean(pssim_map), 0.0, 1.0))
    return pssim


# ----------------------------
# PCA-based geometric features per point (linearity, planarity, sphericity, omnivariance, anisotropy, eigenentropy)
# ----------------------------
def pca_geometric_features(points: np.ndarray, k: int = 32):
    """
    Returns a dict of per-point geometric features computed from local PCA eigenvalues.
    Features returned as arrays of length N:
      - linearity = (lambda3 - lambda2) / lambda3
      - planarity = (lambda2 - lambda1) / lambda3
      - sphericity = lambda1 / lambda3
      - omnivariance = (lambda1 * lambda2 * lambda3) ** (1/3)
      - anisotropy = (lambda3 - lambda1) / lambda3
      - eigenentropy = -sum(p_i * log(p_i)) where p_i = lambda_i / sum(lambda)
      - curvature = lambda1 / sum(lambda)
    """
    n_pts = points.shape[0]
    if n_pts == 0:
        return {k: np.zeros((0,)) for k in [
            "linearity", "planarity", "sphericity", "omnivariance", "anisotropy", "eigenentropy", "curvature"
        ]}

    nn = NearestNeighbors(n_neighbors=min(k, n_pts), algorithm="kd_tree").fit(points)
    dists, idxs = nn.kneighbors(points)

    linearity = np.zeros(n_pts)
    planarity = np.zeros(n_pts)
    sphericity = np.zeros(n_pts)
    omnivariance = np.zeros(n_pts)
    anisotropy = np.zeros(n_pts)
    eigenentropy = np.zeros(n_pts)
    curvature = np.zeros(n_pts)

    for i in range(n_pts):
        neigh = points[idxs[i]]
        mu = np.mean(neigh, axis=0)
        cov = np.cov((neigh - mu).T) if neigh.shape[0] > 1 else np.zeros((3, 3))
        cov = (cov + cov.T) / 2.0
        ev = np.linalg.eigvalsh(cov)
        ev = np.sort(ev) + 1e-12  # avoid zeros
        l1, l2, l3 = ev  # ascending
        s = l1 + l2 + l3
        linearity[i] = (l3 - l2) / l3 if l3 != 0 else 0.0
        planarity[i] = (l2 - l1) / l3 if l3 != 0 else 0.0
        sphericity[i] = l1 / l3 if l3 != 0 else 0.0
        omnivariance[i] = (l1 * l2 * l3) ** (1.0 / 3.0)
        anisotropy[i] = (l3 - l1) / l3 if l3 != 0 else 0.0
        p = ev / (s + 1e-12)
        eigenentropy[i] = -np.sum(p * np.log(p + 1e-12))
        curvature[i] = l1 / (s + 1e-12)

    return {
        "linearity": linearity,
        "planarity": planarity,
        "sphericity": sphericity,
        "omnivariance": omnivariance,
        "anisotropy": anisotropy,
        "eigenentropy": eigenentropy,
        "curvature": curvature,
    }


# ----------------------------
# Curvature similarity (compare per-gt vs matched pred)
# ----------------------------
def curvature_similarity(pred_pts: np.ndarray, gt_pts: np.ndarray, k: int = 32):
    """
    Compute curvature similarity: match GT->Pred, compute per-point curvature (from PCA), return mean abs diff and correlation.
    """
    out = {}
    if pred_pts.shape[0] == 0 or gt_pts.shape[0] == 0:
        out["error"] = "empty_pred_or_gt"
        return out

    # Match GT -> Pred
    matcher = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(pred_pts)
    _, idx_pred = matcher.kneighbors(gt_pts)
    idx_pred = idx_pred.ravel()

    gt_feats = pca_geometric_features(gt_pts, k=k)
    pred_feats = pca_geometric_features(pred_pts, k=k)

    gt_curv = gt_feats["curvature"]
    pred_curv_matched = pred_feats["curvature"][idx_pred]

    mean_abs_diff = float(np.mean(np.abs(gt_curv - pred_curv_matched)))
    corrcoef = float(np.corrcoef(gt_curv, pred_curv_matched)[0, 1]) if gt_curv.size > 1 else 0.0

    out["mean_abs_diff"] = mean_abs_diff
    out["correlation"] = corrcoef
    return out


# ----------------------------
# Combined wrapper: compute all advanced metrics
# ----------------------------
def evaluate_advanced(pred_pts: np.ndarray, gt_pts: np.ndarray, k: int = 32):
    """
    Compute all advanced evaluation metrics and return a dict:
      {
        'graphsim': { ... },
        'pointssim': float,
        'pca_feats_gt': {...},
        'pca_feats_pred': {...},
        'curvature_similarity': {...}
      }
    """
    res = {}

    # GraphSIM (normal alignment)
    res["graphsim"] = graphsim(pred_pts, gt_pts, k=k)

    # PointSSIM
    res["pointssim"] = pointssim(pred_pts, gt_pts, k=k)

    # PCA geometric features (per-point arrays) - precompute for analysis
    res["pca_feats_gt"] = pca_geometric_features(gt_pts, k=k)
    res["pca_feats_pred"] = pca_geometric_features(pred_pts, k=k)

    # Curvature similarity
    res["curvature_similarity"] = curvature_similarity(pred_pts, gt_pts, k=k)

    return res


# --- Scaling factor functions --
def get_correspondences(pred_points, lidar_points):
    """
    pred_points: stereo points Nx3
    lidar_points: LiDAR points Mx3
    returns Zs, Zl (matched depth pairs)
    """
    if pred_points.shape[0] == 0 or lidar_points.shape[0] == 0:
        return np.array([]), np.array([])

    # Find nearest stereo point for each LiDAR point
    nn = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(pred_points)
    dists, idx = nn.kneighbors(lidar_points, return_distance=True)
    idx = idx.ravel()

    Zs = pred_points[idx, 2]  # stereo Zs
    Zl = lidar_points[:, 2]  # lidar Zl

    # Filter invalid
    mask = (Zs > 0.2) & (Zs < 80) & (Zl > 0.2) & (Zl < 80)
    return Zs[mask], Zl[mask]

def estimate_scale(Zs, Zl):
    """Estimate scale using RANSAC + Huber refinement."""
    if len(Zs) < 20:
        return 1.0

    # --- RANSAC ---
    N = len(Zs)
    best_a = 1.0
    best_inliers = []

    for _ in range(100):
        sample = np.random.choice(N, size=min(30, N), replace=False)
        a_try = np.sum(Zl[sample]) / (np.sum(Zs[sample]) + 1e-12)
        pred = Zs * a_try
        inliers = np.where(np.abs(pred - Zl) < 0.5)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_a = a_try

    if len(best_inliers) < 10:
        return best_a

    # --- Huber refinement ---
    Zs_in = Zs[best_inliers]
    Zl_in = Zl[best_inliers]

    def residual(a):
        return (a * Zs_in) - Zl_in

    result = least_squares(
        residual, x0=np.array([best_a]), loss="huber", f_scale=1.0
    )
    return float(result.x[0])