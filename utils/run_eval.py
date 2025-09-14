import os
import json
import math
import bisect
import argparse
import subprocess
from typing import Optional, List, Dict, Tuple
import numpy as np

def index_based_association(gt: List[Tuple[float, np.ndarray]], est: List[Tuple[float, np.ndarray]]) -> List[Tuple[int,int]]:
    pairs = []
    if not gt or not est:
        return pairs
    n_gt = len(gt)
    n_est = len(est)
    if n_est == 1:
        # map single est to nearest gt by timestamp
        est_t = est[0][0]
        best_g = min(range(n_gt), key=lambda i: abs(gt[i][0] - est_t))
        return [(best_g, 0)]
    # proportional mapping
    for j in range(n_est):
        gi = int(round(j * (n_gt - 1) / (n_est - 1)))
        gi = max(0, min(n_gt - 1, gi))
        pairs.append((gi, j))
    # remove duplicates on gi if any (keep first)
    seen_g = set()
    filtered = []
    for gi, ej in pairs:
        if gi in seen_g:
            continue
        seen_g.add(gi)
        filtered.append((gi, ej))
    return filtered


def associate_tum(gt: List[Tuple[float, np.ndarray]], est: List[Tuple[float, np.ndarray]], max_diff: float = 0.05) -> List[Tuple[int, int]]:
    pairs: List[Tuple[int, int]] = []
    if not gt or not est:
        return pairs

    gt_times = [t for t, _ in gt]
    est_times = [t for t, _ in est]

    # quick statistic on est timestamps
    if len(est_times) > 1:
        est_std = float(np.std(np.array(est_times)))
    else:
        est_std = 0.0

    # If est timestamps are nearly constant, skip timestamp matching and use index-based mapping
    if est_std < 1e-6:
        return index_based_association(gt, est)

    # Normal case: nearest-neighbor using binary search
    est_idx_sorted = list(range(len(est_times)))
    est_times_sorted = [est_times[i] for i in est_idx_sorted]
    # ensure sorted order (est_times might already be monotonic but we don't assume it)
    perm = sorted(range(len(est_times_sorted)), key=lambda i: est_times_sorted[i])
    est_times_sorted = [est_times_sorted[i] for i in perm]
    est_idx_sorted = [est_idx_sorted[i] for i in perm]

    used_est = set()
    for gi, (gts, _) in enumerate(gt):
        pos = bisect.bisect_left(est_times_sorted, gts)
        candidates = []
        if pos < len(est_times_sorted):
            candidates.append((abs(est_times_sorted[pos] - gts), est_idx_sorted[pos]))
        if pos > 0:
            candidates.append((abs(est_times_sorted[pos-1] - gts), est_idx_sorted[pos-1]))
        if not candidates:
            continue
        candidates.sort(key=lambda x: x[0])
        best_diff, best_est_idx = candidates[0]
        if best_diff <= max_diff and best_est_idx not in used_est:
            used_est.add(best_est_idx)
            pairs.append((gi, best_est_idx))

    # if association succeeded with some matches, return them
    if pairs:
        # sort by gt times (gi ascending)
        pairs.sort(key=lambda p: gt[p[0]][0])
        return pairs

    # fallback: no timestamp matches found -> try index-based association
    return index_based_association(gt, est)

def get_latest_file(directory: str, ext: str) -> Optional[str]:
    if not os.path.isdir(directory):
        return None
    files = [f for f in os.listdir(directory) if f.endswith(f".{ext}")]
    if not files:
        return None
    latest_ts = -1.0
    latest_file = None
    for fn in files:
        name = fn[:-len(ext)-1]
        try:
            ts = float(name)
            if ts > latest_ts:
                latest_ts = ts
                latest_file = fn
        except Exception:
            if latest_file is None or fn > latest_file:
                latest_file = fn
    return os.path.join(directory, latest_file) if latest_file else None


def list_timestamp_files(directory: str, ext: str) -> List[str]:
    if not os.path.isdir(directory):
        return []
    files = [f for f in os.listdir(directory) if f.endswith(f".{ext}")]
    def sort_key(fn):
        base = fn[:-len(ext)-1]
        if base == "final":
            return float('inf')
        try:
            return float(base)
        except:
            return float('inf') - 1
    return sorted(files, key=sort_key)


def count_tum_poses(tum_path: str) -> int:
    if not tum_path or not os.path.isfile(tum_path):
        return 0
    with open(tum_path, 'r') as f:
        return sum(1 for _ in f)


def run_subprocess_capture(cmd: List[str], cwd: Optional[str] = None) -> Tuple[int, str, str]:
    try:
        proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, check=False)
        return proc.returncode, proc.stdout, proc.stderr
    except FileNotFoundError as e:
        return 127, "", str(e)


def safe_mkdir(path: str):
    os.makedirs(path, exist_ok=True)


def merge_tums(tum_paths: List[str], out_path: str) -> None:
    seen_ts = set()
    lines = []
    for p in tum_paths:
        if not p or not os.path.isfile(p):
            continue
        with open(p, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                ts = parts[0]
                if ts in seen_ts:
                    continue
                seen_ts.add(ts)
                lines.append(line + "\n")
    with open(out_path, 'w') as f:
        f.writelines(lines)


def find_robot_dirs(run_dir: str) -> List[str]:
    dirs = []
    for name in os.listdir(run_dir):
        if name.startswith("r") and name[1:].isdigit() and os.path.isdir(os.path.join(run_dir, name)):
            dirs.append(os.path.join(run_dir, name))
    return sorted(dirs)


def read_tum(tum_path: str) -> List[Tuple[float, np.ndarray]]:
    out = []
    if not tum_path or not os.path.isfile(tum_path):
        return out
    with open(tum_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0])
                tx = float(parts[1]); ty = float(parts[2]); tz = float(parts[3])
                qx = float(parts[4]); qy = float(parts[5]); qz = float(parts[6]); qw = float(parts[7])
                T = transform_from_quaternion(tx, ty, tz, qx, qy, qz, qw)
                out.append((ts, T))
            except Exception:
                continue
    return out


def transform_from_quaternion(tx, ty, tz, qx, qy, qz, qw) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.dot(q, q)
    if n < 1e-12:
        q = np.array([0, 0, 0, 1.0], dtype=float)
    else:
        q = q / math.sqrt(n)
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
    ], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.array([tx, ty, tz], dtype=float)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=float)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T.dot(t)
    return Tinv

# ---------------------- associations ----------------------

def associate_tum(gt: List[Tuple[float, np.ndarray]], est: List[Tuple[float, np.ndarray]], max_diff: float = 0.05) -> List[Tuple[int, int]]:
    pairs: List[Tuple[int, int]] = []
    if not gt or not est:
        return pairs
    est_times = [t for t, _ in est]
    # Build index for EST times (sorted)
    est_sorted_idx = list(range(len(est_times)))
    # For each gt timestamp find nearest est
    for gi, (gts, _) in enumerate(gt):
        # binary search in est_times
        lo = 0; hi = len(est_times) - 1
        best_j = None
        best_diff = float('inf')
        # linear scan near neighbors might be faster for small lists, but binary search + expand
        # We will do a simple linear search with clamp for reliability
        # (this keeps code simple and robust)
        for ej, ets in enumerate(est_times):
            diff = abs(ets - gts)
            if diff < best_diff:
                best_diff = diff
                best_j = ej
        if best_diff <= max_diff and best_j is not None:
            pairs.append((gi, best_j))
    # Remove duplicates on est index: keep first match (by gt ordering)
    used_est = set()
    filtered = []
    for gi, ej in pairs:
        if ej in used_est:
            continue
        used_est.add(ej)
        filtered.append((gi, ej))
    return filtered

# ---------------------- Umeyama / APE ----------------------

def umeyama_align(A_pts: np.ndarray, B_pts: np.ndarray, with_scale: bool = True) -> Tuple[float, np.ndarray, np.ndarray]:
    assert A_pts.shape == B_pts.shape
    n = A_pts.shape[0]
    if n == 0:
        return 1.0, np.eye(3), np.zeros(3)
    mu_A = np.mean(A_pts, axis=0)
    mu_B = np.mean(B_pts, axis=0)
    X = A_pts - mu_A
    Y = B_pts - mu_B
    cov = (X.T @ Y) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_B = np.sum(Y ** 2) / n
    if with_scale and var_B > 0:
        scale = np.trace(np.diag(D) @ S) / var_B
    else:
        scale = 1.0
    t = mu_A - scale * R @ mu_B
    return scale, R, t


def apply_similarity(T_pts: np.ndarray, s: float, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (s * (R @ T_pts.T).T) + t

def index_based_association(gt: List[Tuple[float, np.ndarray]], est: List[Tuple[float, np.ndarray]]) -> List[Tuple[int,int]]:
    pairs = []
    if not gt or not est:
        return pairs
    n_gt = len(gt)
    n_est = len(est)
    if n_est == 1:
        # map single est to nearest gt by timestamp
        est_t = est[0][0]
        best_g = min(range(n_gt), key=lambda i: abs(gt[i][0] - est_t))
        return [(best_g, 0)]
    # proportional mapping
    for j in range(n_est):
        gi = int(round(j * (n_gt - 1) / (n_est - 1)))
        gi = max(0, min(n_gt - 1, gi))
        pairs.append((gi, j))
    # remove duplicates on gi if any (keep first)
    seen_g = set()
    filtered = []
    for gi, ej in pairs:
        if gi in seen_g:
            continue
        seen_g.add(gi)
        filtered.append((gi, ej))
    return filtered


def associate_tum(gt: List[Tuple[float, np.ndarray]], est: List[Tuple[float, np.ndarray]], max_diff: float = 0.05) -> List[Tuple[int, int]]:
    pairs: List[Tuple[int, int]] = []
    if not gt or not est:
        return pairs

    gt_times = [t for t, _ in gt]
    est_times = [t for t, _ in est]

    # quick statistic on est timestamps
    if len(est_times) > 1:
        est_std = float(np.std(np.array(est_times)))
    else:
        est_std = 0.0

    # If est timestamps are nearly constant, skip timestamp matching and use index-based mapping
    if est_std < 1e-6:
        return index_based_association(gt, est)

    # Normal case: nearest-neighbor using binary search
    est_idx_sorted = list(range(len(est_times)))
    est_times_sorted = [est_times[i] for i in est_idx_sorted]
    # ensure sorted order (est_times might already be monotonic but we don't assume it)
    perm = sorted(range(len(est_times_sorted)), key=lambda i: est_times_sorted[i])
    est_times_sorted = [est_times_sorted[i] for i in perm]
    est_idx_sorted = [est_idx_sorted[i] for i in perm]

    used_est = set()
    for gi, (gts, _) in enumerate(gt):
        pos = bisect.bisect_left(est_times_sorted, gts)
        candidates = []
        if pos < len(est_times_sorted):
            candidates.append((abs(est_times_sorted[pos] - gts), est_idx_sorted[pos]))
        if pos > 0:
            candidates.append((abs(est_times_sorted[pos-1] - gts), est_idx_sorted[pos-1]))
        if not candidates:
            continue
        candidates.sort(key=lambda x: x[0])
        best_diff, best_est_idx = candidates[0]
        if best_diff <= max_diff and best_est_idx not in used_est:
            used_est.add(best_est_idx)
            pairs.append((gi, best_est_idx))

    # if association succeeded with some matches, return them
    if pairs:
        # sort by gt times (gi ascending)
        pairs.sort(key=lambda p: gt[p[0]][0])
        return pairs

    # fallback: no timestamp matches found -> try index-based association
    return index_based_association(gt, est)


# ---------------------- RPE ----------------------

def rotation_angle_from_matrix(R: np.ndarray) -> float:
    # angle in radians from rotation matrix
    # clip trace to [-1,3]
    tr = np.trace(R)
    cosval = (tr - 1.0) / 2.0
    cosval = max(-1.0, min(1.0, cosval))
    angle = math.acos(cosval)
    return angle

def compute_ape_stats(gt_tum: str, est_tum: str, max_diff: float = 0.05) -> Dict:
    gt = read_tum(gt_tum)
    est = read_tum(est_tum)
    # try association with provided tolerance
    pairs = associate_tum(gt, est, max_diff=max_diff)
    # if no pairs at all, try a more permissive tolerance before falling back (helpful for clock skew)
    if not pairs and len(gt) and len(est):
        pairs = associate_tum(gt, est, max_diff=max(0.5, max_diff * 10))
    if not pairs:
        # final fallback: index-based association (proportional mapping / end-align)
        pairs = index_based_association(gt, est)
    if not pairs:
        return {"count": 0}

    A_pts = []
    B_pts = []
    for gi, ej in pairs:
        _, Tg = gt[gi]
        _, Te = est[ej]
        A_pts.append(Tg[:3, 3])
        B_pts.append(Te[:3, 3])
    A = np.array(A_pts)
    B = np.array(B_pts)
    # Use rigid Umeyama (no scale) for APE alignment
    s, R, t = umeyama_align(A, B, with_scale=False)
    B_aligned = apply_similarity(B, s, R, t)
    trans_errs = np.linalg.norm(A - B_aligned, axis=1)
    stats = {
        "count": int(len(trans_errs)),
        "trans_rmse": float(math.sqrt(float(np.mean(trans_errs**2)))) if len(trans_errs) > 0 else None,
        "trans_mean": float(np.mean(trans_errs)) if len(trans_errs) > 0 else None,
        "trans_median": float(np.median(trans_errs)) if len(trans_errs) > 0 else None,
        "trans_min": float(np.min(trans_errs)) if len(trans_errs) > 0 else None,
        "trans_max": float(np.max(trans_errs)) if len(trans_errs) > 0 else None,
    }
    return stats


def compute_rpe(gt_tum: str, est_tum: str, delta: int = 1, max_diff: float = 0.05) -> Dict:
    gt = read_tum(gt_tum)
    est = read_tum(est_tum)
    pairs = associate_tum(gt, est, max_diff=max_diff)
    if not pairs and len(gt) and len(est):
        pairs = associate_tum(gt, est, max_diff=max(0.5, max_diff * 10))
    if not pairs:
        pairs = index_based_association(gt, est)
    if not pairs:
        return {"count": 0}

    # sort pairs by gt timestamp order
    pairs_sorted = sorted(pairs, key=lambda p: gt[p[0]][0])
    matched_gt_T = [gt[gi][1] for gi, _ in pairs_sorted]
    matched_est_T = [est[ej][1] for _, ej in pairs_sorted]

    trans_errs = []
    rot_errs = []
    n = len(matched_gt_T)
    if n <= delta:
        return {"count": 0}

    for i in range(0, n - delta):
        Tg_i = matched_gt_T[i]; Tg_j = matched_gt_T[i + delta]
        Te_i = matched_est_T[i]; Te_j = matched_est_T[i + delta]
        rel_gt = invert_transform(Tg_i) @ Tg_j
        rel_est = invert_transform(Te_i) @ Te_j
        E = invert_transform(rel_gt) @ rel_est
        t_err = np.linalg.norm(E[:3, 3])
        trans_errs.append(float(t_err))
        angle = rotation_angle_from_matrix(E[:3, :3])
        rot_errs.append(math.degrees(angle))
    trans_errs = np.array(trans_errs) if trans_errs else np.array([])
    rot_errs = np.array(rot_errs) if rot_errs else np.array([])
    stats = {
        "count": int(len(trans_errs)),
        "rpe_trans_rmse": float(math.sqrt(float(np.mean(trans_errs**2)))) if trans_errs.size > 0 else None,
        "rpe_trans_mean": float(np.mean(trans_errs)) if trans_errs.size > 0 else None,
        "rpe_trans_median": float(np.median(trans_errs)) if trans_errs.size > 0 else None,
        "rpe_trans_min": float(np.min(trans_errs)) if trans_errs.size > 0 else None,
        "rpe_trans_max": float(np.max(trans_errs)) if trans_errs.size > 0 else None,
        "rpe_rot_rmse_deg": float(math.sqrt(float(np.mean(rot_errs**2)))) if rot_errs.size > 0 else None,
        "rpe_rot_mean_deg": float(np.mean(rot_errs)) if rot_errs.size > 0 else None,
        "rpe_rot_median_deg": float(np.median(rot_errs)) if rot_errs.size > 0 else None,
    }
    return stats


# ---------------------- evo wrappers (plots only) ----------------------

def run_evo_ape(gt_tum: str, est_tum: str, out_png: str, out_zip: str, align: bool = True, show_plot: bool = True) -> Dict:
    """Run evo_ape to save plot and save_results zip. We DO NOT parse evo output programmatically here; instead we return raw stdout/stderr.
    """
    cmd = ["evo_ape", "tum", gt_tum, est_tum]
    if align:
        cmd += ["--align"]
    if show_plot:
        cmd += ["-p"]
    cmd += ["--save_plot", out_png, "--save_results", out_zip]
    rc, out, err = run_subprocess_capture(cmd)
    return {"cmd": cmd, "returncode": rc, "stdout": out, "stderr": err, "plot": out_png, "results": out_zip}


def run_evo_traj(gt_tum: str, est_tum: str, out_png: str, show_plot: bool = True) -> Dict:
    cmd = ["evo_traj", "tum", gt_tum, est_tum, "--ref", gt_tum]
    if show_plot:
        cmd += ["-p"]
    cmd += ["--save_plot", out_png]
    rc, out, err = run_subprocess_capture(cmd)
    return {"cmd": cmd, "returncode": rc, "stdout": out, "stderr": err, "plot": out_png}

# ---------------------- per-robot evaluation ----------------------

def evaluate_per_robot(run_dir: str, robot_dir: str, rid: int, nav_mode: str, out_dir: str, show_plots: bool):
    results = {}
    gt_dir = os.path.join(robot_dir, "ground_truth")
    est_dir = os.path.join(robot_dir, nav_mode)
    gt_file = get_latest_file(gt_dir, 'tum')
    est_file = get_latest_file(est_dir, 'tum')
    results['gt'] = gt_file
    results['est'] = est_file

    if not gt_file or not est_file:
        results['error'] = "Missing ground truth or estimate tum file."
        return results

    safe_mkdir(out_dir)
    # trajectory plot (evo)
    plot_path = os.path.join(out_dir, f"r{rid}_{nav_mode}_trajectory_plot.png")
    evo_traj_res = run_evo_traj(gt_file, est_file, plot_path, show_plot=show_plots)

    # ape via evo (we will print its stdout/stderr raw) - helpful for manual inspection
    ape_plot = os.path.join(out_dir, f"r{rid}_{nav_mode}_ape_plot.png")
    ape_zip = os.path.join(out_dir, f"r{rid}_{nav_mode}_ape_stats.zip")
    ape_res = run_evo_ape(gt_file, est_file, ape_plot, ape_zip, align=True, show_plot=show_plots)

    # compute numeric APE ourselves (rigid alignment, no scale) and RPE delta=1
    ape_stats = compute_ape_stats(gt_file, est_file, max_diff=0.05)
    rpe_stats = compute_rpe(gt_file, est_file, delta=1, max_diff=0.05)

    results['ape_evo_raw'] = {"returncode": ape_res['returncode'], "stdout": ape_res['stdout'], "stderr": ape_res['stderr']}
    results['ape_stats'] = ape_stats
    results['rpe_stats'] = rpe_stats
    results['ape_plot'] = ape_plot
    results['ape_zip'] = ape_zip

    summary = {
        "rid": rid,
        "gt_file": gt_file,
        "est_file": est_file,
        "ape_stats": ape_stats,
        "rpe_stats": rpe_stats,
        "ape_returncode": ape_res.get('returncode', 1),
        "ape_plot": ape_plot,
        "ape_zip": ape_zip
    }
    results['summary'] = summary
    return results

def evaluate_global_snapshots(run_dir: str, robot_dirs: List[str], nav_mode: str, out_dir: str, show_plots: bool):
    results = {}
    global_dir = os.path.join(run_dir, "cslam_global")
    ts_files = list_timestamp_files(global_dir, 'tum')
    results['global_dir'] = global_dir
    results['snapshots'] = []

    for fname in ts_files:
        base = os.path.splitext(fname)[0]
        global_tum = os.path.join(global_dir, fname)
        if not os.path.isfile(global_tum):
            continue
        local_tums = []
        gt_tums = []
        local_counts = 0
        # gather per-robot local estimates and ground-truths that match this snapshot filename
        for rd in robot_dirs:
            local_candidate = os.path.join(rd, nav_mode, fname)
            gt_candidate = os.path.join(rd, "ground_truth", fname)
            if os.path.isfile(local_candidate):
                local_tums.append(local_candidate)
                local_counts += count_tum_poses(local_candidate)
            if os.path.isfile(gt_candidate):
                gt_tums.append(gt_candidate)
        if not gt_tums:
            # nothing to compare for this snapshot
            continue

        merged_gt_path = os.path.join(out_dir, f"merged_gt_{base}.tum")
        merge_tums(gt_tums, merged_gt_path)

        # compute overlay ratio: how many local poses matched against global
        global_pose_count = count_tum_poses(global_tum)

        # compute our numeric APE and RPE between merged ground-truth and global pose graph
        # Use a larger max_diff to account for possible timing mismatch in global pose graph dumps
        ape_stats = compute_ape_stats(merged_gt_path, global_tum, max_diff=0.1)
        rpe_stats = compute_rpe(merged_gt_path, global_tum, delta=1, max_diff=0.1)

        # also run evo for plot (optional)
        plot_path = os.path.join(out_dir, f"global_{base}_ape_plot.png")
        results_zip = os.path.join(out_dir, f"global_{base}_ape_stats.zip")
        ape_res = run_evo_ape(merged_gt_path, global_tum, plot_path, results_zip, align=True, show_plot=show_plots)

        # estimate overlay ratio: how many matched / global count (best-effort)
        # We use associations from compute_ape_stats which returns 'count'
        matched_count = ape_stats.get('count', 0)
        overlay_ratio = (local_counts / global_pose_count) if global_pose_count > 0 else None

        snapshot_entry = {
            "timestamp": base,
            "global_tum": global_tum,
            "merged_gt": merged_gt_path,
            "num_local_poses_sum": local_counts,
            "global_pose_count": global_pose_count,
            "matched_count": matched_count,
            "overlay_ratio": overlay_ratio,
            "ape_stats": ape_stats,
            "rpe_stats": rpe_stats,
            "ape_evo_raw": {"returncode": ape_res['returncode'], "stdout": ape_res['stdout'], "stderr": ape_res['stderr']},
            "ape_plot": plot_path,
            "ape_zip": results_zip
        }
        results['snapshots'].append(snapshot_entry)

    return results

# ---------------------- CLI / main ----------------------

def main():
    parser = argparse.ArgumentParser(description="Run evaluation for a Swarm-SLAM run directory.")
    parser.add_argument('--run-dir', required=True, help='Top-level run directory (where r0 r1 ... and cslam_global are).')
    parser.add_argument('--nav-mode', default='cslam', help='Local graph directory to evaluate (default: cslam).')
    parser.add_argument('--robots', nargs='*', type=int, default=None, help='List of robot IDs to evaluate (default: autodiscover).')
    parser.add_argument('--no-plots', action='store_true', help='Disable showing evo plot windows (still saves images if evo present).')
    parser.add_argument('--out-subdir', default='eval_outputs', help='Subdirectory in run-dir to save evaluation plots/results.')
    args = parser.parse_args()

    run_dir = args.run_dir
    nav_mode = args.nav_mode
    show_plots = not args.no_plots

    if not os.path.isdir(run_dir):
        print(f"ERROR: run-dir {run_dir} does not exist or is not a directory.")
        return

    out_dir = os.path.join(run_dir, args.out_subdir)
    safe_mkdir(out_dir)

    robot_dirs = find_robot_dirs(run_dir)
    if not robot_dirs:
        print("No robot directories found (expected r0, r1, ...).")
    if args.robots:
        filtered = []
        ids = set(args.robots)
        for rd in robot_dirs:
            rid = int(os.path.basename(rd)[1:])
            if rid in ids:
                filtered.append(rd)
        robot_dirs = filtered

    print(f"Discovered robot directories: {[os.path.basename(d) for d in robot_dirs]}")
    summary = {"per_robot": {}, "global": None, "run_dir": run_dir}

    for rd in robot_dirs:
        rid = int(os.path.basename(rd)[1:])
        print(f"\n--- Evaluating robot r{rid} (nav_mode={nav_mode}) ---")
        pr = evaluate_per_robot(run_dir, rd, rid, nav_mode, out_dir, show_plots)
        summary['per_robot'][f"r{rid}"] = pr
        if 'error' in pr:
            print(f" r{rid}: ERROR - {pr['error']}")
        else:
            ape = pr['summary']['ape_stats']
            rpe = pr['summary']['rpe_stats']
            if ape and ape.get('count', 0) > 0:
                print(f" r{rid}: APE translational RMSE: {ape.get('trans_rmse'):.6f} m  (count={ape.get('count')})")
                print(f"    mean: {ape.get('trans_mean'):.6f}, median: {ape.get('trans_median'):.6f}, min: {ape.get('trans_min'):.6f}, max: {ape.get('trans_max'):.6f}")
            else:
                print(" r{rid}: APE: no matches found between GT and EST (check timestamps)")
            if rpe and rpe.get('count', 0) > 0:
                print(f"    RPE translational RMSE: {rpe.get('rpe_trans_rmse'):.6f} m  (count={rpe.get('count')})")
                print(f"    RPE rotational RMSE: {rpe.get('rpe_rot_rmse_deg'):.6f} deg")
            else:
                print("    RPE: not enough matched poses for delta=1")
            print(f"   gt: {pr['summary']['gt_file']}")
            print(f"   est: {pr['summary']['est_file']}")

    print("\n--- Evaluating global snapshots (cslam_global) ---")
    global_res = evaluate_global_snapshots(run_dir, robot_dirs, nav_mode, out_dir, show_plots)
    summary['global'] = global_res

    summary_path = os.path.join(run_dir, "eval_summary.json")
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)

    print("\n=== EVALUATION SUMMARY ===")
    for rname, pr in summary['per_robot'].items():
        print(f"\nRobot {rname}:")
        if 'error' in pr:
            print(f"  ERROR: {pr['error']}")
            continue
        ape = pr['summary']['ape_stats']
        rpe = pr['summary']['rpe_stats']
        if ape and ape.get('count', 0) > 0:
            print(f"  APE translational RMSE: {ape.get('trans_rmse'):.6f} m (count={ape.get('count')})")
        else:
            print("  No APE numeric results (no matches)")
        if rpe and rpe.get('count', 0) > 0:
            print(f"  RPE translational RMSE: {rpe.get('rpe_trans_rmse'):.6f} m; RPE rotational RMSE: {rpe.get('rpe_rot_rmse_deg'):.6f} deg")
        else:
            print("  No RPE (not enough matched pairs)")
        print(f"  ape_plot: {pr.get('ape_plot')}")
        print(f"  ape_zip: {pr.get('ape_zip')}")

    snaps = global_res.get('snapshots', [])
    print(f"\nGlobal snapshots evaluated: {len(snaps)}")
    for s in snaps:
        base = s['timestamp']
        overlay = s.get('overlay_ratio')
        matched = s.get('matched_count')
        gp = s.get('global_pose_count')
        ape_s = s.get('ape_stats', {})
        rpe_s = s.get('rpe_stats', {})
        print(f" - {base}: overlay_ratio={overlay}, global_poses={gp}, matched={matched}")
        if ape_s and ape_s.get('count', 0) > 0:
            print(f"     APE RMSE: {ape_s.get('trans_rmse'):.6f} m (count={ape_s.get('count')})")
        else:
            print("     APE: <none> (check timestamps or timestamp tolerance)")
        if rpe_s and rpe_s.get('count', 0) > 0:
            print(f"     RPE RMSE: {rpe_s.get('rpe_trans_rmse'):.6f} m; RPE rot RMSE: {rpe_s.get('rpe_rot_rmse_deg'):.6f} deg")
        else:
            print("     RPE: <none>")

    print(f"\nSaved summary JSON to: {summary_path}")
    print(f"Plots & evo results saved under: {out_dir}")


if __name__ == "__main__":
    main()
