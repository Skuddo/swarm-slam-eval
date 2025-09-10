import os
import re
import json
import argparse
import subprocess
from typing import Optional, List, Dict, Tuple

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

def parse_evo_ape_stdout(stdout: str) -> Dict[str, float]:
    res = {}
    s = stdout.lower()
    patterns = {
        'trans_rmse': [
            r"trans.*rmse[:\s]+([0-9eE+.\-]+)",
            r"translation.*rmse[:\s]+([0-9eE+.\-]+)",
            r"abs.*trans.*rmse[:\s]+([0-9eE+.\-]+)"
        ],
        'rot_rmse': [
            r"rot.*rmse[:\s]+([0-9eE+.\-]+)",
            r"rotation.*rmse[:\s]+([0-9eE+.\-]+)",
        ],
        'trans_mean': [
            r"trans.*mean[:\s]+([0-9eE+.\-]+)",
            r"translation.*mean[:\s]+([0-9eE+.\-]+)"
        ],
        'trans_median': [
            r"trans.*median[:\s]+([0-9eE+.\-]+)",
        ]
    }
    for key, pats in patterns.items():
        for pat in pats:
            m = re.search(pat, s)
            if m:
                try:
                    res[key] = float(m.group(1))
                    break
                except:
                    continue
    return res

def safe_mkdir(path: str):
    os.makedirs(path, exist_ok=True)

def run_evo_ape(gt_tum: str, est_tum: str, out_png: str, out_zip: str, align: bool = True, show_plot: bool = True) -> Dict:
    cmd = ["evo_ape", "tum", gt_tum, est_tum]
    if align:
        cmd += ["--align"]
    if show_plot:
        cmd += ["-p"]
    cmd += ["--save_plot", out_png, "--save_results", out_zip]
    rc, out, err = run_subprocess_capture(cmd)
    print
    return {
        "cmd": cmd,
        "returncode": rc,
        "stdout": out,
        "stderr": err,
        # "parsed": parsed,
        "plot": out_png,
        "results": out_zip
    }

def run_evo_traj(gt_tum: str, est_tum: str, out_png: str, show_plot: bool = True) -> Dict:
    cmd = ["evo_traj", "tum", gt_tum, est_tum, "--ref", gt_tum]
    if show_plot:
        cmd += ["-p"]
    cmd += ["--save_plot", out_png]
    rc, out, err = run_subprocess_capture(cmd)
    return {"cmd": cmd, "returncode": rc, "stdout": out, "stderr": err, "plot": out_png}

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

def extract_timestamp_from_fname(fname: str) -> str:
    return os.path.splitext(os.path.basename(fname))[0]

def evaluate_per_robot(run_dir: str, robot_dir: str, robot_id: int, nav_mode: str, out_dir: str, show_plots: bool):
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
    plot_path = os.path.join(out_dir, f"r{robot_id}_{nav_mode}_trajectory_plot.png")
    run_evo_traj(gt_file, est_file, plot_path, show_plot=show_plots)

    ape_plot = os.path.join(out_dir, f"r{robot_id}_{nav_mode}_ape_plot.png")
    ape_zip = os.path.join(out_dir, f"r{robot_id}_{nav_mode}_ape_stats.zip")
    ape_res = run_evo_ape(gt_file, est_file, ape_plot, ape_zip, align=True, show_plot=show_plots)
    results['ape'] = ape_res

    summary = {
        "robot_id": robot_id,
        "gt_file": gt_file,
        "est_file": est_file,
        "ape_parsed": ape_res.get("parsed", {}),
        "ape_returncode": ape_res.get("returncode", 1),
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
        local_tums = []
        gt_tums = []
        local_counts = 0
        for rd in robot_dirs:
            robot_name = os.path.basename(rd)
            local_candidate = os.path.join(rd, nav_mode, fname)
            gt_candidate = os.path.join(rd, "ground_truth", fname)
            if os.path.isfile(local_candidate):
                local_tums.append(local_candidate)
                local_counts += count_tum_poses(local_candidate)
            if os.path.isfile(gt_candidate):
                gt_tums.append(gt_candidate)
        if not gt_tums:
            continue

        merged_gt_path = os.path.join(out_dir, f"merged_gt_{base}.tum")
        merge_tums(gt_tums, merged_gt_path)
        plot_path = os.path.join(out_dir, f"global_{base}_ape_plot.png")
        results_zip = os.path.join(out_dir, f"global_{base}_ape_stats.zip")
        ape_res = run_evo_ape(merged_gt_path, global_tum, plot_path, results_zip, align=True, show_plot=show_plots)

        global_pose_count = count_tum_poses(global_tum)
        overlay_ratio = (local_counts / global_pose_count) if global_pose_count > 0 else None

        snapshot_entry = {
            "timestamp": base,
            "global_tum": global_tum,
            "merged_gt": merged_gt_path,
            "num_local_poses_sum": local_counts,
            "global_pose_count": global_pose_count,
            "overlay_ratio": overlay_ratio,
            "ape": ape_res
        }
        results['snapshots'].append(snapshot_entry)

    return results

def main():
    parser = argparse.ArgumentParser(description="Run evaluation for a Swarm-SLAM run directory.")
    parser.add_argument('--run-dir', required=True, help='Top-level run directory (where r0 r1 ... and cslam_global are).')
    parser.add_argument('--nav-mode', default='cslam', help='Local graph directory to evaluate (default: cslam).')
    parser.add_argument('--robots', nargs='*', type=int, default=None, help='List of robot IDs to evaluate (default: autodiscover).')
    parser.add_argument('--no-plots', action='store_true', help='Disable showing evo plot windows (still saves images).')
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
            parsed = pr['summary']['ape_parsed']
            print(f" r{rid}: APE parsed stats: {parsed if parsed else 'No parsed stats (check evo stdout)'}")
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
        ape_parsed = pr['summary']['ape_parsed']
        if ape_parsed:
            for k, v in ape_parsed.items():
                print(f"  {k}: {v}")
        else:
            print("  No ape parsed stats (check evo stdout/stderr).")
        print(f"  ape_plot: {pr['summary']['ape_plot']}")
        print(f"  ape_zip: {pr['summary']['ape_zip']}")

    print("\nGlobal snapshots evaluated:", len(global_res.get('snapshots', [])))
    for s in global_res.get('snapshots', []):
        base = s['timestamp']
        overlay = s['overlay_ratio']
        print(f" - {base}: overlay_ratio={overlay}, global_poses={s['global_pose_count']}")
        parsed = s['ape'].get('parsed', {})
        if parsed:
            print(f"     ape_parsed: {parsed}")
        else:
            print("     ape_parsed: <none> (check evo output)")

    print(f"\nSaved summary JSON to: {summary_path}")
    print(f"Plots & evo results saved under: {out_dir}")

if __name__ == "__main__":
    main()
