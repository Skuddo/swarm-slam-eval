import os
import argparse
import subprocess

# Get the most recent file from the provided directory
# using the provided fieltype 
def getLatestFile(directory: str, type: str):
    g2o_files = [f for f in os.listdir(directory) if f.endswith(f".{type}")]
    if not g2o_files:
        return None
    
    latest_file = None
    latest_timestamp = 0.0

    for filename in g2o_files:
        try:
            ts = float(filename[:-4])
            
            if ts > latest_timestamp:
                latest_timestamp = ts
                latest_file = filename
        except (ValueError, IndexError):
            continue

    return os.path.join(directory, latest_file) if latest_file else None

def main():
    parser = argparse.ArgumentParser(
        description="Evaluate a trajectory from a results folder against ground truth using evo.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        '--run_dir', type=str,
        help='Path to the top-level run directory.'
        )
    parser.add_argument(
        '--robot_id', type=int,
        default=1, 
        help='The ID of the robot to evaluate.'
        )
    parser.add_argument(
        '--nav_mode', type=str,
        default='imu', 
        help='The navigation mode (e.g., imu, vins) to evaluate.'
        )
    parser.add_argument(
        '--no_plots', action='store_true',
        help='Disable showing plot windows (still saves them).'
        )
    args = parser.parse_args()

    # get files
    gt_path = os.path.join(args.run_dir, f"r{args.robot_id}", "ground_truth")
    est_path = os.path.join(args.run_dir, f"r{args.robot_id}", args.nav_mode)
    gt_file = getLatestFile(gt_path, 'tum')
    est_file = getLatestFile(est_path, 'tum')

    print(f"Comparing files for Robot {args.robot_id}:")
    print(f"   - Ground Truth: {gt_file}")
    print(f"   - Estimate ({args.nav_mode}): {est_file}")

    # Trajectories
    print("\n--- Visualizing trajectories (evo_traj) ---")
    plot_path = os.path.join(args.run_dir, f"r{args.robot_id}_{args.nav_mode}_trajectory_plot.png")
    traj_command = ["evo_traj", "tum", gt_file, est_file, "--ref", gt_file, "--align", "-p", "--save_plot", plot_path]
    if args.no_plots: traj_command.remove("-p")
    subprocess.run(traj_command)

    # APE
    print("\n--- Calculating APE (Absolute Pose Error) ---")
    ape_plot_path = os.path.join(args.run_dir, f"r{args.robot_id}_{args.nav_mode}_ape_plot.png")
    ape_results_path = os.path.join(args.run_dir, f"r{args.robot_id}_{args.nav_mode}_ape_stats.zip")
    ape_command = ["evo_ape", "tum", gt_file, est_file, "--align", "-p", "--save_plot", ape_plot_path, "--save_results", ape_results_path]
    if args.no_plots: ape_command.remove("-p")
    subprocess.run(ape_command)
    
    # RPE
    print("\n--- Calculating RPE (Relative Pose Error) ---")
    rpe_plot_path = os.path.join(args.run_dir, f"r{args.robot_id}_{args.nav_mode}_rpe_plot.png")
    rpe_results_path = os.path.join(args.run_dir, f"r{args.robot_id}_{args.nav_mode}_rpe_stats.zip")
    rpe_command = ["evo_rpe", "tum", gt_file, est_file, "--align", "-p", "--save_plot", rpe_plot_path, "--save_results", rpe_results_path]
    if args.no_plots: rpe_command.remove("-p")
    subprocess.run(rpe_command)
    
    print(f"\n✅ Evaluation complete. Plots and stats saved in {args.run_dir}")

if __name__ == '__main__':
    main()