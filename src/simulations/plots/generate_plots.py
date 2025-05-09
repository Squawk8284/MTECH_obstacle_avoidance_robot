#!/usr/bin/env python3
 
import os
import glob
import pandas as pd
import numpy as np
 
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Circle
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 for 3D projection
 
def parse_path_topic_points(path_str):
    """Parse a 'x1,y1;x2,y2;...' string into two numpy arrays: x and y."""
    if isinstance(path_str, float) or pd.isna(path_str):
        return np.array([]), np.array([])
    pairs = path_str.strip().split(';')
    xs, ys = [], []
    for p in pairs:
        if not p:
            continue
        try:
            x, y = map(float, p.split(','))
            xs.append(x)
            ys.append(y)
        except ValueError:
            continue
    return np.array(xs), np.array(ys)
 
def make_plots_for_folder(folder_path, human_img, obstacle_img):
    """Generate and save five plots, including Plot 5 with both obstacle and human icons."""
    folder_name = os.path.basename(folder_path)
    csv_files  = glob.glob(os.path.join(folder_path, 'record_*.csv'))
    if not csv_files:
        print(f"No CSV found in {folder_path}, skipping.")
        return
 
    df = pd.read_csv(csv_files[0])
    # Zero-base time
    t  = df['time'].values - df['time'].values[0]
    x  = df['odom_pos_x'].values
    y  = df['odom_pos_y'].values
    vx = df['cmd_lin_x'].values
    wz = df['cmd_ang_z'].values
 
    # ------------------------------
    # Plot 1: 3D Odometry Over Time
    # ------------------------------
    fig = plt.figure(figsize=(10, 8), constrained_layout=True)
    ax  = fig.add_subplot(111, projection='3d')
    ax.scatter(y, x, t, color='black', s=5, label="Odometry")
    ax.scatter(y[0], x[0], t[0], c='r', s=100, label="Start")
    ax.scatter(y[-1], x[-1], t[-1], c='g', s=100, label="End", marker='x')
    ax.text(y[0], x[0], t[0], f"Start ({x[0]:.2f},{y[0]:.2f},{t[0]:.2f})",
            ha="right", va="bottom")
    ax.text(y[-1], x[-1], t[-1], f"End   ({x[-1]:.2f},{y[-1]:.2f},{t[-1]:.2f})",
            ha="right", va="bottom")
    ax.set_xlabel('Odometry Y [m]')
    ax.set_ylabel('Odometry X [m]')
    ax.set_zlabel('Time [s]')
    ax.set_title('3D Odometry Position Over Time')
    ax.set_xlim(6, 0)
    ax.set_ylim(0, 5)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    fig.savefig(os.path.join(folder_path, f'plot1_odometry_{folder_name}.png'))
    plt.close(fig)
 
    # ------------------------------
    # Plot 2: Linear Velocity
    # ------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, vx, label='Linear Velocity', color='tab:blue')
 
    # Extend to final zero point using ax.plot
    if len(t) > 1:
        dt = t[1] - t[0]
    else:
        dt = 0.1  # fallback
    final_time = t[-1] + dt
    ax.plot([t[-1], final_time], [vx[-1], 0.0], color='tab:blue', linestyle='--')
 
    # Optionally mark the final zero point
    ax.scatter(final_time, 0.0, color='tab:blue')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Linear Velocity [m/s]")
    ax.set_title("Linear Velocity (cmd_lin_x)")
    ax.set_ylim(-0.5, 0.5)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
    fig.savefig(os.path.join(folder_path, f'plot2_linear_velocity_{folder_name}.png'),
                bbox_inches='tight')
    plt.close(fig)
 
    # ------------------------------
    # Plot 3: Angular Velocity
    # ------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, wz, label='Angular Velocity', color='tab:orange')
 
    # Extend to final zero point using ax.plot
    if len(t) > 1:
        dt = t[1] - t[0]
    else:
        dt = 0.1  # fallback
    final_time = t[-1] + dt
    ax.plot([t[-1], final_time], [wz[-1], 0.0], color='tab:orange', linestyle='--')
 
    # Optionally mark the final zero point
    ax.scatter(final_time, 0.0, color='tab:orange')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Velocity [rad/s]")
    ax.set_title("Angular Velocity (cmd_ang_z)")
    ax.set_ylim(-1.5, 1.5)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
    fig.savefig(os.path.join(folder_path, f'plot3_angular_velocity_{folder_name}.png'),
                bbox_inches='tight')
    plt.close(fig)
 
    # ------------------------------
    # Plot 4: Error from Goal & Distance from Start
    # ------------------------------
    goal_x, goal_y = df['goal_x'].iloc[0], df['goal_y'].iloc[0]
    error_from_goal = np.hypot(x - goal_x, y - goal_y)
    dist_from_start = np.hypot(x - x[0],    y - y[0])
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, dist_from_start, label='Distance from Start', color='tab:purple')
    ax.plot(t, error_from_goal, '--', label='Goal Error', color='tab:green')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Distance [m]")
    ax.set_title("Error from Goal & Distance from Start Over Time")
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    fig.savefig(os.path.join(folder_path, f'plot4_error_and_start_distance_{folder_name}.png'),
                bbox_inches='tight')
    plt.close(fig)
 
    # ------------------------------
    # Plot 5: 2D Motion + Obstacles + Humans
    # ------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(y, x, label='2D Motion', color='tab:red')
    ax.scatter(y[0], x[0], c='r', s=100, label='Start')
    ax.scatter(y[-1], x[-1], c='g', s=100, label='End', marker='x')
 
    ax.annotate(
    f"Start ({x[0]:.2f}, {y[0]:.2f})",
    xy=(y[0], x[0]),
    xytext=(-50, -10),  # (x_offset, y_offset) in points
    textcoords='offset points',
    ha="left", va="top",
    fontsize=9)
 
    ax.annotate(
    f"End   ({x[-1]:.2f},{y[-1]:.2f})",
    xy=(y[-1], x[-1]),
    xytext=(50, 10),  # (x_offset, y_offset) in points
    textcoords='offset points',
    ha="right", va="bottom",
    fontsize=9)
 
 
    # Static obstacles: circles + icon
    obstacle_data = {}
    with open('obstacles.txt', 'r') as f:
        exec(f.read(), {}, obstacle_data)
 
    static_obstacles = obstacle_data['static_obstacles']
    human_obstacles = obstacle_data['human_obstacles']
 
    scenario = folder_name[:2]  # e.g., '2b'
    scenario_digit = folder_name[0]  # e.g., '2'
 
    # Static Obstacles: circle + icon
    if static_obstacles.get(scenario):
        obs_icon = OffsetImage(obstacle_img, zoom=0.015)
        for i, (oy, ox) in enumerate(static_obstacles[scenario]):
            circ = Circle((ox, oy), radius=0.15,
                        edgecolor='orange', facecolor='none',
                        label="Static obstacle" if i == 0 else None)
            safe = Circle((ox, oy), radius=0.73, linestyle='--',
                        edgecolor='orange', facecolor='none',
                        label="Static safety distance" if i == 0 else None)
            ab_obs = AnnotationBbox(
                obs_icon, (ox, oy),
                frameon=False, box_alignment=(0.5, 0.5)
            )
            ax.add_patch(circ)
            ax.add_artist(ab_obs)
            ax.add_patch(safe)
 
    # Human positions: circles + icon
    if 'b' in folder_name and human_obstacles.get(scenario_digit):
        hum_icon = OffsetImage(human_img, zoom=0.01)
        for i, (hy, hx) in enumerate(human_obstacles[scenario_digit]):
            body = Circle((hx, hy), radius=0.19,
                        edgecolor='b', facecolor='none',
                        label="Stationary Human" if i == 0 else None)
            safety = Circle((hx, hy), radius=0.77, linestyle='--',
                            edgecolor='b', facecolor='none',
                            label="Human safety distance" if i == 0 else None)
            ab_hum = AnnotationBbox(
                hum_icon, (hx, hy),
                frameon=False, box_alignment=(0.5, 0.5)
            )
            ax.add_patch(body)
            ax.add_artist(ab_hum)
            ax.add_patch(safety)
 
 
    ax.set_xlabel("Odometry Y [m]")
    ax.set_ylabel("Odometry X [m]")
    ax.set_title("2D Motion: Path from Start to End")
    ax.set_xlim(6, 0)
    ax.set_ylim(0, 6)
    ax.grid(True)
    ax.yaxis.set_ticks_position("right")
    ax.yaxis.set_label_position("right")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
 
    fig.savefig(os.path.join(folder_path, f'plot5_2d_motion_{folder_name}.png'),
                bbox_inches='tight')
    plt.close(fig)
 
    print(f"✅ Plots saved in {folder_path}")
 
def main():
    base_dir     = os.path.dirname(__file__)
    human_path   = os.path.join(base_dir, 'human_icon.png')
    obstacle_path= os.path.join(base_dir, 'obstacle.png')
    human_img    = mpimg.imread(human_path)
    obstacle_img = mpimg.imread(obstacle_path)
 
    subdirs = sorted(glob.glob(os.path.join(base_dir, '**'), recursive=True))
    for d in subdirs:
        if os.path.isdir(d):
            print(f"Processing {d} ...")
            make_plots_for_folder(d, human_img, obstacle_img)
 
if __name__ == '__main__':
    main()
