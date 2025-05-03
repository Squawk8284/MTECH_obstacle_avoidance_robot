#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D


def parse_path_topic_points(path_str):
    """Parse a 'x1,y1;x2,y2;...' string into two numpy arrays: x and y."""
    if isinstance(path_str, float) or pd.isna(path_str):
        return np.array([]), np.array([])
    
    pairs = path_str.strip().split(';')
    x_vals, y_vals = [], []
    for pair in pairs:
        if not pair:
            continue
        try:
            x, y = map(float, pair.split(','))
            x_vals.append(x)
            y_vals.append(y)
        except ValueError:
            continue
    return np.array(x_vals), np.array(y_vals)


def make_plots_for_folder(folder_path):
    csv_files = glob.glob(os.path.join(folder_path, 'record_*.csv'))
    folder_name = os.path.basename(folder_path)
    if not csv_files:
        print(f"No CSV found in {folder_path}, skipping.")
        return
    csv_path = csv_files[0]
    df = pd.read_csv(csv_path)

    # Adjust time so that it starts at 0.
    t = [i - df['time'].values[0] for i in df['time'].values]
    x = df['odom_pos_x'].values
    y = df['odom_pos_y'].values
    vx = df['cmd_lin_x'].values
    wz = df['cmd_ang_z'].values

    # ------------------------------
    # Plot 1: 3D Odometry Over Time (X and Y Swapped)
    # ------------------------------
    fig = plt.figure(figsize=(10, 8), constrained_layout=True)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(y, x, t, color='black', s=5, label="Odometry")
    ax.set_xlabel('Odometry Y [m]')
    ax.set_ylabel('Odometry X [m]')
    ax.set_zlabel('Time [s]')
    ax.set_title('3D Odometry Position Over Time')

    ax.scatter(y[0], x[0], t[0], c='r', s=100, label="Start Point")
    ax.text(y[0], x[0], t[0],
            s=f"Start ({x[0]:.2f}, {y[0]:.2f}, {t[0]:.2f})",
            ha="right", va="bottom")
    ax.text(y[-1], x[-1], t[-1],
            s=f"End ({x[-1]:.2f}, {y[-1]:.2f}, {t[-1]:.2f})",
            ha="right", va="bottom")
    ax.scatter(y[-1], x[-1], t[-1], c='g', s=100, label="End Point")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)

    ax.set_xlim(6, 0)  # Y-axis range
    ax.set_ylim(0, 5)  # X-axis reversed

    fig.savefig(os.path.join(folder_path, f'plot1_odometry_{folder_name}.png'))
    plt.close(fig)

    # ------------------------------
    # Plot 2: Linear Velocity
    # ------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, vx, label='Linear Velocity', color='tab:blue')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Linear Velocity [m/s]")
    ax.set_title("Linear Velocity (cmd_lin_x)")
    ax.set_ylim(-0.5, 0.5)
    ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5, zorder=0)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, f'plot2_linear_velocity_{folder_name}.png'), bbox_inches='tight')
    plt.close(fig)

    # ------------------------------
    # Plot 3: Angular Velocity
    # ------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, wz, label='Angular Velocity', color='tab:orange')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Velocity [rad/s]")
    ax.set_title("Angular Velocity (cmd_ang_z)")
    ax.set_ylim(-1.5, 1.5)
    ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5, zorder=0)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, f'plot3_angular_velocity_{folder_name}.png'), bbox_inches='tight')
    plt.close(fig)

    # ------------------------------
    # Plot 4: Error from Goal & Distance from Start
    # ------------------------------
    goal_x, goal_y = df['goal_x'].iloc[0], df['goal_y'].iloc[0]
    error_from_goal = np.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)
    start_x, start_y = x[0], y[0]
    dist_from_start = np.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, dist_from_start, label='Distance from Start', color='tab:purple')
    ax.plot(t, error_from_goal, '--', label='Goal Error', color='tab:green')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Distance [m]")
    ax.set_title("Error from Goal & Distance from Start Over Time")
    ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5, zorder=0)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, f'plot4_error_and_start_distance_{folder_name}.png'), bbox_inches='tight')
    plt.close(fig)

    # ------------------------------
    # Plot 5: 2D Motion (X and Y Swapped)
    # ------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(y, x, label='2D Motion', color='tab:red')
    ax.scatter(y[0], x[0], c='r', s=100, label='Start')
    ax.scatter(y[-1], x[-1], c='g', s=100, label='End')

    ### New Part
    obstacles = {'1':None, '2':[(2.2, 2.7), (2.5, 2.4), (2.5, 2.7), (2.5, 3.0), (2.8,2.7)], '3':[(2.75, 1.60), (0.95, 3.43), (2.8, 5.2)]}
    humans = {'1':[(3.78, 0.35), (2.65, 3.35), (1.29, 4.3)], '2':[(2.2, 1.35), (4.02, 0.88), (1.9, 5.36)], '3':[(4.12, 0.88), (1.1, 5.2), (2.8, 4.3)]}

    humans_present = False
    if 'b' in folder_name:
        humans_present = True
    scenario = folder_name[0]

    if obstacles[scenario]:
        obstacle = np.array(obstacles[scenario])
        for i, obs in enumerate(obstacle):

            circle = Circle((obs[1], obs[0]), radius=0.15, edgecolor='orange', label="Static obstacle" if i==0 else None, facecolor='orange')
            safety_circle = Circle((obs[1], obs[0]), radius=0.73, ls='--', edgecolor='orange', facecolor='none', label="Static safety distance" if i==0 else None)
            ax.add_patch(safety_circle)
            ax.add_patch(circle)

    if humans_present:
        human = np.array(humans[scenario])
        for i, hum in enumerate(human):
            circle = Circle((hum[1], hum[0]), radius=0.19, edgecolor='b', label="Stationary Human" if i == 0 else None)
            safety_circle = Circle((hum[1], hum[0]), radius=0.77, ls='--', edgecolor='b', facecolor='none', label="Human safety distance" if i == 0 else None)
            ax.add_patch(safety_circle)
            ax.add_patch(circle)

    ###
    offset_x, offset_y = 0.3, 0.3
    ax.text(y[0] - offset_y, x[0] + offset_x,
            s=f"Start ({x[0]:.2f}, {y[0]:.2f})",
            ha="right", va="bottom", fontsize=9)
    ax.text(y[-1] + offset_y, x[-1] - offset_x,
            s=f"End ({x[-1]:.2f}, {y[-1]:.2f})",
            ha="left", va="top", fontsize=9)

    ax.set_xlabel("Odometry Y [m]")
    ax.set_ylabel("Odometry X [m]")
    ax.set_title("2D Motion: Path from Start to End")
    ax.set_xlim(6, 0)  # Y-axis range
    ax.set_ylim(0, 5)  # X-axis reversed

    ax.yaxis.set_ticks_position("right")
    ax.yaxis.set_label_position("right")

    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, f'plot5_2d_motion_{folder_name}.png'), bbox_inches='tight')
    plt.close(fig)

    print(f"✅ Plots saved in {folder_path}")

def main():
    base_dir = os.path.dirname(__file__)
    pattern = os.path.join(base_dir, '**')
    subdirs = sorted(glob.glob(pattern, recursive=True))
    
    if not subdirs:
        print("No record_* folders found.")
        return

    for folder in subdirs:
        if os.path.isdir(folder):
            print(f"Processing {folder} ...")
            make_plots_for_folder(folder)
        else:
            print(f"Skipping {folder} (not a folder).")


if __name__ == '__main__':
    main()
