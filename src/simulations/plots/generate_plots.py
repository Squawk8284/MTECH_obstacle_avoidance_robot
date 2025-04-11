#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
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
    if not csv_files:
        print(f"No CSV found in {folder_path}, skipping.")
        return

    csv_path = csv_files[0]
    df = pd.read_csv(csv_path)

    t = [i - df['time'].values[0] for i in df['time'].values]
    x = df['odom_pos_x'].values
    y = df['odom_pos_y'].values
    vx = df['cmd_lin_x'].values
    wz = df['cmd_ang_z'].values

    # Plot 1: Odometry 3D only
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, t, color='black', s=5, label="Odometry")
    ax.set_xlabel('Odometry X [m]')
    ax.set_ylabel('Odometry Y [m]')
    ax.set_zlabel('Time [s]')
    ax.set_title('3D Odometry Position Over Time')
    ax.scatter(x[0], y[0], t[0], c='r', s=100, label="Start Point")
    ax.text(x[0], y[0], t[0], s=f"Start {round(x[0],2), round(y[0],2), round(t[0],2)}", ha="right", va="bottom")
    ax.text(x[-1], y[-1], t[-1], s=f"End {round(x[-1],2), round(y[-1],2), round(t[-1],2)}", ha="right", va="bottom")
    ax.scatter(x[-1], y[-1], t[-1], c='g', s=100, label="End Point")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, 'plot1_odometry.png'), bbox_inches='tight')
    plt.close(fig)

    # Plot 2: Linear Velocity
    fig, ax = plt.subplots()
    ax.plot(t, vx, label='Linear Velocity', color='tab:blue')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Linear Velocity [m/s]")
    ax.set_title("Linear Velocity (cmd_lin_x)")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, 'plot2_linear_velocity.png'), bbox_inches='tight')
    plt.close(fig)

    # Plot 3: Angular Velocity
    fig, ax = plt.subplots()
    ax.plot(t, wz, label='Angular Velocity', color='tab:orange')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Velocity [rad/s]")
    ax.set_title("Angular Velocity (cmd_ang_z)")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, 'plot3_angular_velocity.png'), bbox_inches='tight')
    plt.close(fig)

    # Compute error from goal and distance from start
    goal_x, goal_y = df['goal_x'].iloc[0], df['goal_y'].iloc[0]
    error_from_goal = np.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)

    start_x, start_y = x[0], y[0]
    dist_from_start = np.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)

    # Plot 4: Combined - Goal Error and Distance from Start
    fig, ax = plt.subplots()
    ax.plot(t, dist_from_start, label='Distance from Start', color='tab:purple')
    ax.plot(t, error_from_goal, '--', label='Goal Error', color='tab:green')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Distance [m]")
    ax.set_title("Error from Goal & Distance from Start Over Time")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    plt.tight_layout()
    fig.savefig(os.path.join(folder_path, 'plot4_error_and_start_distance.png'), bbox_inches='tight')
    plt.close(fig)

    print(f"✅ Plots saved in {folder_path}")


def main():
    base = os.path.join(os.path.dirname(__file__), 'record_*')
    subdirs = sorted(glob.glob(base))
    if not subdirs:
        print("No record_* subfolders found.")
        return

    for folder in subdirs:
        print(f"Processing {folder} ...")
        make_plots_for_folder(folder)


if __name__ == '__main__':
    main()
