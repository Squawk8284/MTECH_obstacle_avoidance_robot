#!/usr/bin/env python3
import os
import glob
import pandas as pd

# Force non-GUI backend before importing pyplot
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def make_plots_for_folder(folder_path):
    # find the single CSV in this folder
    csv_files = glob.glob(os.path.join(folder_path, 'record_*.csv'))
    if not csv_files:
        print(f"No CSV found in {folder_path}, skipping.")
        return
    csv_path = csv_files[0]
    df = pd.read_csv(csv_path)

    # extract as numpy arrays
    t = df['time'].values
    x = df['odom_pos_x'].values
    y = df['odom_pos_y'].values
    vx = df['cmd_lin_x'].values
    wz = df['cmd_ang_z'].values

    # 1) plot1: Odometry (3D)
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, t, c=t, cmap='viridis', s=5)
    ax.set_xlabel('odom_pos_x')
    ax.set_ylabel('odom_pos_y')
    ax.set_zlabel('time')
    ax.set_title('Odometry')
    plt.tight_layout()
    out1 = os.path.join(folder_path, 'plot1_odometry.png')
    fig.savefig(out1)
    plt.close(fig)

    # 2) plot2: Linear Velocity
    fig, ax = plt.subplots(figsize=(6,4))
    ax.plot(t, vx, color='tab:blue')
    ax.set_xlabel('time')
    ax.set_ylabel('cmd_lin_x')
    ax.set_title('Linear Velocity (cmd_lin_x)')
    plt.tight_layout()
    out2 = os.path.join(folder_path, 'plot2_linear_velocity.png')
    fig.savefig(out2)
    plt.close(fig)

    # 3) plot3: Angular Velocity
    fig, ax = plt.subplots(figsize=(6,4))
    ax.plot(t, wz, color='tab:orange')
    ax.set_xlabel('time')
    ax.set_ylabel('cmd_ang_z')
    ax.set_title('Angular Velocity (cmd_ang_z)')
    plt.tight_layout()
    out3 = os.path.join(folder_path, 'plot3_angular_velocity.png')
    fig.savefig(out3)
    plt.close(fig)

    print(f"  Plots saved in {folder_path}")

def main():
    # assume this script lives alongside the 'plots/' directory
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
