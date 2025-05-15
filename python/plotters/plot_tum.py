#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_tum(filename):
    data = np.loadtxt(filename)
    t    = data[:,0]
    xyz  = data[:,1:4]
    return t, xyz

def plot_trajectory(xyz, skip=1):
    fig = plt.figure(figsize=(8,6))
    ax  = fig.add_subplot(111, projection='3d')
    # main trajectory
    ax.plot(xyz[:,0], xyz[:,1], xyz[:,2], '-', lw=1, label='trajectory')
    ax.scatter(xyz[::skip,0], xyz[::skip,1], xyz[::skip,2],
               c='gray', s=10, label=f'every {skip}-th pose')

    # mark start with a big red circle
    ax.scatter(xyz[0,0], xyz[0,1], xyz[0,2],
               c='red', marker='o', s=100, label='start')

    # mark end   with a big black square
    ax.scatter(xyz[-1,0], xyz[-1,1], xyz[-1,2],
               c='black', marker='s', s=100, label='end')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('TUM Trajectory')
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 plot_tum.py <tum_pose_file.txt>")
        sys.exit(1)

    filename = sys.argv[1]
    t, xyz = load_tum(filename)
    skip = max(1, len(xyz)//500)
    plot_trajectory(xyz, skip)

