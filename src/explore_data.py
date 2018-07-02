"""

Script for exploring the data for this project

"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def main():
    cols = ['x', 'y', 's', 'dx', 'dy']
    d = pd.read_csv("../data/highway_map.csv", header=None, names=cols, sep=' ')

    # Show the overall track in cartesian coords with the unit vectors shown
    # pointing perpendicular to the direction of the track
    plt.plot(d.x, d.y)
    for idx, row in d.iterrows():
        plt.arrow(row.x, row.y, row.dx*100, row.dy*100, fc="k", ec="k", head_width=0.05, head_length=0.1)
    plt.show()


if __name__ == '__main__':
    main()
