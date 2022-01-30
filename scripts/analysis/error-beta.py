#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

def process_file(filename, robots):
    s = pd.read_csv(filename).to_numpy().reshape((-1, robots, 2, 2))
    error = s[:,:,1] - s[:,:,0]
    linear = np.sqrt(error[:,:,0] ** 2 + error[:,:,1] ** 2)
    return np.median(linear)

def main(filenames, labels, robots):
    for i, names in enumerate(filenames):
        errors = [process_file(name, robots[i]) for name in names]
        plt.plot(labels, errors, marker='o', linestyle=' ', label='{} robots'.format(robots[i]))

    plt.ylabel('Median Error (m)')
    plt.xlabel('Significance of inter-robot ranging')
    plt.legend()

    if input('Save? ') == 'y':
        name = input('name: ')
        plt.savefig(name)

    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 4:
        sys.exit(1)
    robots = [int(i) for i in sys.argv[1].split(' ')]
    labels = sys.argv[2].split(' ')
    filename_series = map(lambda s: s.split(' '), sys.argv[3:])
    main(filename_series, labels, robots)
