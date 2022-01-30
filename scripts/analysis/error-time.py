#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

def main(filename, robots):
    s = pd.read_csv(filename).to_numpy().reshape((-1, robots, 2, 2))
    error = s[:,:,1] - s[:,:,0]
    linear = np.sqrt(error[:,:,0] ** 2 + error[:,:,1] ** 2)

    plt.plot(linear)
    plt.ylabel('Error (m)')
    plt.xlabel('Time')
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(1)
    filename = sys.argv[1]
    robots = int(sys.argv[2])
    main(filename, robots)
