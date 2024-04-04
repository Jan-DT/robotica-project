import numpy as np
import matplotlib.pyplot as plt


def load_grid_from_file(filename: str):
    with open(filename, 'rb') as f:
        return np.load(f)


if __name__ == "__main__":
    grid = load_grid_from_file('grid.npy')
    plt.imshow(grid)
    plt.colorbar()
    plt.show()
