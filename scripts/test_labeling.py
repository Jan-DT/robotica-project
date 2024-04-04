import numpy as np
from matplotlib import pyplot as plt


def label(grid):
    """
    Label objects in a grid
    :param grid: Occupancy grid, 0 is free space
    :return: Tuple of the labeled grid and the number of labels
    """
    labeled_grid = np.zeros_like(grid)
    num_labels = 0
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] > 0 and labeled_grid[i, j] == 0:
                num_labels += 1
                stack = [(i, j)]
                while stack:
                    x, y = stack.pop()
                    if x < 0 or x >= grid.shape[0] or y < 0 or y >= grid.shape[1]:
                        continue
                    if grid[x, y] == 0 or labeled_grid[x, y] > 0:
                        continue
                    labeled_grid[x, y] = num_labels
                    stack.append((x + 1, y))
                    stack.append((x - 1, y))
                    stack.append((x, y + 1))
                    stack.append((x, y - 1))

    return labeled_grid, num_labels


def main():
    GRID_SIZE = 50
    GRID = np.zeros((GRID_SIZE, GRID_SIZE))

    def draw_circle(_grid, x, y, r):
        for _i in range(x - r, x + r):
            for _j in range(y - r, y + r):
                if _i < 0 or _i >= GRID_SIZE or _j < 0 or _j >= GRID_SIZE:
                    continue
                if (_i - x) ** 2 + (_j - y) ** 2 < r ** 2:
                    _grid[_i, _j] += (r ** 2 - (_i - x) ** 2 - (_j - y) ** 2) / r ** 2
        return _grid

    def fill_grid_with_random_obstacles(_grid):
        for i in range(10):
            x = np.random.randint(0, GRID_SIZE)
            y = np.random.randint(0, GRID_SIZE)
            r = np.random.randint(2, 8)
            _grid = draw_circle(_grid, x, y, r)
        return _grid
    
    grid = fill_grid_with_random_obstacles(GRID)

    labeled_grid, num_labels = label(grid)
    print(f"Found {num_labels} obstacles")

    # test if all labels are unique
    assert np.unique(labeled_grid).shape[0] == num_labels + 1

    plt.imshow(labeled_grid)
    
    # draw numbers in the center of each object
    for i in range(1, num_labels + 1):
        x, y = np.where(labeled_grid == i)
        x = np.mean(x)
        y = np.mean(y)
        plt.text(y, x, str(i), color='red', fontsize=12, ha='center', va='center')
    
    plt.show()


if __name__ == '__main__':
    main()
