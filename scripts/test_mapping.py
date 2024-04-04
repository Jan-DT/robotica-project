import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage as ndimage

GRID_SIZE = 250  # cm
FOV = 15  # degrees


def random_offset(spread: float) -> float:
    """
    Returns a random offset f
    rom a normal distribution with spread spread.
    Used to simulate sensor noise.
    :param spread: standard deviation of the normal distribution
    """
    return np.random.normal(0, spread)


def sonar_distance(angle: float) -> float:
    """Returns distance in cm. Simulates a sonar sensor, returns distance in cm."""
    A = 160
    B = 200

    # simulate some obstacles
    if (120 * np.pi / 180) < angle < (130 * np.pi / 180):  # too close to the center
        return 5.0 + random_offset(2)

    if (60 * np.pi / 180) < angle < (85 * np.pi / 180):  # too large
        return 65.0 + random_offset(2)

    if (230 * np.pi / 180) < angle < (233 * np.pi / 180):  # very small
        return 50.0 + random_offset(2)

    if (320 * np.pi / 180) < angle < (325 * np.pi / 180):  # very close to the wall
        return 76.0 + random_offset(2)

    # simulate walls
    return np.sqrt(
        (A / 2 * np.cos(angle)) ** 2 + (B / 2 * np.sin(angle)) ** 2
    ) + random_offset(2) - np.sin(angle) * 10


def filter_for_obstacles(position, _labeled_grid, _num_labels, max_size, min_distance):
    """
    Filters out obstacles that are too large or too close to the robot.
    :param position: Position of the robot, used to calculate distance to obstacles
    :param _labeled_grid: Occupancy grid with labeled obstacles, 0 is free space
    :param _num_labels: Number of labels in the labeled grid
    :param max_size: Maximum size of an obstacle in cm^2. Obstacles larger than this are removed.
    :param min_distance: Minimum distance to an obstacle in cm. Obstacles closer than this are removed.
    :return: Tuple of the filtered labeled grid and a list of obstacle positions.
    """
    x, y = position
    obstacles = []
    _labeled_grid = _labeled_grid.copy()
    for _i in range(_num_labels + 1):
        mask = _labeled_grid == _i
        # remove obstacles that are too large, like walls and other robots
        if np.sum(mask) > max_size:
            _labeled_grid[mask] = 0
            continue

        center = ndimage.center_of_mass(_labeled_grid, labels=_labeled_grid, index=_i)
        # if the obstacle is too close, remove it
        if np.sqrt((center[0] - x) ** 2 + (center[1] - y) ** 2) < min_distance:
            _labeled_grid[mask] = 0
            continue

        # if the obstacle is not too large or too close, add it to the list
        obstacles.append(center)

    return _labeled_grid, obstacles


def plot_separate(grid, filtered_grid, thresholded_grid, labeled_grid, labeled_grid2):
    # clear previous plots
    plt.clf()

    # plot data and save each plot to a separate file
    plt.imshow(grid)
    plt.title("Raw data")
    plt.colorbar()
    plt.scatter(GRID_SIZE // 2, GRID_SIZE // 2, c='r', s=2)
    plt.savefig("raw_data.png")
    plt.clf()

    plt.imshow(filtered_grid)
    plt.title("Filtered data")
    plt.colorbar()
    plt.scatter(GRID_SIZE // 2, GRID_SIZE // 2, c='r', s=2)
    plt.savefig("filtered_data.png")
    plt.clf()

    plt.imshow(thresholded_grid)
    plt.title("Thresholded data")
    plt.colorbar()
    plt.scatter(GRID_SIZE // 2, GRID_SIZE // 2, c='r', s=2)
    plt.savefig("thresholded_data.png")
    plt.clf()

    plt.imshow(labeled_grid)
    plt.title("Labeled data")
    plt.colorbar()
    plt.scatter(GRID_SIZE // 2, GRID_SIZE // 2, c='r', s=2)
    plt.savefig("labeled_data.png")
    plt.clf()

    plt.imshow(labeled_grid2)
    plt.title("Filtered labeled data")
    plt.colorbar()
    plt.scatter(GRID_SIZE // 2, GRID_SIZE // 2, c='r', s=2)
    plt.savefig("filtered_labeled_data.png")
    plt.clf()


def plot_all(grid, filtered_grid, thresholded_grid, labeled_grid, labeled_grid2):
    # clear previous plots
    plt.clf()

    fig, ax = plt.subplots(3, 2)
    ax[0][0].imshow(grid)
    ax[0][1].imshow(filtered_grid)
    ax[1][0].imshow(thresholded_grid)
    ax[1][1].imshow(labeled_grid)
    ax[2][0].imshow(labeled_grid2)
    ax[0][0].set_title("Raw data")
    ax[0][1].set_title("Filtered data")
    ax[1][0].set_title("Thresholded data")
    ax[1][1].set_title("Labeled data")
    ax[2][0].set_title("Filtered labeled data")

    # draw center
    for _i in range(3):
        for _j in range(2):
            if _i == 2 and _j == 1:
                continue
            ax[_i][_j].scatter(GRID_SIZE // 2, GRID_SIZE // 2, c='r', s=2)

    # save total plot
    plt.savefig("total_plot.png")

    # fig.colorbar(ax[0].imshow(grid), ax=ax[0])
    plt.show()


def main():
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    num_rotations = 5
    num_measurements = 50
    uncertainty = 6  # cm
    max_obstacle_size = 150  # cm^2
    min_obstacle_distance = 10  # cm

    def draw_circle(_grid, x, y, r):
        for _i in range(x - r, x + r):
            for _j in range(y - r, y + r):
                if (_i - x) ** 2 + (_j - y) ** 2 < r ** 2:
                    _grid[_i, _j] += (r ** 2 - (_i - x) ** 2 - (_j - y) ** 2) / r ** 2
        return _grid

    for i in range(num_rotations):
        for j in range(num_measurements):
            angle = j * 2 * np.pi / (num_measurements + 1)
            angle += i * 2 * np.pi / (num_rotations + 1)
            distance = sonar_distance(angle)
            x = int(GRID_SIZE / 2 + distance * np.cos(angle))
            y = int(GRID_SIZE / 2 + distance * np.sin(angle))
            grid = draw_circle(grid, x, y, uncertainty)

    filtered_grid = ndimage.rank_filter(grid, rank=1, size=5)
    thresholded_grid = filtered_grid > 0.28
    labeled_grid, num_labels = ndimage.label(thresholded_grid)

    labeled_grid2, obstacles = filter_for_obstacles((GRID_SIZE // 2, GRID_SIZE // 2),
                                                    labeled_grid, num_labels,
                                                    max_obstacle_size,
                                                    min_obstacle_distance)

    print(f"Found {len(obstacles)} obstacles:\n{obstacles}")

    plot_separate(grid, filtered_grid, thresholded_grid, labeled_grid, labeled_grid2)
    plot_all(grid, filtered_grid, thresholded_grid, labeled_grid, labeled_grid2)


if __name__ == "__main__":
    main()
