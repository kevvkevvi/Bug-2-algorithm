"""
COMS 4733: Computational Aspects of Robotics
HW1 Problem 5.2
Bug Algorithm
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import math


def plot(start: tuple, end: tuple, c_obs: list, movements: list) -> None:
    c_obs.append(c_obs[0])
    x, y = np.array(c_obs).T
    plt.scatter(x, y)
    plt.plot(x, y)
    plt.fill_between(x, y)
    x_values = [start[0], end[0]]
    y_values = [start[1], end[1]]
    plt.scatter(x_values, y_values)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.annotate("Start", (x_values[0], y_values[0]))
    plt.annotate("Goal", (x_values[1], y_values[1]))
    plt.plot(x_values, y_values, "--", linewidth=6)
    x_moves = []
    y_moves = []
    for a, b in movements:
        x_moves.append(a)
        y_moves.append(b)
    plt.scatter(x_moves, y_moves, zorder=5)
    plt.plot(x_moves, y_moves)
    plt.show()


def hit_m_line(movements: list, m_line: Path) -> bool:
    last_move = Path([movements[-2], movements[-1]])
    if last_move.intersects_path(m_line):
        return True
    return False


def bug(start: tuple, end: tuple, c_obs: list, d: float, delt: float) -> list:
    m_line = Path([start, end])
    movements = [list(start)]
    obs = Path(c_obs)
    if start[0] < end[0]:
        sign = 1
    if start[0] > end[0]:
        sign = -1
    slope = float(end[1] - start[1]) / float(end[0] - start[0])
    starting_angle = math.atan(slope)
    current_angle = starting_angle
    next_move = (
        movements[-1][0] + sign * d * np.cos(starting_angle),
        movements[-1][1] + sign * d * np.sin(starting_angle),
    )
    # move toward goal before the obstacle
    while not obs.contains_point(next_move):
        if start[0] < end[0]:
            if movements[-1][0] > end[0]:
                movements.pop()
                movements.append(end)
                plot(start, end, c_obs, movements)
                print(movements)
                return movements
        if start[0] > end[0]:
            if movements[-1][0] < end[0]:
                movements.pop()
                movements.append(end)
                plot(start, end, c_obs, movements)
                print(movements)
                return movements
        movements.append(list(next_move))
        next_move = (
            movements[-1][0] + sign * d * np.cos(current_angle),
            movements[-1][1] + sign * d * np.sin(current_angle),
        )
    # hit obstacle, turn and get out of the m-line's way
    while obs.contains_point(next_move):
        next_move = (
            movements[-1][0] + sign * d * np.cos(current_angle + delt),
            movements[-1][1] + sign * d * np.sin(current_angle + delt),
        )
        current_angle = current_angle + delt
    movements.append(list(next_move))
    next_move = (
        movements[-1][0] + sign * d * np.cos(current_angle),
        movements[-1][1] + sign * d * np.sin(current_angle),
    )
    while obs.contains_point(next_move):
        next_move = (
            movements[-1][0] + sign * d * np.cos(current_angle + delt),
            movements[-1][1] + sign * d * np.sin(current_angle + delt),
        )
        current_angle = current_angle + delt
    movements.append(list(next_move))
    # follow the boundary
    while not hit_m_line(movements, m_line):
        next_move = (
            movements[-1][0] + sign * d * np.cos(current_angle - delt),
            movements[-1][1] + sign * d * np.sin(current_angle - delt),
        )
        if obs.contains_point(next_move):
            next_move = (
                movements[-1][0] + sign * d * np.cos(current_angle),
                movements[-1][1] + sign * d * np.sin(current_angle),
            )

        else:
            prev_angle = current_angle + delt
            current_angle = current_angle - delt
            while not obs.contains_point(next_move):
                next_move = (
                    movements[-1][0] + sign * d * np.cos(current_angle - delt),
                    movements[-1][1] + sign * d * np.sin(current_angle - delt),
                )
                current_angle = current_angle - delt
            next_move = (
                movements[-1][0] + sign * d * np.cos(current_angle + delt),
                movements[-1][1] + sign * d * np.sin(current_angle + delt),
            )
            current_angle = current_angle + delt
        while obs.contains_point(next_move):
            next_move = (
                movements[-1][0] + sign * d * np.cos(current_angle + delt),
                movements[-1][1] + sign * d * np.sin(current_angle + delt),
            )
            current_angle = current_angle + delt
        new_path = Path([movements[-1], list(next_move)])
        if obs.intersects_path(new_path):
            next_move = (
                movements[-1][0] + sign * d * np.cos(prev_angle),
                movements[-1][1] + sign * d * np.sin(prev_angle),
            )
        movements.append(list(next_move))
    # clear sight to the goal --> delete the last move,
    # turn towards the direction of the m-line, then Go go go!
    movements.pop()
    last_obs_point = movements[-1]
    goal_slope = float(end[1] - last_obs_point[1]) / float(end[0] - last_obs_point[0])
    goal_angle = math.atan(goal_slope)
    turns = (goal_angle - current_angle) / delt
    next_move = (
        movements[-1][0] + sign * d * np.cos(current_angle + turns * delt),
        movements[-1][1] + sign * d * np.sin(current_angle + turns * delt),
    )
    if start[0] < end[0]:
        while movements[-1][0] < end[0]:
            movements.append(list(next_move))
            next_move = (
                movements[-1][0] + sign * d * np.cos(current_angle + turns * delt),
                movements[-1][1] + sign * d * np.sin(current_angle + turns * delt),
            )
    if start[0] > end[0]:
        while movements[-1][0] > end[0]:
            movements.append(list(next_move))
            next_move = (
                movements[-1][0] + sign * d * np.cos(current_angle + turns * delt),
                movements[-1][1] + sign * d * np.sin(current_angle + turns * delt),
            )
    movements.pop()
    movements.append(end)
    plot(start, end, c_obs, movements)
    print(movements)
    return movements


if __name__ == "__main__":
    start_1 = [1, 1]
    end_1 = [10, 19]
    c_obs_1 = [[6, 18], [3, 3], [9, 3]]
    d_1 = 0.5
    delt_1 = np.radians(5)
    bug(start_1, end_1, c_obs_1, d_1, delt_1)

    start_2 = [10, 1]
    end_2 = [1, 19]
    c_obs_2 = [[4, 12], [4, 6], [8, 6], [8, 12]]
    d_2 = 0.5
    delt_2 = np.radians(5)
    bug(start_2, end_2, c_obs_2, d_2, delt_2)

    start_3 = [2, 3]
    end_3 = [11, 3]
    c_obs_3 = [[4, 12], [4, 6], [8, 6], [8, 12]]
    d_3 = 0.5
    delt_3 = np.radians(5)
    bug(start_3, end_3, c_obs_3, d_3, delt_3)
