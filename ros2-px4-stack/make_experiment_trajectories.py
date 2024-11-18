import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from attrs import field, define
from typing import Tuple, List
from scipy.interpolate import interp1d, splprep, splev

from fontTools.ttLib import TTFont
from fontTools.pens.basePen import BasePen

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import pandas as pd

from drone_vars import *


@define
class Path:
    x_vals: np.ndarray = field()
    y_vals: np.ndarray = field()
    z_vals: np.ndarray = field()

    # validate that all values are the same length
    def __attrs_post_init__(self):
        assert (
            len(self.x_vals) == len(self.y_vals) == len(self.z_vals)
        ), "All values must be the same length"

        # make sure that all values are 1D numpy arrays
        assert self.x_vals.ndim == 1, "x_vals must be 1D numpy array"
        assert self.y_vals.ndim == 1, "y_vals must be 1D numpy array"
        assert self.z_vals.ndim == 1, "z_vals must be 1D numpy array"

    # define the add operation to add two paths together by concatenating the values
    def __add__(self, other):
        return Path(
            x_vals=np.concatenate([self.x_vals, other.x_vals]),
            y_vals=np.concatenate([self.y_vals, other.y_vals]),
            z_vals=np.concatenate([self.z_vals, other.z_vals]),
        )

    @property
    def times(self):
        points = np.stack([self.x_vals, self.y_vals, self.z_vals], axis=1)
        point_to_point_diff = np.diff(points, axis=0)
        point_to_point_dist = np.linalg.norm(point_to_point_diff, axis=1)
        total_dist = np.sum(point_to_point_dist)
        times = np.cumsum(np.concatenate([[0], point_to_point_dist])) / total_dist
        times[-1] = 1.0
        return times

    @property
    def length(self):
        return len(self.x_vals)

    @property
    def x_bounds(self):
        return (np.min(self.x_vals), np.max(self.x_vals))

    @property
    def y_bounds(self):
        return (np.min(self.y_vals), np.max(self.y_vals))

    @property
    def z_bounds(self):
        return (np.min(self.z_vals), np.max(self.z_vals))

    def get_window(
        self, start_idx, window_size
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # make a copy of x/y/z that is rolled so that the start_idx is the first index
        x_vals = np.roll(self.x_vals, -start_idx)[:window_size]
        y_vals = np.roll(self.y_vals, -start_idx)[:window_size]
        z_vals = np.roll(self.z_vals, -start_idx)[:window_size]

        return x_vals, y_vals, z_vals

    def get_all_points(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.x_vals, self.y_vals, self.z_vals

    def set_start_idx(self, start_idx) -> None:
        assert start_idx < len(self.x_vals)

        # circular rotation of indices to set a new first index
        self.x_vals = np.roll(self.x_vals, -start_idx)
        self.y_vals = np.roll(self.y_vals, -start_idx)
        self.z_vals = np.roll(self.z_vals, -start_idx)

    def add_points(self, xs, ys, zs) -> None:
        assert (
            len(xs) == len(ys) == len(zs)
        ), f"all values must be the same length: {len(xs)}, {len(ys)}, {len(zs)}"
        self.x_vals = np.concatenate([self.x_vals, xs])
        self.y_vals = np.concatenate([self.y_vals, ys])
        self.z_vals = np.concatenate([self.z_vals, zs])


class FontPen(BasePen):
    def __init__(self, glyph_set):
        super().__init__(glyph_set)
        self.points = []

    def _moveTo(self, p0):
        self.points.append(p0)

    def _lineTo(self, p1):
        self.points.append(p1)

    def _curveToOne(self, p1, p2, p3):
        self.points.append(p3)  # Only saving the final curve point for simplicity.

    def _closePath(self):
        pass


def get_quat_from_yaw(yaw):
    return Quaternion(x=0, y=0, z=np.sin(yaw / 2), w=np.cos(yaw / 2))


@define
class Trajectory:
    x: np.ndarray = field()
    y: np.ndarray = field()
    z: np.ndarray = field()
    times: np.ndarray = field()
    no_turning: bool = field(default=False)

    @property
    def length(self):
        assert len(self.x) == len(self.y) == len(self.z) == len(self.times)
        return len(self.x)

    @property
    def x_bounds(self):
        return (np.min(self.x), np.max(self.x))

    @property
    def y_bounds(self):
        return (np.min(self.y), np.max(self.y))

    def __attrs_post_init__(self):
        assert (
            len(self.x) == len(self.y) == len(self.z) == len(self.times)
        ), f"All values must be the same length: {len(self.x)}, {len(self.y)}, {len(self.z)}, {len(self.times)}"

    def offset(self, dx, dy, dz):
        self.x += dx
        self.y += dy
        self.z += dz

    def scale_x(self, scale):
        self.x *= scale

    def scale_y(self, scale):
        self.y *= scale

    def scale_z(self, scale):
        self.z *= scale

    def circular_shift(self, shift):
        self.x = np.roll(self.x, shift)
        self.y = np.roll(self.y, shift)
        self.z = np.roll(self.z, shift)

    def reverse_order(self):
        self.x = self.x[::-1]
        self.y = self.y[::-1]
        self.z = self.z[::-1]

    def animate(self):
        fig, ax = plt.subplots()
        x_bounds = (np.min(self.x), np.max(self.x))
        y_bounds = (np.min(self.y), np.max(self.y))
        x_range = x_bounds[1] - x_bounds[0]
        y_range = y_bounds[1] - y_bounds[0]

        ax.set_xlim(x_bounds[0] - 0.1 * x_range, x_bounds[1] + 0.1 * x_range)
        ax.set_ylim(y_bounds[0] - 0.1 * y_range, y_bounds[1] + 0.1 * y_range)

        (line,) = ax.plot([], [], lw=2)

        def init():
            line.set_data([], [])
            return (line,)

        def animate(i):
            line.set_data(self.x[: i + 1], self.y[: i + 1])
            return (line,)

        ani = animation.FuncAnimation(
            fig, animate, init_func=init, frames=len(self.x), interval=20, blit=True
        )
        plt.show()

    def get_pose_at_idx(self, idx) -> Pose:
        return Pose(
            position=self.get_point_at_idx(idx),
            orientation=self.get_orientation_at_idx(idx),
        )

    def get_point_at_idx(self, idx) -> Point:
        return Point(x=self.x[idx], y=self.y[idx], z=self.z[idx])

    def get_orientation_at_idx(self, idx) -> Quaternion:
        return get_quat_from_yaw(self.get_heading_at_idx(idx))

    def get_smoothed_heading_at_idx(self, idx):
        # average the heading with the next two headings
        next_idx = idx+1
        next_idx = next_idx % self.length

        next_next_idx = idx+2
        next_next_idx = next_next_idx % self.length


        curr_heading = self.get_heading_at_idx(idx)
        next_heading = self.get_heading_at_idx(next_idx)
        next_next_heading = self.get_heading_at_idx(next_next_idx)

        avg_heading = (curr_heading + next_heading + next_next_heading) / 3
        return avg_heading


    def get_heading_at_idx(self, idx):
        if self.no_turning:
            return np.pi / 2.0

        # get the previous and next points. Take care to handle the edge cases.
        prev_pt_idx = max(0, idx - 1)
        next_pt_idx = min(idx + 1, self.length - 1)

        prev_pt = np.array(
            [self.x[prev_pt_idx], self.y[prev_pt_idx], self.z[prev_pt_idx]]
        )
        next_pt = np.array(
            [self.x[next_pt_idx], self.y[next_pt_idx], self.z[next_pt_idx]]
        )

        # get the direction vector between the previous and next points
        direction = next_pt - prev_pt

        # normalize the direction vector
        direction /= np.linalg.norm(direction)

        # assume that the drone is always level, so we're only interested in the heading
        # the heading is the angle between the direction vector and the x-axis
        heading = np.arctan2(direction[1], direction[0])

        return heading

    @property
    def qx(self):
        return np.array([self.get_orientation_at_idx(i).x for i in range(self.length)])

    @property
    def qy(self):
        return np.array([self.get_orientation_at_idx(i).y for i in range(self.length)])

    @property
    def qz(self):
        return np.array([self.get_orientation_at_idx(i).z for i in range(self.length)])

    @property
    def qw(self):
        return np.array([self.get_orientation_at_idx(i).w for i in range(self.length)])


def get_followable_trajectory_from_path(
    path: Path, time_interval: float, duration: float, turning: bool
) -> Trajectory:
    assert duration > 0, "Duration must be greater than 0"
    times = np.linspace(0, duration, int(duration / time_interval) + 1)

    # consider the path as a series of segments -- straight lines between each point
    # assume that the time to traverse each segment is proportional to the distance between the points
    # and the total time to traverse the path is the sum of the times to traverse each segment

    x, y, z = path.get_all_points()

    # first linearly interpolate the times at half the resolution by taking only every other time point
    interp1d_x_linear = interp1d(path.times * duration, x, kind="linear")
    interp1d_y_linear = interp1d(path.times * duration, y, kind="linear")
    interp1d_z_linear = interp1d(path.times * duration, z, kind="linear")
    x = interp1d_x_linear(times)
    y = interp1d_y_linear(times)
    z = interp1d_z_linear(times)

    use_fancy_interpolation = True
    if not use_fancy_interpolation:
        return Trajectory(x=x, y=y, z=z, times=times, no_turning=not turning)
    else:  # this is a fancier

        # here we will do some tricks to smooth out the trajectory just a bit for the sake of the drone
        half_times = np.linspace(0, duration, int(duration / time_interval / 2) + 1)
        x = interp1d_x_linear(half_times)
        y = interp1d_y_linear(half_times)
        z = interp1d_z_linear(half_times)

        # interpolation options: "linear", "nearest", "zero", "slinear", "quadratic", "cubic"
        interp_type = "quadratic"
        # interp_type = "linear"
        interp1d_x = interp1d(half_times, x, kind=interp_type)
        interp1d_y = interp1d(half_times, y, kind=interp_type)
        interp1d_z = interp1d(half_times, z, kind=interp_type)

        x_vals = interp1d_x(times)
        y_vals = interp1d_y(times)
        z_vals = interp1d_z(times)

        return Trajectory(x=x_vals, y=y_vals, z=z_vals, times=times, no_turning=not turning)


def get_followable_trajectory_from_path_with_spline(
    path: Path, time_interval: float, duration: float, smoothing_factor: float = 1.0
) -> Trajectory:
    """Use spline interpolation to generate a followable trajectory from a path

    Args:
        path (Path): the path to follow
        time_interval (float): the time interval between each point in the trajectory
        duration (float): the total duration of the trajectory
        smoothing_factor (float, optional): the smoothing factor for the spline interpolation. Defaults to 0.5.

    Returns:
        Trajectory: a followable trajectory
    """
    assert duration > 0, "Duration must be greater than 0"

    x, y, z = path.get_all_points()

    # start by linearly interpolating the points at half the resolution
    interp1d_x_linear = interp1d(path.times * duration, x, kind="linear")
    interp1d_y_linear = interp1d(path.times * duration, y, kind="linear")
    interp1d_z_linear = interp1d(path.times * duration, z, kind="linear")
    half_times = np.linspace(0, duration, int(duration / time_interval / 2) + 1)
    x = interp1d_x_linear(half_times)
    y = interp1d_y_linear(half_times)
    z = interp1d_z_linear(half_times)

    # spline interpolation
    tck, u = splprep([x, y, z], s=smoothing_factor)

    # get evenly spaced points along the spline
    u_fine = np.linspace(0, 1, int(duration / time_interval) + 1)
    x_vals, y_vals, z_vals = splev(u_fine, tck)

    times = np.linspace(0, duration, int(duration / time_interval) + 1)

    return Trajectory(x=x_vals, y=y_vals, z=z_vals, times=times)


def get_glyph_points(font, letter):
    # Access glyph for the letter
    glyph_set = font.getGlyphSet()
    glyph = glyph_set[font.getBestCmap().get(ord(letter), ".notdef")]

    pen = FontPen(glyph_set)
    glyph.draw(pen)
    return np.array(pen.points)


def get_letter_path(
    letter, z_val, font_path="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
) -> Path:
    # Load the font file
    font = TTFont(font_path)

    # Get points for the letter
    points = get_glyph_points(font, letter)

    if points.size == 0:
        print(f"Letter '{letter}' not found in the font.")
        return Path(x_vals=np.array([]), y_vals=np.array([]), z_vals=np.array([]))

    # Get the x and y values
    x, y = points[:, 0], points[:, 1]

    # Make the letter wrap around by adding the first point to the end
    x = np.append(x, x[0])
    y = np.append(y, y[0])

    z_vals = np.ones_like(x) * z_val
    return Path(x_vals=x, y_vals=y, z_vals=z_vals)


def plot_path(path):
    x, y, z = path.get_all_points()

    plt.figure(figsize=(5, 5))
    plt.plot(x, y, "o-", markersize=2)
    # plt.scatter(x, y, s=2)
    plt.show()


def animate_multiple_trajectories(trajs: List[Trajectory]):
    fig, ax = plt.subplots()

    # make the viewer square
    ax.set_aspect("equal")

    front_left = HIGHBAY_FRONT_LEFT
    back_right = HIGHBAY_BACK_RIGHT
    x_min, y_min = front_left
    x_max, y_max = back_right
    x_bounds = (x_min, x_max)
    y_bounds = (y_min, y_max)
    x_range = x_bounds[1] - x_bounds[0]
    y_range = y_bounds[1] - y_bounds[0]

    ax.set_xlim(x_bounds[0] - 0.1 * x_range, x_bounds[1] + 0.1 * x_range)
    ax.set_ylim(y_bounds[0] - 0.1 * y_range, y_bounds[1] + 0.1 * y_range)

    # lines for each trajectory
    lines = [ax.plot([], [], lw=2)[0] for _ in trajs]

    # circles to represent the diameter of each vehicle
    circ_radius = 0.5
    circles = [
        ax.add_patch(
            plt.Circle(
                xy=(traj.x[0], traj.y[0]),
                radius=circ_radius,
                fill=False,
                linestyle="--",
                linewidth=2,
                color="black",
            )
        )
        for traj in trajs
    ]

    # arrows to represent the heading of each vehicle
    arrow_length = 0.5
    arrows = [
        ax.arrow(
            x=traj.x[0],
            y=traj.y[0],
            dx=np.cos(traj.get_heading_at_idx(0))*arrow_length,
            dy=np.sin(traj.get_heading_at_idx(0))*arrow_length,
            head_width=0.1,
            head_length=0.1,
            fc="k",
            ec="k",
        )
        for traj in trajs
    ]

    # draw boundaries at x_min, x_max, y_min, y_max
    ax.hlines(y_min, x_min, x_max, color="black", linestyle="--")
    ax.hlines(y_max, x_min, x_max, color="black", linestyle="--")
    ax.vlines(x_min, y_min, y_max, color="black", linestyle="--")
    ax.vlines(x_max, y_min, y_max, color="black", linestyle="--")

    # if mocap mode is optitrack_full, then draw the curtain
    if MOCAP_MODE == "optitrack_full":
        back_left_corner = OPTITRACK_WEST_BACK_RIGHT
        front_right_corner = OPTITRACK_EAST_FRONT_LEFT
        min_x = back_left_corner[0]
        max_x = front_right_corner[0]
        min_y = back_left_corner[1]
        max_y = front_right_corner[1]
        ax.hlines(min_y, min_x, max_x, color="black", linestyle="--")
        ax.hlines(max_y, min_x, max_x, color="black", linestyle="--")
        ax.vlines(min_x, min_y, max_y, color="black", linestyle="--")
        ax.vlines(max_x, min_y, max_y, color="black", linestyle="--")


    def init():
        for line in lines:
            line.set_data([], [])
        return lines + circles + arrows

    def animate(i):
        for line, traj, circle, arrow in zip(lines, trajs, circles, arrows):
            # line
            line.set_data(traj.x[: i + 1], traj.y[: i + 1])

            # circle
            circle.center = (traj.x[i], traj.y[i])

            # arrow
            # heading = traj.get_smoothed_heading_at_idx(i)
            heading = traj.get_heading_at_idx(i)
            arrow.set_data(
                x=traj.x[i],
                y=traj.y[i],
                dx=np.cos(heading)*arrow_length,
                dy=np.sin(heading)*arrow_length,
            )

        # set all circles to black
        for circle in circles:
            circle.set_edgecolor("black")

        # if the circles intersect, then we should color them red
        for circ_i_idx in range(len(circles)):
            circ_i = circles[circ_i_idx]
            circ_i_center = np.array(circ_i.center)
            for circ_j_idx in range(circ_i_idx + 1, len(circles)):
                circ_j = circles[circ_j_idx]
                circ_j_center = np.array(circ_j.center)
                circ_center_dist = np.linalg.norm(circ_i_center - circ_j_center)
                if circ_center_dist < 2 * circ_radius:
                    print(
                        f"Collision detected between {circ_i_idx} and {circ_j_idx} at {i}"
                    )
                    circ_i.set_edgecolor("red")
                    circ_j.set_edgecolor("red")

        return lines + circles + arrows

    ani = animation.FuncAnimation(
        fig, animate, init_func=init, frames=len(trajs[0].x), interval=20, blit=True
    )
    plt.show()


def fit_trajectories_to_space(
    trajs: List[Trajectory], space_bottom_left_xy, space_top_right_xy
) -> List[Trajectory]:
    # check that all of the trajs have the same time points
    assert all(
        [np.allclose(traj.times, trajs[0].times) for traj in trajs]
    ), "All trajectories must have the same time points"

    trajs_x_min = np.min([np.min(traj.x) for traj in trajs])
    trajs_x_max = np.max([np.max(traj.x) for traj in trajs])
    trajs_y_min = np.min([np.min(traj.y) for traj in trajs])
    trajs_y_max = np.max([np.max(traj.y) for traj in trajs])

    space_center = (space_bottom_left_xy + space_top_right_xy) / 2
    space_x_width = space_top_right_xy[0] - space_bottom_left_xy[0]
    space_y_width = space_top_right_xy[1] - space_bottom_left_xy[1]

    # rescale the trajectories to be 90% of the space
    necessary_x_scale = space_x_width / (trajs_x_max - trajs_x_min)
    necessary_y_scale = space_y_width / (trajs_y_max - trajs_y_min)
    # necessary_x_scale *= 0.9
    # necessary_y_scale *= 0.9
    necessary_scale = min(necessary_x_scale, necessary_y_scale) * 0.9
    for traj in trajs:
        # traj.scale_x(necessary_x_scale)
        # traj.scale_y(necessary_y_scale)
        traj.scale_x(necessary_scale)
        traj.scale_y(necessary_scale)

    # offset trajectories to be centered in the space
    new_traj_x_max = np.max([np.max(traj.x) for traj in trajs])
    new_traj_x_min = np.min([np.min(traj.x) for traj in trajs])
    new_traj_y_max = np.max([np.max(traj.y) for traj in trajs])
    new_traj_y_min = np.min([np.min(traj.y) for traj in trajs])
    new_traj_center = (new_traj_x_max + new_traj_x_min) / 2, (
        new_traj_y_max + new_traj_y_min
    ) / 2
    necessary_offset = space_center - new_traj_center
    for traj in trajs:
        traj.offset(necessary_offset[0], necessary_offset[1], 0)

    return trajs


if __name__ == "__main__":
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    traj_update_interval = 0.2
    traj_duration = 30.0

    # these values are set in drone_vars.py
    space_front_left_xy = HIGHBAY_FRONT_LEFT
    space_back_right_xy = HIGHBAY_BACK_RIGHT

    # back curtain 3.37 -- front curtain -3.73
    space_front_left_xy[1] = max(space_front_left_xy[1], -3.73)
    space_back_right_xy[1] = min(space_back_right_xy[1], 3.37)

    # to be safe, let's reduce the front-left and back-right coordinates by a small amount (padding)
    space_front_left_xy += 1.0
    space_back_right_xy -= 1.0

    # space_front_left_xy += 0.5
    # space_back_right_xy -= 0.5

    # move the back right a bit further down
    # space_back_right_xy[0] -= 1.0

    # increase the min y (front left) and decrease the max y (back right)
    # space_front_left_xy[1] += 1.0
    # space_back_right_xy[1] -= 1.0

    # what's the current x-width and y-width of the space?
    space_x_width = space_back_right_xy[0] - space_front_left_xy[0]
    space_y_width = space_back_right_xy[1] - space_front_left_xy[1]
    print(f"Space x-width: {space_x_width}, Space y-width: {space_y_width}")

    letters = "MMM"
    letters = "MIT"
    # letters = "MT"
    letters = "ACL"
    # letters = "MRG"
    heights = [2.75, 2.35, 2.0]

    letter_paths = [
        get_letter_path(l, h, font_path) for (l, h) in zip(letters, heights)
    ]
    trajs = [
        get_followable_trajectory_from_path(
        # get_followable_trajectory_from_path_with_spline(
            path, time_interval=traj_update_interval, duration=traj_duration, turning=False
        )
        for path in letter_paths
    ]

    # trajs[1].reverse_order()

    if letters == "MIT":
        # circular shift the third trajectory by 1/2 of its length to give vehicles more clearance
        trajs[2].circular_shift(int(trajs[2].length / 2))

    # offset the trajectories so that they are not overlapping
    # We will space by the width of the letter "I"
    cap_i_path = get_letter_path("I", 0, font_path)
    cap_i_x_width = cap_i_path.x_bounds[1] - cap_i_path.x_bounds[0]
    cap_i_y_width = cap_i_path.y_bounds[1] - cap_i_path.y_bounds[0]
    for i in range(1, len(trajs)):
        max_x_prev = np.max(trajs[i - 1].x)
        y_center_prev = (np.min(trajs[i - 1].y) + np.max(trajs[i - 1].y)) / 2
        # trajs[i].offset(max_x_prev + 3 * cap_i_x_width, y_center_prev, 0) # offset in y direction
        # if i == 1:
            # trajs[i].scale_x(2)
        trajs[i].offset(max_x_prev + 6 * cap_i_x_width, 0, 0) # no y offset
        # elif i == 2:
        #     trajs[i].offset(max_x_prev + 4 * cap_i_x_width, 0, 0)
        # else:
        #     raise ValueError("Invalid index")

    trajs = fit_trajectories_to_space(trajs, space_front_left_xy, space_back_right_xy)
    animate_multiple_trajectories(trajs)

    # check that the trajectories are all the same length
    assert all(
        [traj.length == trajs[0].length for traj in trajs]
    ), f"All trajectories must be the same length. Received lengths: {[traj.length for traj in trajs]}"

    # check that the time points are all the same
    assert all(
        [np.allclose(traj.times, trajs[0].times) for traj in trajs]
    ), f"All trajectories must have the same time points. Received time points: {[traj.times for traj in trajs]}"

    # write the trajectories to a data frame
    traj_df = pd.DataFrame({"time": trajs[0].times})
    for ns, traj in zip(ROS_NAMESPACES, trajs):
        traj_df[f"{ns}_x"] = traj.x
        traj_df[f"{ns}_y"] = traj.y
        traj_df[f"{ns}_z"] = traj.z
        traj_df[f"{ns}_qx"] = traj.qx
        traj_df[f"{ns}_qy"] = traj.qy
        traj_df[f"{ns}_qz"] = traj.qz
        traj_df[f"{ns}_qw"] = traj.qw

    # save the trajectories to a csv file
    traj_df.to_csv(TRAJ_FPATH, index=False)
    print(f"Saved trajectories to {TRAJ_FPATH}")

    # print the minimum and maximum x/y values for each trajectory
    for ns, traj in zip(ROS_NAMESPACES, trajs):
        print(
            f"{ns} x bounds: {traj.x_bounds}, y bounds: {traj.y_bounds}, x width: {traj.x_bounds[1] - traj.x_bounds[0]}, y width: {traj.y_bounds[1] - traj.y_bounds[0]}"
        )
