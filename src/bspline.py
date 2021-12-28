import numpy as np
import scipy.interpolate as scipy_interpolate


def approximate_b_spline_path(x: list, y: list, n_path_points: int,
                              degree: int = 3) -> tuple:
    """
    approximate points with a B-Spline path
    :param x: x position list of approximated points
    :param y: y position list of approximated points
    :param n_path_points: number of path points
    :param degree: (Optional) B Spline curve degree
    :return: x and y position list of the result path
    """
    t = range(len(x))
    x_tup = scipy_interpolate.splrep(t, x, k=degree)
    y_tup = scipy_interpolate.splrep(t, y, k=degree)

    x_list = list(x_tup)
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    rx = scipy_interpolate.splev(ipl_t, x_list)
    ry = scipy_interpolate.splev(ipl_t, y_list)

    return rx, ry


def interpolate_b_spline_path(x: list, y: list, n_path_points: int,
                              degree: int = 3) -> tuple:
    """
    interpolate points with a B-Spline path
    :param x: x positions of interpolated points
    :param y: y positions of interpolated points
    :param n_path_points: number of path points
    :param degree: B-Spline degree
    :return: x and y position list of the result path
    """
    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)

    travel = np.linspace(0.0, len(x) - 1, n_path_points)
    return spl_i_x(travel), spl_i_y(travel)