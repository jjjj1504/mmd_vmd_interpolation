import sys

import numpy as np

from transform import Transform


class MMDCurveInterp(object):

    @classmethod
    def interp(cls, frame_id_endpoint, value_endpoint, curve_param, frame_ids_desired):
        # needn't do interpolation for flat data
        if (value_endpoint[1] == value_endpoint[0]).all():
            if value_endpoint.ndim == 1:
                return np.full(len(frame_ids_desired), value_endpoint[0])
            else:
                return np.tile(value_endpoint[0,:], [len(frame_ids_desired), 1])
        # get 4 control points from mmd curve parameters (4-by-2)
        control_points = cls._get_bezier_curve_control_points(curve_param)
        # get the coefficints of time polynomial of x, y on cubic bezier curve by give 4 control points
        cubic_bezier_coeffs = cls._get_cubic_bezier_coeffs(control_points)
        # map frame index to x data with range 0 ~ 1
        x = (frame_ids_desired - frame_id_endpoint[0]) / float(frame_id_endpoint[1] - frame_id_endpoint[0])
        # solve y on cubic bezier curve from given x
        y = cls._solve_cubic_bezier_y_from_x(cubic_bezier_coeffs, x)
        # map y with range 0 ~1 to value space
        if value_endpoint.ndim == 1:
            values = value_endpoint[0] + y*(value_endpoint[1] - value_endpoint[0])
        else:
            values = value_endpoint[0,:] + y.reshape(-1,1)*(value_endpoint[1,:] - value_endpoint[0,:])
        return values

    @classmethod
    def interp_position(cls, frame_id_endpoint, position_endpoint, curve_param_xyz, frame_ids_desired):
        position = np.zeros([len(frame_ids_desired), 3])
        for i in range(3):
            position[:,i] = cls.interp(frame_id_endpoint, position_endpoint[:,i], curve_param_xyz[i], frame_ids_desired)
        return position

    @classmethod
    def interp_quaternion(cls, frame_id_endpoint, quaternion_endpoint, curve_param, frame_ids_desired):
        if (quaternion_endpoint[0] == quaternion_endpoint[1]).all():
            return np.tile(quaternion_endpoint[0,:], [len(frame_ids_desired), 1])
        q_diff = Transform.divide_left_quaternion(quaternion_endpoint[0], quaternion_endpoint[1])
        axis, angle = Transform.decompose_quaternion(q_diff)
        angles = cls.interp(frame_id_endpoint, np.array([0., angle]), curve_param, frame_ids_desired)
        quaternions_diff_interp = Transform.form_quaternion(axis.reshape(3,1), angles).T
        quaternions = Transform.product_quaternion(quaternion_endpoint[0], quaternions_diff_interp.T).T
        return quaternions

    @staticmethod
    def _get_bezier_curve_control_points(curve_param):
        control_point0 = np.array([0., 0.])
        control_point1 = curve_param[0:2] / 127.0
        control_point2 = curve_param[2:4] / 127.0
        control_point3 = np.array([1., 1.])
        control_points = np.row_stack([control_point0, control_point1, control_point2, control_point3])
        return control_points

    @staticmethod
    def _get_cubic_bezier_coeffs(control_points):
        # get 4 control points from mmd curve parameters (2-by-4)
        # each row is 3rd order polynomial coefficients of time for x, y curve
        cubic_bezier_coeffs = \
            control_points[[0],:].T * [-1,  3, -3, 1] + \
            control_points[[1],:].T * [ 3, -6,  3, 0] + \
            control_points[[2],:].T * [-3,  3,  0, 0] + \
            control_points[[3],:].T * [ 1,  0,  0, 0]   # shape = (2,4)
        return cubic_bezier_coeffs

    @classmethod
    def _solve_cubic_bezier_y_from_x(cls, cubic_bezier_coeffs, x):
        coeffs_to_solve = cubic_bezier_coeffs[0, :] - np.column_stack([np.zeros([len(x),3]), x])
        t_sol = cls._solve_cubic_equation_real_root(coeffs_to_solve)
        y = (t_sol.reshape(-1,1) ** range(4)[::-1]).dot(cubic_bezier_coeffs[1,:])
        return y

    @staticmethod
    def _solve_cubic_equation_real_root(coeffs):
        a = coeffs[:, 0]
        b = coeffs[:, 1]
        c = coeffs[:, 2]
        d = coeffs[:, 3]
        ca = c/a
        b3a = b/(3*a)
        p = ca - 3*(b3a**2)
        q = d/a + 2*(b3a**3) - ca*b3a
        delta = q**2 + (4/27.0)*(p**3)
        # case discussion
        y = np.zeros_like(delta, dtype="float")
        mask = delta >= 0
        # case for single real root
        delta_sqrt_mask = np.sqrt(delta[mask])
        m_mask = np.cbrt(0.5*(-q[mask] + delta_sqrt_mask))
        n_mask = np.cbrt(0.5*(-q[mask] - delta_sqrt_mask))
        y[mask] = m_mask + n_mask
        # case for 3 real roots
        th0_imask = np.arctan2(np.sqrt(-delta[~mask]), -q[~mask])
        y[~mask] = 2 * np.sqrt(-p[~mask]/3.0) * np.cos((th0_imask+4*np.pi)/3.0)
        # final
        sols = y - b3a
        if ((sols > 1.0 + sys.float_info.epsilon) | (sols < 0.0 - sys.float_info.epsilon)).any():
            print("hello")
        return sols
