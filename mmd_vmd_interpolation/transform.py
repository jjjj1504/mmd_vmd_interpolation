import numpy as np


class Transform(object):

    @staticmethod
    def product_quaternion(ql, qr):
        qlx, qly, qlz, qlw = ql
        qrx, qry, qrz, qrw = qr
        q = np.stack([
            qlx*qrw + qlw*qrx + qly*qrz - qlz*qry,
            qly*qrw + qlw*qry + qlz*qrx - qlx*qrz,
            qlz*qrw + qlw*qrz + qlx*qry - qly*qrx,
           -qlx*qrx - qly*qry - qlz*qrz + qlw*qrw,
        ])
        return q

    @staticmethod
    def inverse_quaternion(q):
        q_inv = np.empty_like(q)
        q_inv[:3] = -q[:3]
        q_inv[3] = q[3]
        return q_inv

    @classmethod
    def divide_left_quaternion(cls, ql, qr):
        ql_inv = cls.inverse_quaternion(ql)
        q = cls.product_quaternion(ql_inv, qr)
        return q

    @staticmethod
    def decompose_quaternion(q):
        costh2 = q[3]
        sinth2 = np.sqrt(q[0]**2 + q[1]**2 + q[2]**2)
        angle = 2 * np.arctan2(sinth2, costh2)
        if sinth2 > 0.:
            axis = q[:3] / sinth2
        else:
            axis = np.zeros_like(q[:3], dtype="float")
        return axis, angle

    @staticmethod
    def form_quaternion(axis, angle):
        costh2 = np.cos(angle/2.)
        sinth2 = np.sin(angle/2.)
        quaternion = np.stack([sinth2*axis[0], sinth2*axis[1], sinth2*axis[2], costh2])
        return quaternion

    @classmethod
    def transform_pose(cls, q1, t1, q2, t2):
        q12 = cls.product_quaternion(q1, q2)
        t12 = cls.rotate_vector(q1, t2) + t1
        return q12, t12

    @classmethod
    def rotate_vector(cls, q, v):
        v4 = cls.extend_vector_to_quaternion(v)
        qi = cls.inverse_quaternion(q)
        qv4 = cls.product_quaternion(q, v4)
        qv4qi = cls.product_quaternion(qv4, qi)
        return qv4qi[:3]

    @staticmethod
    def extend_vector_to_quaternion(v):
        zero_append = np.zeros_like(v[0])
        v4 = np.stack([v[0], v[1], v[2], zero_append])
        return v4

    @classmethod
    def convert_mmd_euler_angles_to_quaternion(cls, mmd_euler_angles):
        mmd_x_pitch, mmd_y_yaw, mmd_z_roll = mmd_euler_angles
        q_x_pitch = cls.form_quaternion(np.array([[1.,  0.,  0.]]).T, mmd_x_pitch)
        q_y_yaw   = cls.form_quaternion(np.array([[0., -1.,  0.]]).T, mmd_y_yaw)   # attention: in reverse
        q_z_roll  = cls.form_quaternion(np.array([[0.,  0., -1.]]).T, mmd_z_roll)   # attention: in reverse
        q_yp = cls.product_quaternion(q_y_yaw, q_x_pitch)
        q_ypr = cls.product_quaternion(q_yp, q_z_roll)
        return q_ypr
