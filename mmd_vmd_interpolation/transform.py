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
