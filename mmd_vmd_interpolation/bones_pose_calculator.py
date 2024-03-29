import numpy as np
import scipy.signal

from .mmd_curve_interp import MMDCurveInterp
from .transform import Transform
from .vmd_profile import VmdBoneData


class BonesTree(object):

    @classmethod
    def get(cls, bones_list):
        # type: (list[tuple[str, str, np.ndarray]]) -> dict[str, dict[str, str | np.ndarray]]
        bones_tree = {}  # type: dict[str, dict[str, str | np.ndarray]]
        # assign basic attribute
        for bone_name, parent_name, position in bones_list:
            bones_tree[bone_name] = {
                "parent": parent_name,
                "position": position,
                "trans_from_parent": None,
            }
        # get translation from parent bone
        for bone_name, bone_info in bones_tree.items():
            if bone_info["parent"] is not None:
                bone_info["trans_from_parent"] = (bone_info["position"]
                    - bones_tree[bone_info["parent"]]["position"])
        return bones_tree


class BonesPoseCalculator(object):

    def __init__(self, bones_data, bone_tree={}):
        # type: (dict[str, VmdBoneData], dict[str, dict[str, str | np.ndarray]]) -> None
        self._bones_data = bones_data  # type: dict[str, VmdBoneData]
        self._bones_tree = bone_tree
        self._full_frame_num = max([
            b.frame_ids[-1] if b.get_frame_num() else 0 \
                for b in self._bones_data.values()
        ])
        self._bones_full_interp = {}  # type: dict[str, VmdBoneData]
        self._bones_full_pose = {}  # type: dict[str, VmdBoneData]
        self._bones_full_position_lpf = {}  # type: dict[str, np.ndarray]

    def get_full_interp_bones(self):
        if self._bones_full_interp:
            return self._bones_full_interp
        # loop to do data interpolation for each bone
        for name in self._bones_data.keys():
            self._get_full_interp_bone(name)
        return self._bones_full_interp

    def _get_full_interp_bone(self, bone_name):
        # type: (str) -> VmdBoneData
        if bone_name in self._bones_full_interp:
            return self._bones_full_interp[bone_name]
        bone_data = self._bones_data[bone_name]
        bone_full_interp = VmdBoneData(bone_name, self._full_frame_num)
        bone_full_interp.frame_ids = np.arange(self._full_frame_num)
        # loop to do data interpolation for each interval
        for i in range(bone_data.get_frame_num()-1):
            frame_id_endpoint = bone_data.frame_ids[i:i+2]
            fid0, fid1 = frame_id_endpoint
            # do data interpolation in frame fid0 ~ fid1-1
            frame_ids_desired = np.arange(fid0, fid1)
            position_interp = MMDCurveInterp.interp_position(
                frame_id_endpoint,
                bone_data.positions[i:i+2, :],
                [bone_data.curve_x[i+1,:], bone_data.curve_y[i+1,:], bone_data.curve_z[i+1,:]],
                frame_ids_desired,
            )
            orientation_interp = MMDCurveInterp.interp_quaternion(
                frame_id_endpoint,
                bone_data.orientations[i:i+2, :],
                bone_data.curve_rot[i+1,:],
                frame_ids_desired,
            )
            # record interval data
            bone_full_interp.positions[fid0:fid1, :] = position_interp
            bone_full_interp.orientations[fid0:fid1, :] = orientation_interp
        # append the last frame
        if bone_data.get_frame_num() > 1:
            bone_full_interp.positions[-1, :] = bone_data.positions[-1, :]
            bone_full_interp.orientations[-1, :] = bone_data.orientations[-1, :]
        # padding constant data for single frame
        elif bone_data.get_frame_num() == 1:
            bone_full_interp.positions[:] = bone_data.positions[0, :]
            bone_full_interp.orientations[:] = bone_data.orientations[0, :]
        # remain default value if 0 frame
        else:
            pass
        # record interpolated bone data
        self._bones_full_interp[bone_name] = bone_full_interp
        return self._bones_full_interp[bone_name]

    def get_full_pose_bones(self):
        if self._bones_full_pose:
            return self._bones_full_pose
        # loop to do data interpolation for each bone
        for name in self._bones_data.keys():
            self._get_full_pose_bone(name)
        return self._bones_full_pose

    def _get_full_pose_bone(self, bone_name):
        # type: (str) -> VmdBoneData
        # get parent bone
        if bone_name in self._bones_full_pose:
            return self._bones_full_pose[bone_name]
        # get parent bone
        parent_name = self._bones_tree[bone_name]["parent"]
        if parent_name is not None:
            parent_full_pose = self._get_full_pose_bone(parent_name)
        else:
            # if no parent, just return itself
            self._bones_full_pose[bone_name] = self._get_full_interp_bone(bone_name)
            return self._bones_full_pose[bone_name]
        # get relative translation
        trans_from_parent = self._bones_tree[bone_name]["trans_from_parent"]
        # get its bone data
        bone_full_interp = self._get_full_interp_bone(bone_name)
        # successsive transformation
        full_frame_num = bone_full_interp.get_frame_num()
        bone_full_pose = VmdBoneData(bone_name, full_frame_num)
        bone_full_pose.frame_ids = bone_full_interp.frame_ids
        full_orientations_T, full_positions_T = Transform.transform_pose(
            parent_full_pose.orientations.T,
            parent_full_pose.positions.T,
            bone_full_interp.orientations.T,
            (bone_full_interp.positions + trans_from_parent).T,
        )
        bone_full_pose.orientations = full_orientations_T.T
        bone_full_pose.positions = full_positions_T.T
        # record
        self._bones_full_pose[bone_name] = bone_full_pose
        return self._bones_full_pose[bone_name]

    def get_lpf_full_positions_bones(self, time_delay):
        # type: (float) -> dict[str, np.ndarray]
        if self._bones_full_position_lpf:
            return self._bones_full_position_lpf
        # loop to apply low-pass filter to position for each bone
        for bone_name in self.get_full_pose_bones().keys():
            self._get_lpf_full_positions_bone(bone_name, time_delay)
        return self._bones_full_position_lpf

    def _get_lpf_full_positions_bone(self, bone_name, time_delay):
        # type: (str, float) -> np.ndarray
        if bone_name in self._bones_full_position_lpf:
            return self._bones_full_position_lpf[bone_name]
        bone_full_pose = self._get_full_pose_bone(bone_name)
        bone_lpf = VmdBoneData(bone_name, bone_full_pose.get_frame_num())
        bone_lpf.frame_ids = bone_full_pose.frame_ids
        bone_lpf.positions = self._apply_lpf(bone_full_pose.positions, time_delay)
        self._bones_full_position_lpf[bone_name] = bone_lpf
        return self._bones_full_position_lpf[bone_name]

    @staticmethod
    def _apply_lpf(x, time_delay):
        # type: (np.ndarray, float) -> np.ndarray
        if time_delay == 0:
            return x
        else:
            def lpf_by_for_loop(x, lag_ratio, update_ratio):
                # type: (np.ndarray, float, float) -> np.ndarray
                y = np.empty_like(x)  # type: np.ndarray
                y[0] = x[0]
                for i in range(1, len(x)):
                    # 1-st order low-pass filter with backward difference approach
                    y[i] = lag_ratio * y[i-1] + update_ratio * x[i]
                return y

            def lpf_by_scipy_lfilter(x, lag_ratio, update_ratio):
                # type: (np.ndarray, float, float) -> np.ndarray
                # apply digital filter
                b = np.array([update_ratio])
                a = np.array([1.0, -lag_ratio])
                zi = scipy.signal.lfilter_zi(b, a)    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.lfilter_zi.html
                y, _ = scipy.signal.lfilter(b, a, x, axis=0, zi=zi*np.array([x[0]]))
                return y

            # low pass filter constant
            dt = 1/30.0
            time_constant = time_delay/2.5  # time delay is defined as the rise time
                                            # for reaching about 90% of step input
            pole_mag = 1.0/time_constant
            lag_ratio = 1.0 / (1.0 + pole_mag*dt)
            update_ratio = 1.0 - lag_ratio
            # y = lpf_by_for_loop(x, lag_ratio, update_ratio)
            y = lpf_by_scipy_lfilter(x, lag_ratio, update_ratio)
            return y
