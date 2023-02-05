import nump as np

from mmd_curve_interp import MMDCurveInterp
from vmd_profile import VmdBoneData


class BonesTree(object):

    @classmethod
    def get(cls, bones_list):
        bones_tree = {}
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
                bone_info["trans_from_parent"] = \
                    bone_info["position"] - bones_tree[bone_info["parent"]]["position"]
        return bones_tree


class BonesPoseCalculator(object):

    def __init__(self, bones_data, bone_tree={}):
        self._bones_data = bones_data  # type: dict[str, VmdBoneData]
        self._bones_tree = bone_tree
        self._full_frame_num = max([b.frame_ids[-1] if b.get_frame_num() else 0 for b in self._bones_data.values()])
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
            parent_full_pose.orientations.T, parent_full_pose.positions.T,
            bone_full_interp.orientations.T, (bone_full_interp.positions + trans_from_parent).T,
        )
        bone_full_pose.orientations = full_orientations_T.T
        bone_full_pose.positions = full_positions_T.T
        # record
        self._bones_full_pose[bone_name] = bone_full_pose
        return self._bones_full_pose[bone_name]

    def apply_lpf_to_trans(self):
        pass
