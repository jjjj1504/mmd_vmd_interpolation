import nump as np

from mmd_curve_interp import MMDCurveInterp
from vmd_profile import VmdBoneData


class BonesTree(object):
    _bones_list = [
        ["全ての親", None, np.array([0.,0.,0.])],
        ["センター", "全ての親", np.array([0.,0.,0.])],
        ["グルーブ", "センター", np.array([0.,0.,0.])],
        ["腰", "グルーブ", np.array([0.,0.,0.])],
        ["上半身", "腰", np.array([0.,0.,0.])],
        ["上半身2", "上半身", np.array([0.,0.,0.])],
        ["首", "上半身2", np.array([0.,0.,0.])],
        ["頭", "首", np.array([0.,0.,0.])],
        ["面", "頭", np.array([0.,0.,0.])],
    ]

    @classmethod
    def get(cls, bones_list=None):
        if bones_list is None:
            bones_list = cls._bones_list
        bones_tree = {}
        for bone_name, parent_name, position in bones_list:
            bones_tree[bone_name] = {
                "parent": parent_name,
                "position": position,
                "trans_from_parent": None,
            }
        for bone_name, bone_info in bones_tree.items():
            if bone_info["parent"] is not None:
                bone_info["trans_from_parent"] = \
                    bone_info["position"] - bones_tree[bone_info["parent"]]["position"]
        return bones_tree


class BonesPoseCalculator(object):

    def __init__(self, bones_data):
        self._bones_data = bones_data  # type: dict[str, VmdBoneData]
        self._bones_tree = BonesTree.get()
        self._full_frame_num = max([b.frame_ids[-1] if len(b.frame_ids) else 0 for b in self._bones_data.values()])
        self._bones_full_interp = {}  # type: dict[str, VmdBoneData]
        self._bones_full_pose = {}  # type: dict[str, VmdBoneData]
        self._bones_full_position_lpf = {}  # type: dict[str, np.ndarray]

    def get_full_interp_bones(self):
        if self._bones_full_interp:
            return self._bones_full_interp
        # loop to do data interpolation for each bone
        for name, bone_data in self._bones_data.items():
            bone_full_interp = VmdBoneData(name, self._full_frame_num)
            bone_full_interp.frame_ids = np.arange(self._full_frame_num)
            # loop to do data interpolation for each interval
            for i in range(len(bone_data.frame_ids)-1):
                frame_id_endpoint = bone_data.frame_ids[i:i+2]
                fid0, fid1 = frame_id_endpoint
                if fid1 - fid0 > 1:
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
                else:
                    # no need to do interpolation with single frame
                    position_interp = bone_data.positions[i, :].reshape(1,-1)
                    orientation_interp = bone_data.orientations[i, :].reshape(1,-1)
                # record interval data
                bone_full_interp.positions[fid0:fid1, :] = position_interp
                bone_full_interp.orientations[fid0:fid1, :] = orientation_interp
            # append the last frame
            if len(bone_data.frame_ids) > 1:
                bone_full_interp.positions[-1, :] = bone_data.positions[i, :]
                bone_full_interp.orientations[-1, :] = bone_data.orientations[i, :]
            elif len(bone_data.frame_ids) == 1:
                bone_full_interp.positions[:] = bone_data.positions[0, :]
                bone_full_interp.orientations[:] = bone_data.orientations[0, :]
            # record interpolated bone data
            self._bones_full_interp[name] = bone_full_interp
        # end of loop for doing data interpolation for each bone 
        return self._bones_full_interp

    def _gen_full_interp_bone(self, bone_name):
        # get parent bone
        parent_name = self._bones_tree[bone_name]["parent"]
        if parent_name is not None:
            if parent_name not in self._bones_full_interp:
                self._gen_full_interp_bone(parent_name)
            parent_bone = self._bones_full_interp[parent_name]
        else:
            parent_bone = None
        # 


        for bone_name in self._bones_data.keys():
            full_bone = VmdBoneData(bone_name, full_frame_num)
            full_bone.frame_ids = np.arange(full_frame_num)
            self._bones_full_interp[bone_name] = full_bone
        
    def apply_lpf_to_trans(self):
        pass
