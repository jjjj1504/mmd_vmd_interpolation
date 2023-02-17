# -*- coding: shift-jis -*-
import numpy as np

from mmd_vmd_interpolation.vmd_profile import VmdSimpleProfile
from mmd_vmd_interpolation.bones_pose_calculator import (
    BonesPoseCalculator,
    BonesTree,
)


def main():

    generate_nonrotatable_bones_data(
        src = "bbfモーションby@Ai.vmd",
        dst = "oooooooooout.vmd",
        motion_time_delay=0.5,
    )


def generate_nonrotatable_bones_data(src, dst, motion_time_delay=0.0):

    vp = VmdSimpleProfile(src)

    if vp.check_is_camera():
        print("Not bones data but camera data: " + src)
        return

    # setting
    desired_bones_names = ["全ての親","センター","グルーブ","腰","上半身","上半身2","首","頭","面","右肩P","右肩","右腕","左肩P","左肩","左腕"]
    bones_tree = BonesTree.get([
        ["全ての親", None, np.array([0., 0., 0.])],
        ["センター", "全ての親", np.array([0., 8., 0.])],
        ["グルーブ", "センター", np.array([0., 8.2, 0.])],
        ["腰", "グルーブ", np.array([0., 12., 0.255])],
        ["上半身", "腰", np.array([0., 12.8, -0.5])],
        ["上半身2", "上半身", np.array([0., 13.9, -0.46])],
        ["首", "上半身2", np.array([0., 16.34, -0.11])],
        ["頭", "首", np.array([0., 17.2, -0.12])],
        ["面", "頭", np.array([0., 17.8, -1.0])],
        ["右肩P", "上半身2", np.array([-0.235, 16.06, -0.15])],
        ["右肩", "右肩P", np.array([-0.235, 16.06, -0.15])],
        ["右腕", "右肩", np.array([-1.1, 15.8, -0.13])],
        ["左肩P", "上半身2", np.array([0.235, 16.06, -0.15])],
        ["左肩", "左肩P", np.array([0.235, 16.06, -0.15])],
        ["左腕", "左肩", np.array([1.1, 15.8, -0.13])],
    ])
    bones_name_remap = {
        "全ての親":"parent of all",
        "センター":"center",
        "グルーブ":"groove",
        "腰":"waist",
        "上半身":"upper body",
        "上半身2":"upper body 2",
        "首":"neck",
        "頭":"head",
        "面":"face",
        "右肩": "right shoulder",
        "右腕": "right arm",
        "左肩": "left shoulder",
        "左腕": "left arm",
    }
    dst_model_name = "nonrotatable_bone"

    # load
    model_name = vp.read_model_name()
    print("loading bonse data from model: %s ..." % model_name)
    bones_dict = vp.read_desired_bones(desired_bones_names)
    for bone_name in desired_bones_names:
        bone_data = bones_dict[bone_name]
        print("load %d frames of bone %s" % (bone_data.get_frame_num(), bone_data.name))

    # create object to processing bone data
    bpc = BonesPoseCalculator(bones_dict, bones_tree)

    # interpolation
    print("doing interpolation of bone data...")
    bones_interp_data = bpc.get_full_interp_bones()

    # generate nonrotatable bones
    print("generating nonrotatable bones...")
    nonrotatable_bones = bpc.get_lpf_full_positions_bones(motion_time_delay)
    nonrotatable_bones_remap = {new_name: nonrotatable_bones[old_name] for old_name, new_name in bones_name_remap.items()}

    # write to file
    print("exporting nonrotatable bone data to file: %s ..." % dst)
    vp.write_bones(dst, dst_model_name, nonrotatable_bones_remap)
    print("done!")


if __name__ == "__main__":
    main()
