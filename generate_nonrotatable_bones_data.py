# -*- coding: shift-jis -*-
import numpy as np

from mmd_vmd_interpolation.vmd_profile import VmdSimpleProfile
from mmd_vmd_interpolation.bones_pose_calculator import (
    BonesPoseCalculator,
    BonesTree,
)


def main():

    generate_nonrotatable_bones_data(
        src = "bbf���[�V����by@Ai.vmd",
        dst = "oooooooooout.vmd",
        motion_time_delay=0.5,
    )


def generate_nonrotatable_bones_data(src, dst, motion_time_delay=0.0):

    vp = VmdSimpleProfile(src)

    if vp.check_is_camera():
        print("Not bones data but camera data: " + src)
        return

    # setting
    desired_bones_names = ["�S�Ă̐e","�Z���^�[","�O���[�u","��","�㔼�g","�㔼�g2","��","��","��","�E��P","�E��","�E�r","����P","����","���r"]
    bones_tree = BonesTree.get([
        ["�S�Ă̐e", None, np.array([0., 0., 0.])],
        ["�Z���^�[", "�S�Ă̐e", np.array([0., 8., 0.])],
        ["�O���[�u", "�Z���^�[", np.array([0., 8.2, 0.])],
        ["��", "�O���[�u", np.array([0., 12., 0.255])],
        ["�㔼�g", "��", np.array([0., 12.8, -0.5])],
        ["�㔼�g2", "�㔼�g", np.array([0., 13.9, -0.46])],
        ["��", "�㔼�g2", np.array([0., 16.34, -0.11])],
        ["��", "��", np.array([0., 17.2, -0.12])],
        ["��", "��", np.array([0., 17.8, -1.0])],
        ["�E��P", "�㔼�g2", np.array([-0.235, 16.06, -0.15])],
        ["�E��", "�E��P", np.array([-0.235, 16.06, -0.15])],
        ["�E�r", "�E��", np.array([-1.1, 15.8, -0.13])],
        ["����P", "�㔼�g2", np.array([0.235, 16.06, -0.15])],
        ["����", "����P", np.array([0.235, 16.06, -0.15])],
        ["���r", "����", np.array([1.1, 15.8, -0.13])],
    ])
    bones_name_remap = {
        "�S�Ă̐e":"parent of all",
        "�Z���^�[":"center",
        "�O���[�u":"groove",
        "��":"waist",
        "�㔼�g":"upper body",
        "�㔼�g2":"upper body 2",
        "��":"neck",
        "��":"head",
        "��":"face",
        "�E��": "right shoulder",
        "�E�r": "right arm",
        "����": "left shoulder",
        "���r": "left arm",
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
