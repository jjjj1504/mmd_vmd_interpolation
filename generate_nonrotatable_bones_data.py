# -*- coding: shift-jis -*-
import argparse

import numpy as np

from mmd_vmd_interpolation.bones_pose_calculator import (
    BonesPoseCalculator,
    BonesTree,
)
from mmd_vmd_interpolation.vmd_profile import VmdSimpleProfile


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("src", type=str, help="source motion vmd file")
    parser.add_argument(
        "-o", "--output", type=str, default="output_nonrotatable_bone_motion.vmd",
        help="output nonrotatable bones motion vmd file",
    )
    parser.add_argument(
        "-d", "--delay", type=float, default=0.0,
        help="time delay of motion smoothing (second)",
    )
    args = parser.parse_args()

    generate_nonrotatable_bones_data(
        src=args.src,
        dst=args.output,
        motion_time_delay=args.delay,
    )


def generate_nonrotatable_bones_data(src, dst, motion_time_delay=0.0):

    vp = VmdSimpleProfile(src)

    if vp.check_is_camera():
        print("Not bones data but camera data: " + src)
        return

    # setting
    desired_bones_names = [
        "SÄÌe", "Z^[", "O[u", "",
        "ăŒg", "ăŒg2", "ń", "Ș", "Ê",
        "EšP", "Eš", "Er", "¶šP", "¶š", "¶r",
    ]
    bones_tree = BonesTree.get([
        ["SÄÌe", None, np.array([0., 0., 0.])],
        ["Z^[", "SÄÌe", np.array([0., 8., 0.])],
        ["O[u", "Z^[", np.array([0., 8.2, 0.])],
        ["", "O[u", np.array([0., 12., 0.255])],
        ["ăŒg", "", np.array([0., 12.8, -0.5])],
        ["ăŒg2", "ăŒg", np.array([0., 13.9, -0.46])],
        ["ń", "ăŒg2", np.array([0., 16.34, -0.11])],
        ["Ș", "ń", np.array([0., 17.2, -0.12])],
        ["Ê", "Ș", np.array([0., 17.8, -1.0])],
        ["EšP", "ăŒg2", np.array([-0.235, 16.06, -0.15])],
        ["Eš", "EšP", np.array([-0.235, 16.06, -0.15])],
        ["Er", "Eš", np.array([-1.1, 15.8, -0.13])],
        ["¶šP", "ăŒg2", np.array([0.235, 16.06, -0.15])],
        ["¶š", "¶šP", np.array([0.235, 16.06, -0.15])],
        ["¶r", "¶š", np.array([1.1, 15.8, -0.13])],
    ])
    bones_name_remap = {
        "SÄÌe":"parent of all",
        "Z^[":"center",
        "O[u":"groove",
        "":"waist",
        "ăŒg":"upper body",
        "ăŒg2":"upper body 2",
        "ń":"neck",
        "Ș":"head",
        "Ê":"face",
        "Eš": "right shoulder",
        "Er": "right arm",
        "¶š": "left shoulder",
        "¶r": "left arm",
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
    print(
        "generating nonrotatable bones "
        "with %f sec of time delay..." % motion_time_delay
    )
    nonrotatable_bones = bpc.get_lpf_full_positions_bones(motion_time_delay)
    nonrotatable_bones_remap = {
        new_name: nonrotatable_bones[old_name] \
            for old_name, new_name in bones_name_remap.items()
    }

    # write to file
    print("exporting nonrotatable bone data to file: '%s' ..." % dst)
    vp.write_bones(dst, dst_model_name, nonrotatable_bones_remap)
    print("done!")


if __name__ == "__main__":
    main()
