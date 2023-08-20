# -*- coding: utf-8 -*-
import argparse

from mmd_vmd_interpolation.bones_pose_calculator import BonesPoseCalculator
from mmd_vmd_interpolation.camera_trace_bone import (
    CameraSmoother,
    CameraTracer,
)
from mmd_vmd_interpolation.vmd_profile import VmdSimpleProfile


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "src_camera", type=str,
        help="source camera vmd file",
    )
    parser.add_argument(
        "-o", "--output", type=str, default="output_camera.vmd",
        help="output camera vmd file",
    )
    parser.add_argument(
        "-b", "--src_nonrotatable_bone", type=str,
        help="nonrotatable bone vmd file",
    )
    parser.add_argument(
        "-t", "--trace_bone_name", type=str,
        help="name of the bone camera wanted to trace",
    )
    parser.add_argument(
        "--shake_interval", type=float, default=0.0,
        help="period of camera shaking motion (second)",
    )
    parser.add_argument(
        "--shake_amplitude", type=float, default=0.0,
        help="aplitude of camera shaking motion (meter)",
    )
    parser.add_argument(
        "--force_default_interp", action="store_true",
        help="flag of using mmd default interpolation",
    )
    parser.add_argument(
        "--smooth_fov_angles", action="store_true",
        help="flag of smoothing camera fov angles",
    )
    parser.add_argument(
        "--interp_frame_interval", type=int, default=2,
        help="number of frames between 2 interpolation frames",
    )
    args = parser.parse_args()

    # warning message about src_nonrotatable_bone
    if args.src_nonrotatable_bone and not args.trace_bone_name:
        print(
            "\nWarning: because trace_bone_name is not given, "
            "ignore src_nonrotatable_bone: '%s'\n"
             % args.src_nonrotatable_bone
        )
    elif not args.src_nonrotatable_bone and args.trace_bone_name:
        print(
            "\nWarning: because src_nonrotatable_bone is not given, "
            "ignore trace_bone_name: '%s'\n"
             % args.trace_bone_name
        )
    # warning message about camera shaking
    if args.shake_interval and not args.shake_amplitude:
        print(
            "\nWarning: because shake_amplitude is not given, "
            "ignore shake_interval: %f sec\n" % args.shake_interval
        )
    elif not args.shake_interval and args.shake_amplitude:
        print(
            "\nWarning: because shake_interval is not given, "
            "ignore shake_amplitude: %f m\n" % args.shake_amplitude
        )

    generate_bone_tracing_camera_data(
        src_camera=args.src_camera,
        dst_camera=args.output,
        src_nonrotatable_bone=args.src_nonrotatable_bone,
        trace_bone_name=args.trace_bone_name,
        camera_shake_interval=args.shake_interval,
        camera_shake_amplitude=args.shake_amplitude,
        need_smooth=not args.force_default_interp,
        need_smooth_fov_angles=args.smooth_fov_angles,
        interp_frame_interval=args.interp_frame_interval,
    )


def generate_bone_tracing_camera_data(
        src_camera,
        dst_camera,
        need_smooth=True,
        need_smooth_fov_angles=False,
        camera_shake_interval=1.0,
        camera_shake_amplitude=0.1,
        src_nonrotatable_bone=None,
        trace_bone_name=None,
        interp_frame_interval=2,
    ):

    vpc = VmdSimpleProfile(src_camera)

    if not vpc.check_is_camera():
        print("Not camera data but bone data: " + src_camera)
        return

    # load
    print("load camera data")
    camera_data = vpc.read_camera()
    print("load %d frames of camera" % len(camera_data.frame_ids))

    # create object to processing camera data
    cs = CameraSmoother(camera_data, interp_frame_interval)

    # interpolation
    print("doing interpolation of camera data...")
    camera_interp = cs.interp(need_smooth, need_smooth_fov_angles)

    if src_nonrotatable_bone and trace_bone_name:
        vpb = VmdSimpleProfile(src_nonrotatable_bone)
        if vpb.check_is_camera():
            print("Not bone data but camera data: " + src_nonrotatable_bone)
        else:
            print("load fully interpolated nonrotatable bone data...")
            bone_data = vpb.read_desired_bones({trace_bone_name})[trace_bone_name]
            print("load %d frames of bone %s" % (bone_data.get_frame_num(), bone_data.name))

        print("calculate camera tracing bone...")
        # camera distance data is redundant (useless, and misleading) for bone tracing
        camera_interp.distances = camera_interp.positions[:,2]
        camera_interp.positions = CameraTracer.trace_bone(camera_interp, bone_data)

    if camera_shake_interval > 0. and camera_shake_amplitude > 0.:
        print(
            "add cammera shake with interval %f sec and amplitiude %f m ..."
            % (camera_shake_interval, camera_shake_amplitude)
        )
        camera_interp.positions = CameraTracer.add_camera_shake(
            camera_interp, camera_shake_interval, camera_shake_amplitude,
        )

    # write to file
    print("exporting camera data to file: '%s' ..." % dst_camera)
    vpc.write_camera(dst_camera, camera_interp)
    print("done!")


if __name__ == "__main__":
    main()
