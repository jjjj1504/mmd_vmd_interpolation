from mmd_vmd_interpolation.vmd_profile import VmdSimpleProfile
from mmd_vmd_interpolation.bones_pose_calculator import BonesPoseCalculator
from mmd_vmd_interpolation.camera_trace_bone import (
    CameraSmoother,
    CameraTracer,
)


def main():

    generate_bone_tracing_camera_data(
        src_camera = "bbf_camera_trace_face.vmd",
        dst_camera = "9898989898.vmd",
        src_nonrotatable_bone = "short.vmd", #"bbfƒ‚[ƒVƒ‡ƒ“by@Ai_nonrotatable_delay05.vmd",
        trace_bone_name = "face",
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
    cs = CameraSmoother(camera_data)

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
        camera_interp.positions = CameraTracer.trace_bone(camera_interp, bone_data)

    if camera_shake_interval > 0. and camera_shake_amplitude > 0.:
        print("add cammera shake...")
        camera_interp.positions = CameraTracer.add_camera_shake(camera_interp, camera_shake_interval, camera_shake_amplitude)

    # write to file
    print("exporting camera data to file: %s ..." % dst_camera)
    vpc.write_camera(dst_camera, camera_interp)
    print("done!")


if __name__ == "__main__":
    main()
