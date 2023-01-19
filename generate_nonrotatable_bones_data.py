from mmd_vmd_interpolation.vmd_profile import VmdSimpleProfile
from mmd_vmd_interpolation.bones_pose_calculator import BonesPoseCalculator


def main():

    src = "bbfモーションby@Ai.vmd"

    vp = VmdSimpleProfile(src)

    if vp.check_is_camera():
        print("load camera data")
        camera_data = vp.read_camera()
        print("load %d frames of camera" % len(camera_data.frame_ids))
        return camera_data
    else:
        model_name = vp.read_model_name()
        print("load bonse data from model: %s" % model_name)
        desired_bones_names = ["全ての親","センター","グルーブ","腰","上半身","上半身2","首","頭","面"]
        bones_dict = vp.read_desired_bones(desired_bones_names)
        for bone_name in desired_bones_names:
            bone_data = bones_dict[bone_name]
            print("load %d frames of bone %s" % (len(bone_data.frame_ids), bone_data.name))
        bpc = BonesPoseCalculator(bones_dict)
        bones_interp_data = bpc.get_full_interp_bones()
        vp.write_bones("oooooooooout.vmd", model_name, bones_interp_data)
        return bones_dict


def generate_nonrotatable_bones_data(src):

    vp = VmdSimpleProfile(src)

    if vp.check_is_camera():
        print("Not bones data but camera data: " + src)
        return

    model_name = vp.read_model_name()
    print("load bonse data from model: %s" % model_name)
    desired_bones_names = ["全ての親","センター","グルーブ","腰","上半身","上半身2","首","頭","面"]
    bones_dict = vp.read_desired_bones(desired_bones_names)
    for bone_name in desired_bones_names:
        bone_data = bones_dict[bone_name]
        print("load %d frames of bone %s" % (len(bone_data.frame_ids), bone_data.name))


if __name__ == "__main__":
    main()
