# -*- coding: utf-8 -*-
import numpy as np
import struct


class VmdSimpleProfile:

    _CODING = "shift-jis"

    ## vmd format
    ## https://mikumikudance.fandom.com/wiki/VMD_file_format
    ## https://blog.csdn.net/haseetxwd/article/details/82821533

    _VERSION_LEN = 30
    _NEW_VERSION_HEADER = "Vocaloid Motion Data 0002"
    _MODEL_NAME_LEN = {
        "Vocaloid Motion Data file" : 10,
        "Vocaloid Motion Data 0002" : 20,
    }
    _CAMERA_HEADER_NAME = "カメラ・照明"

    ## struct usage
    ## https://docs.python.org/3/library/struct.html
    _FRAME_NUM_FORMAT = struct.Struct("I")
    _FRAME_NUM_LEN = 4
    _BONE_FORMAT = struct.Struct("I3f4f64b")
    _BONE_LEN = 15 + 4 + 3*4 + 4*4 + 64
    _BONE_ATTR_NUM = 1 + 3 + 4 + 64
    _MORPH_FORMAT = struct.Struct("If")
    _MORPH_LEN = 15 + 4 + 4
    _CAMERA_FORMAT = struct.Struct("If3f3f24BI?")
    _CAMERA_LEN = 4 + 4 + 3*4 + 3*4 + 24 + 4 + 1
    _CAMERA_ATTR_NUM = 1 + 1 + 3 + 3 + 24 + 1 + 1
    _LIGHT_FORMAT = struct.Struct("I3f3f")
    _LIGHT_LEN = 4 + 3*4 + 3*4

    _BONE_NAME_LEN = 15
    _BONE_BIN_LEN = _BONE_LEN - _BONE_NAME_LEN
    _MORPH_NAME_LEN = 15
    _MORPH_BIN_LEN = _MORPH_LEN - _MORPH_NAME_LEN

    def __init__(self, src):
        # type: (str) -> (None)
        self.src = src

    def _seek(self, fp, part):
        # type: (file, str) -> (None)
        # go to the start point of file header version
        fp.seek(0, 0)
        if part == "version":
            return
        # go to the start point of model name part
        header_version = self._get_header_version(fp)
        if part == "name":
            return
        # go to the start point of bones data
        self._get_model_name(fp, header_version)
        if part in ["bone", "bones"]:
            return
        # go to the start point of morph data
        bone_frame_num = self._get_frame_num(fp)
        fp.seek(bone_frame_num * self._BONE_LEN, 1)
        if part == "morph":
            return
        # go to the start point of camera data
        morph_frame_num = self._get_frame_num(fp)
        fp.seek(morph_frame_num * self._MORPH_LEN, 1)
        if part == "camera":
            return
        # go to the start point of light data
        camera_frame_num = self._get_frame_num(fp)
        fp.seek(camera_frame_num * self._CAMERA_LEN, 1)
        if part == "light":
            return

    def _get_frame_num(self, fp):
        return self._FRAME_NUM_FORMAT.unpack(fp.read(self._FRAME_NUM_LEN))[0]

    def _get_header_version(self, fp):
        header_version_raw = fp.read(self._VERSION_LEN)
        return self._decode_text(header_version_raw)

    def _get_model_name(self, fp, header_version):
        model_name_raw = fp.read(self._MODEL_NAME_LEN[header_version])
        return self._decode_text(model_name_raw)

    def _get_bone_name(self, fp):
        bone_name_raw = fp.read(self._BONE_NAME_LEN)
        return self._decode_text(bone_name_raw)

    def _get_bones_list(self, fp):
        self._seek(fp, "bone")
        bone_frame_num = self._get_frame_num(fp)
        bones_list = {}   # type: dict[str, int]
        for _ in range(bone_frame_num):
            bone_name = self._get_bone_name(fp)
            if bone_name not in bones_list:
                bones_list[bone_name] = 1
            else:
                bones_list[bone_name] += 1
            # skip binary data
            fp.seek(self._BONE_BIN_LEN, 1)
        return bones_list

    def _get_desired_bones_data(self, fp, required_bones_names):
        # initialize
        data_dict = dict.fromkeys(required_bones_names)
        for bone_name in required_bones_names:
            data_dict[bone_name] = VmdBoneData(bone_name)
        # seek required bones in bones data block
        self._seek(fp, "bone")
        bone_frame_num = self._get_frame_num(fp)
        for _ in range(bone_frame_num):
            bone_name = self._get_bone_name(fp)
            if bone_name not in data_dict:
                # skip
                fp.seek(self._BONE_BIN_LEN, 1)
            else:
                # decode binary data to numerical data
                raw = self._BONE_FORMAT.unpack(fp.read(self._BONE_BIN_LEN))
                data_dict[bone_name].append(
                    frame_id =raw[0],
                    position = raw[1:4],
                    orientation = raw[4:8],
                    curve_x = raw[ 8:24:4],
                    curve_y = raw[24:40:4],
                    curve_z = raw[40:56:4],
                    curve_rot = raw[56::4],
                )
        # reindex
        for bone_name in required_bones_names:
            data_dict[bone_name].sort_frame()
        return data_dict

    def _get_camera_data(self, fp):
        self._seek(fp, "camera")
        camera_frame_num = self._get_frame_num(fp)
        data = VmdCameraData(camera_frame_num)
        for i in range(camera_frame_num):
            raw = self._CAMERA_FORMAT.unpack(fp.read(self._CAMERA_LEN))
            data.frame_ids[i] = raw[0]
            data.distances[i] = raw[1]
            data.positions[i] = raw[2:5]
            data.orientations[i] = raw[5:8]
            data.curve_x[i][[0,2,1,3]] = raw[ 8:12]
            data.curve_y[i][[0,2,1,3]] = raw[12:16]
            data.curve_z[i][[0,2,1,3]] = raw[16:20]
            data.curve_rot[i][[0,2,1,3]] = raw[20:24]
            data.curve_dis[i][[0,2,1,3]] = raw[24:28]
            data.curve_fov[i][[0,2,1,3]] = raw[28:32]
            data.fov_angles[i] = float(raw[32])
            data.perspective_flags[i] = raw[33]
        data.sort_frame()
        return data

    def read_model_name(self):
        with open(self.src, "rb") as fp:
            header_version = self._get_header_version(fp)
            model_name = self._get_model_name(fp, header_version)
            return model_name

    def check_is_camera(self):
        model_name = self.read_model_name()
        return model_name.startswith(self._CAMERA_HEADER_NAME)

    def read_desired_bones(self, desired_bones_names):
        with open(self.src, "rb") as fp:
            bone_data = self._get_desired_bones_data(fp, desired_bones_names)
            return bone_data

    def read_camera(self):
        with open(self.src, "rb") as fp:
            return self._get_camera_data(fp)

    @classmethod
    def write_bones(cls, dst, model_name, bones_data):
        with open(dst,"wb") as fp:
            # header
            version_header_raw = cls._encode_text(cls._NEW_VERSION_HEADER, cls._VERSION_LEN)
            fp.write(version_header_raw)
            # model name
            model_name_len = cls._MODEL_NAME_LEN[cls._NEW_VERSION_HEADER]
            model_name_raw = cls._encode_text(model_name, model_name_len)
            fp.write(model_name_raw)
            # bone
            bones_frames_num = sum([len(b.frame_ids) for b in bones_data.values()])
            fp.write(cls._FRAME_NUM_FORMAT.pack(bones_frames_num))
            raw = [0] * cls._BONE_ATTR_NUM
            for name, bone_data in bones_data.items():
                name_raw = cls._encode_text(name, cls._BONE_NAME_LEN)
                for i in range(len(bone_data.frame_ids)):
                    fp.write(name_raw)
                    raw[0] = bone_data.frame_ids[i]
                    raw[1:4] = bone_data.positions[i]
                    raw[4:8] = bone_data.orientations[i]
                    raw[ 8:24:4] = bone_data.curve_x[i]
                    raw[24:40:4] = bone_data.curve_y[i]
                    raw[40:56:4] = bone_data.curve_z[i]
                    raw[56::4] = bone_data.curve_rot[i]
                    fp.write(cls._BONE_FORMAT.pack(*raw))
            # morph
            fp.write(cls._FRAME_NUM_FORMAT.pack(0))
            # camera
            fp.write(cls._FRAME_NUM_FORMAT.pack(0))
            # light
            fp.write(cls._FRAME_NUM_FORMAT.pack(0))

    @classmethod
    def write_camera(cls, dst, camera_data):
        # type: (str, VmdCameraData) -> (None)
        with open(dst,"wb") as fp:
            # header
            version_header_raw = cls._encode_text(cls._NEW_VERSION_HEADER, cls._VERSION_LEN)
            fp.write(version_header_raw)
            # model name
            model_name_len = cls._MODEL_NAME_LEN[cls._NEW_VERSION_HEADER]
            model_name_raw = cls._encode_text(cls._CAMERA_HEADER_NAME, model_name_len)
            fp.write(model_name_raw)
            # bone
            fp.write(cls._FRAME_NUM_FORMAT.pack(0))
            # morph
            fp.write(cls._FRAME_NUM_FORMAT.pack(0))
            # camera
            frame_num = camera_data.get_frame_num()
            fp.write(cls._FRAME_NUM_FORMAT.pack(frame_num))
            raw = [0] * cls._CAMERA_ATTR_NUM
            for i in range(frame_num):
                raw[0] = camera_data.frame_ids[i]
                raw[1] = camera_data.distances[i]
                raw[2:5] = camera_data.positions[i]
                raw[5:8] = camera_data.orientations[i]
                raw[ 8:12] = camera_data.curve_x[i][[0,2,1,3]]
                raw[12:16] = camera_data.curve_y[i][[0,2,1,3]]
                raw[16:20] = camera_data.curve_z[i][[0,2,1,3]]
                raw[20:24] = camera_data.curve_rot[i][[0,2,1,3]]
                raw[24:28] = camera_data.curve_dis[i][[0,2,1,3]]
                raw[28:32] = camera_data.curve_fov[i][[0,2,1,3]]
                raw[32] = int(round(camera_data.fov_angles[i]))
                raw[33] = camera_data.perspective_flags[i]
                fp.write(cls._CAMERA_FORMAT.pack(*raw))
            # light
            fp.write(cls._FRAME_NUM_FORMAT.pack(0))

    @classmethod
    def _decode_text(cls, raw):
        # ignore the string behind "\x00"
        raw = raw.split(b"\x00")[0]
        # convert bytes to string
        text = raw.decode(cls._CODING)
        return text

    @classmethod
    def _encode_text(cls, text, length):
        # convert string to bytes
        raw = text.encode(cls._CODING)
        # pad "\x00"
        raw = raw.ljust(length, b"\x00")
        return raw


class VmdDataBase(object):

    _CURVE_DEFAULT = np.array([20, 20, 107, 107])
    _QUATERNION_DEFAULT = np.array([0., 0., 0., 1.])

    def sort_frame(self):
        # order of vmd frame data is along the order the registration instead of time
        # but to process them, it has better to be sorted along time
        sort_order = np.argsort(self.frame_ids)
        for member_name, member_value in self.__dict__.items():
            if type(member_value) is np.ndarray:
                member_value[:] = member_value[sort_order]

    def get_frame_num(self):
        return len(self.frame_ids)

    @classmethod
    def _gen_default_curve(cls, frame_num):
        return np.tile(cls._CURVE_DEFAULT, [frame_num, 1])

    @classmethod
    def _gen_default_quaternion(cls, frame_num):
        return np.tile(cls._QUATERNION_DEFAULT, [frame_num, 1])

    def apply_mask(self, mask):
        for member_name, member_value in self.__dict__.items():
            if type(member_value) is np.ndarray:
                setattr(self, member_name, member_value[mask])


class VmdCameraData(VmdDataBase):

    def __init__(self, frame_num):
        # type: (int) -> (None)
        self.frame_ids = np.zeros(frame_num, dtype="int")
        self.distances = np.zeros(frame_num)
        self.positions = np.zeros([frame_num, 3])
        self.orientations = np.zeros([frame_num, 3])
        self.curve_x = self._gen_default_curve(frame_num)
        self.curve_y = self._gen_default_curve(frame_num)
        self.curve_z = self._gen_default_curve(frame_num)
        self.curve_rot = self._gen_default_curve(frame_num)
        self.curve_dis = self._gen_default_curve(frame_num)
        self.curve_fov = self._gen_default_curve(frame_num)
        self.fov_angles = np.zeros([frame_num])
        self.perspective_flags = np.ones(frame_num, "bool")


class VmdBoneData(VmdDataBase):

    def __init__(self, name, frame_num=None):
        # type: (str, int) -> (None)
        self.name = name
        self.frame_ids = []
        self.positions = []
        self.orientations = []
        self.curve_x = []
        self.curve_y = []
        self.curve_z = []
        self.curve_rot = []
        if frame_num is not None:
            self.allocate(frame_num)

    def allocate(self, frame_num):
        self.frame_ids = np.zeros(frame_num, dtype="int")
        self.positions = np.zeros([frame_num, 3])
        self.orientations = self._gen_default_quaternion(frame_num)
        self.curve_x = self._gen_default_curve(frame_num)
        self.curve_y = self._gen_default_curve(frame_num)
        self.curve_z = self._gen_default_curve(frame_num)
        self.curve_rot = self._gen_default_curve(frame_num)

    def append(self,
        frame_id,
        position,
        orientation,
        curve_x,
        curve_y,
        curve_z,
        curve_rot,
        ):
        self.frame_ids.append(frame_id)
        self.positions.append(position)
        self.orientations.append(orientation)
        self.curve_x.append(curve_x)
        self.curve_y.append(curve_y)
        self.curve_z.append(curve_z)
        self.curve_rot.append(curve_rot)

    def sort_frame(self):
        # convert list to numpy array
        for member_name, member_value in self.__dict__.items():
            if isinstance(member_value, list):
                setattr(self, member_name, np.array(member_value))
        # sort numpy array along frame_ids
        VmdDataBase.sort_frame(self)
