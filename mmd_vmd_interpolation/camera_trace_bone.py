import numpy as np
import scipy.interpolate

from mmd_curve_interp import MMDCurveInterp
from transform import Transform
from vmd_profile import VmdCameraData


class CameraSmoother(object):

    def __init__(self, camera_data, interp_frame_interval=2):
        self._camera_data = camera_data  # type: VmdCameraData
        self._interp_fram_ids = np.zeros(0, dtype="int")
        self._interp_frame_loc = np.zeros(0, dtype="int")
        self._seg_frame_loc = np.zeros(0, dtype="int")
        self._seg_frame_interp_loc = np.zeros(0, dtype="int")
        self._interp_frame_interval = interp_frame_interval
        self._determine_interp_frame_num()
        self._camera_data_interp = VmdCameraData(len(self._interp_fram_ids))
        self._camera_data_interp.frame_ids = self._interp_fram_ids

    def _determine_interp_frame_num(self):
        interp_frame_ids = []
        interp_frame_loc = [0]
        seg_frame_loc = [0]
        seg_frame_interp_loc = [0]
        seg_frame_interp_num = 0
        for i in range(self._camera_data.get_frame_num()-1):
            fid0, fid1 = self._camera_data.frame_ids[i:i+2]
            if self._camera_data.fov_angles[i] == self._camera_data.fov_angles[i+1]:
                interp_frame_interval = self._interp_frame_interval
            else:
                interp_frame_interval = 1
            interp_frame_id = np.arange(fid0, max(fid0+1, fid1-1), interp_frame_interval)
            interp_frame_num = len(interp_frame_id)
            interp_frame_ids.append(interp_frame_id)
            interp_frame_loc.append(interp_frame_loc[-1] + interp_frame_num)
            seg_frame_interp_num += interp_frame_num
            if fid1 - fid0 == 1:
                seg_frame_loc.append(i+1)
                seg_frame_interp_loc.append(seg_frame_interp_loc[-1] + seg_frame_interp_num)
                seg_frame_interp_num = 0

        interp_frame_ids.append(np.array([self._camera_data.frame_ids[-1]]))
        seg_frame_loc.append(self._camera_data.get_frame_num())
        seg_frame_interp_loc.append(seg_frame_interp_loc[-1] + seg_frame_interp_num + 1)
        self._interp_fram_ids = np.concatenate(interp_frame_ids)
        self._interp_frame_loc = np.array(interp_frame_loc)
        self._seg_frame_loc = np.array(seg_frame_loc)
        self._seg_frame_interp_loc = np.array(seg_frame_interp_loc)

    def interp(self, need_smooth, need_smooth_fov_angles=False):
        # most
        if need_smooth:
            self._interp_smooth(self._camera_data.positions[:,0], self._camera_data.curve_x, self._camera_data_interp.positions[:,0])
            self._interp_smooth(self._camera_data.positions[:,1], self._camera_data.curve_y, self._camera_data_interp.positions[:,1])
            self._interp_smooth(self._camera_data.positions[:,2], self._camera_data.curve_z, self._camera_data_interp.positions[:,2])
            self._interp_smooth(self._camera_data.orientations, self._camera_data.curve_rot, self._camera_data_interp.orientations)
            self._interp_smooth(self._camera_data.distances, self._camera_data.curve_dis, self._camera_data_interp.distances)
        else:
            self._interp_default(self._camera_data.positions[:,0], self._camera_data.curve_x, self._camera_data_interp.positions[:,0])
            self._interp_default(self._camera_data.positions[:,1], self._camera_data.curve_y, self._camera_data_interp.positions[:,1])
            self._interp_default(self._camera_data.positions[:,2], self._camera_data.curve_z, self._camera_data_interp.positions[:,2])
            self._interp_default(self._camera_data.orientations, self._camera_data.curve_rot, self._camera_data_interp.orientations)
            self._interp_default(self._camera_data.distances, self._camera_data.curve_dis, self._camera_data_interp.distances)
        # just check how much overshoot is in the interpolation
        need_plot_curve_for_debug = False
        if need_plot_curve_for_debug:
            self._plot_for_debug()
        # fov
        if need_smooth_fov_angles:
            self._interp_smooth(
                self._camera_data.fov_angles, self._camera_data.curve_fov,
                self._camera_data_interp.fov_angles,
            )
            mask = np.ones(self._camera_data_interp.get_frame_num(), dtype="bool")
        else:
            self._interp_default(
                self._camera_data.fov_angles, self._camera_data.curve_fov,
                self._camera_data_interp.fov_angles,
            )
            mask = self._get_various_mask_for_default(
                self._camera_data.fov_angles, self._camera_data_interp.fov_angles
            )
        # perspective
        self._interp_constant(
            self._camera_data.perspective_flags,
            self._camera_data_interp.perspective_flags,
        )
        # apply mask
        self._camera_data_interp.apply_mask(mask)
        # return
        return self._camera_data_interp

    def _interp_default(self, values, curves, values_interp):
        if self._camera_data.get_frame_num() > 1:
            frame_ids = self._camera_data.frame_ids
            interp_fram_ids = self._interp_fram_ids
            for i in range(self._camera_data.get_frame_num()-1):
                loc0, loc1 = self._interp_frame_loc[i:i+2]
                # start frame
                values_interp[loc0] = values[i]
                # do data interpolation (exclude start frame for boundary precision issue)
                values_interp[loc0+1: loc1] = MMDCurveInterp.interp(
                    frame_id_endpoint = frame_ids[i:i+2],
                    value_endpoint = values[i:i+2],
                    curve_param = curves[i+1],
                    frame_ids_desired = interp_fram_ids[loc0+1 : loc1],
                )
            # append the last frame
            values_interp[-1] = values[-1]
        # padding constant data for single frame
        elif self._camera_data.get_frame_num() == 1:
            values_interp[:] = values[0]
        # remain default value if 0 frame
        else:
            pass

    def _interp_smooth(self, values, curves, values_interp):
        if self._camera_data.get_frame_num() > 1:
            frame_ids = self._camera_data.frame_ids
            interp_fram_ids = self._interp_fram_ids
            seg_frame_loc = self._seg_frame_loc
            seg_frame_interp_loc = self._seg_frame_interp_loc
            for i in range(len(seg_frame_loc)-1):
                loc0, loc1 = seg_frame_loc[i:i+2]
                interp_loc0, interp_loc1 = seg_frame_interp_loc[i:i+2]
                if loc1 - loc0 > 2:
                    values_interp[interp_loc0:interp_loc1] = SmoothInterp.interp(
                        frame_ids = frame_ids[loc0:loc1],
                        values = values[loc0:loc1],
                        frame_ids_desired = interp_fram_ids[interp_loc0:interp_loc1],
                    )
                elif loc1 - loc0 == 2:
                    # do data interpolation (exclude start frame for boundary precision issue)
                    values_interp[interp_loc0:interp_loc1] = MMDCurveInterp.interp(
                        frame_id_endpoint = frame_ids[loc0: loc1],
                        value_endpoint = values[loc0: loc1],
                        curve_param = curves[loc0+1],
                        frame_ids_desired = interp_fram_ids[interp_loc0:interp_loc1],
                    )
                elif loc1 - loc0 == 1:
                    values_interp[interp_loc0] = values[loc0]
                else:
                    raise Exception("what?")
        # padding constant data for single frame
        elif self._camera_data.get_frame_num() == 1:
            values_interp[:] = values[0]
            pass
        # remain default value if 0 frame
        else:
            pass

    def _interp_constant(self, values, values_interp):
        if self._camera_data.get_frame_num() > 1:
            for i in range(self._camera_data.get_frame_num()-1):
                loc0, loc1 = self._interp_frame_loc[i:i+2]
                # start frame
                values_interp[loc0:loc1] = values[i]
            # append the last frame
            values_interp[-1] = values[-1]
        # padding constant data for single frame
        elif self._camera_data.get_frame_num() == 1:
            values_interp[:] = values[0]
        # remain default value if 0 frame
        else:
            pass

    def _plot_for_debug(self):
        def plot(y, y_interp):
            import matplotlib.pyplot as plt
            x = self._camera_data.frame_ids
            x_interp = self._camera_data_interp.frame_ids
            plt.plot(x, y, ".")
            plt.plot(x_interp, y_interp)
            plt.grid()
            plt.show()

        plot(
            self._camera_data.orientations[:,0],
            self._camera_data_interp.orientations[:,0]
        )

    def _get_various_mask_for_default(self, values, values_interp):
        if self._camera_data.get_frame_num() > 1:
            frame_ids = self._camera_data.frame_ids
            interp_fram_ids = self._interp_fram_ids
            mask = np.zeros(len(values_interp), dtype="bool")
            for i in range(self._camera_data.get_frame_num()-1):
                loc0, loc1 = self._interp_frame_loc[i:i+2]
                if values[i] == values[i+1]:
                    mask[loc0:loc1+1] = True
                else:
                    # extraxt section
                    values_interp_section = values_interp[loc0:loc1+1]
                    mask_section = mask[loc0:loc1+1]
                    # middle frames
                    values_section_diff = np.append(np.append(0, np.where(np.diff(np.round(values_interp_section)))[0]+1), len(values_interp_section))
                    values_section_diff_endpoints = np.row_stack([
                        values_section_diff[:-1], values_section_diff[1:],
                    ])
                    diff_ind = np.round(
                        (values_section_diff_endpoints[0,:]
                        + values_section_diff_endpoints[1,:] + 1) / 2.0
                    ).astype("int")
                    mask_section[diff_ind[1:-1]] = True
                    # endpoints frame
                    mask_section[0] = True
                    mask_section[-1] = True
            return mask
        # padding constant data for single frame
        elif self._camera_data.get_frame_num() == 1:
            return np.zeros(len(values_interp), dtype="bool")
        # remain default value if 0 frame
        else:
            return np.array([])


class CameraTracer(object):

    @staticmethod
    def trace_bone(camera_interp_data, bone_full_interp_data):
        # convert camera motion from camera local frame to global frame
        camera_local_motion = np.zeros_like(camera_interp_data.positions)
        camera_local_motion[:, 0:2] = camera_interp_data.positions[:, 0:2]
        camera_motion = Transform.rotate_vector(
            Transform.convert_mmd_euler_angles_to_quaternion(
                camera_interp_data.orientations.T
            ),
            camera_local_motion.T
        ).T
        # align bone data length with camera data length
        if bone_full_interp_data.get_frame_num() > camera_interp_data.frame_ids[-1]:
            bone_motion = bone_full_interp_data.positions[camera_interp_data.frame_ids]
        else:
            bone_motion_padding = np.zeros([camera_interp_data.frame_ids[-1]+1, 3])
            bone_motion_padding[:bone_full_interp_data.get_frame_num()] = bone_full_interp_data.positions
            bone_motion_padding[bone_full_interp_data.get_frame_num():] = bone_full_interp_data.positions[-1]
            bone_motion = bone_motion_padding[camera_interp_data.frame_ids]
        # camera total motion for tracing bone
        camera_total_motion = bone_motion + camera_motion
        return camera_total_motion

    @classmethod
    def add_camera_shake(
            cls, camera_interp_data,
            camera_shake_interval=1.0, camera_shake_amplitude=0.0,
        ):
        shake_local_motion = np.zeros_like(camera_interp_data.positions)
        shake_local_motion[:, 0:2] = cls._gen_2d_shake_motion(
            camera_interp_data.frame_ids, camera_shake_interval,
            camera_shake_amplitude,
        )
        shake_motion = Transform.rotate_vector(
            Transform.convert_mmd_euler_angles_to_quaternion(
                camera_interp_data.orientations.T
            ),
            shake_local_motion.T
        ).T
        camera_motion_with_shake = camera_interp_data.positions + shake_motion
        return camera_motion_with_shake

    @staticmethod
    def _gen_2d_shake_motion(frame_ids, camera_shake_interval, camera_shake_amplitude):
        # calculate how many shake points are needed
        fps = 30.0
        frame_per_interval = camera_shake_interval * fps
        end_frame_id = frame_ids[-1]
        interval_num = np.ceil(end_frame_id / frame_per_interval) + 1
        shake_frame_ids = np.arange(interval_num) * frame_per_interval
        # generate random shake points from polar coordinate system
        mmd_length_unit_per_meter = 1.0 / 0.08
        camera_shake_amplitude_std = camera_shake_amplitude / 3.0
        shake_points = (camera_shake_amplitude_std * mmd_length_unit_per_meter) \
            * np.random.randn(len(shake_frame_ids), 2)
        # smooth
        shake_motion = SmoothInterp.interp(shake_frame_ids, shake_points, frame_ids)
        return shake_motion


class SmoothInterp(object):

    @classmethod
    def interp(cls, frame_ids, values, frame_ids_desired):
        # f = scipy.interpolate.interp1d(frame_ids, values, kind="cubic", axis=0)
        f = scipy.interpolate.PchipInterpolator(frame_ids, values, axis=0)
        values_desired = f(frame_ids_desired)
        return values_desired
