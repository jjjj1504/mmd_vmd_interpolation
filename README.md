# mmd_vmd_interpolation

A tool written in Python to

* make camera motion smoother.
* add camera shaking motion.
* make "exporting 'bone tracing' camera data" possible.

# Demonstration

The camera motion processed by this tool is applied in MMD videos belows

* 【Azur Lane MMD】Formidable - Otome Dissection (乙女解剖)【Camera DL】【1080p】  
  https://youtu.be/jAezBPdKSWU?t=70  

  [![](https://markdown-videos-api.jorgenkh.no/youtube/jAezBPdKSWU)](https://youtu.be/jAezBPdKSWU?t=70)

  > tracing groove (グルーブ) bone.

* 【MMD】ラヴィット / Loveit - カルも式初音ミク【Camera DL】【1080p】  
  https://youtu.be/-bQIrVkLFJw?t=6  
  [![](https://markdown-videos-api.jorgenkh.no/youtube/-bQIrVkLFJw)](https://youtu.be/-bQIrVkLFJw?t=6)

  > tracing head (頭) bone.

# Motivation

* Want to export "bone tracing" camera data, which follows the position of desired
  bone of dancing model, with smooth transition during using [MikuMikuDance (MMD)].
* In MMD, however,
  + When let camera follow certain bone of dancing model in camera mode,
    camera traces not only bone position but orientation,
    which will cause that camera view is swung along bone rotation.
  + While exporting "bone tracing" camera data to vmd file,
    the information of the bone traced by camera isn't exported to vmd file together,
    which will cause that the camera view loaded from that vmd file into MMD
    doesn't meet the original camera view.
* [カメラもまろやかMMD] and [カメラもぬるぬるMMD], developed by [みなみむき],
  is able to process the camera motion data to "bone tracing" camera data.
  Nevertheless,
  + it doesn't support successive transformation.
    Briefly speaking, it might be able to correctly trace center (センター) bone
    but not able to trace upper body (上半身) bone or head (頭) bone.
  + it supports constant position shift only.

[MikuMikuDance (MMD)]: (https://en.wikipedia.org/wiki/MikuMikuDance)
[カメラもまろやかMMD]: https://nico.ms/sm17818591
[カメラもぬるぬるMMD]: https://nico.ms/sm13825435
[みなみむき]: https://web.archive.org/web/20211006150449/http://ch.nicovideo.jp/min/blomaga/ar109650

# Solution

* Calculate the bone position in dancing motion vmd file,
  and export bone position without orientation to "nonrotatable bone motion" vmd file.
  In this way, camera view won't be swung along bone rotation
  while we let camera follow certain bone of dancing model in mmd camera mode.
* Calculate the "bone tracing" camera position from "camera position related to bone"
  vmd file (exported from MMD) and "nonrotatable bone motion" vmd file.

# Usage

* Convert dancing motion vmd file to "nonrotatable bone motion" vmd file
  by `generate_nonrotatable_bones_data.py`
* Open MMD
  + Load [`nonrotatable_center_bone.pmx`],
    and apply "nonrotatable bone motion" vmd file to it.
  + Register camera motion which following the bone of
    `nonrotatable_center_bone.pmx`.
  + Export the camera data to vmd file.
* Convert the vmd file of camera data and nonrotatable bone motion
  to "bone tracing" camera vmd file by `generate_bone_tracing_camera_data.py`.

[`nonrotatable_center_bone.pmx`]: https://bowlroll.net/file/298937

# Dependency

* Python 3
* NumPy
* SciPy
