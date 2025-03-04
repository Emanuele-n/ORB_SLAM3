"""
Script to prepare logs for evaluation with rpg_trajectory_evaluation:
INPUT: 
- platform
- algorithm
- slam logs folder

The datasets under results are organized as

<platform>
├── <alg1>
│   ├── <platform>_<alg1>_<dataset1>
│   ├── <platform>_<alg1>_<dataset2>
│   └── ......
└── <alg2>
│   ├── <platform>_<alg2>_<dataset1>
│   ├── <platform>_<alg2>_<dataset2>
    ├── ......
......


Each sub-folder is of the same format as mentioned above. 
You need to specify the algorithms and datasets to analyze for the script analyze_trajectories.py. 
We use a configuration file under scripts/analyze_trajectories_config to sepcify the details. 
For example, in euroc_vio_mono_stereo.yaml

Datasets:
  MH_01:        ---> dataset name
    label: MH01 ---> plot label for the dataset
  MH_03:
    label: MH03
  MH_05:
    label: MH05
  V2_01:
    label: V201
  V2_02:
    label: V202
  V2_03:
    label: V203
Algorithms:
  vio_mono:         ---> algorithm name
    fn: traj_est    ---> estimation type to find the correct file name
    label: vio_mono ---> plot label for the algorithm
  vio_stereo: 
    fn: traj_est
    label: vio_stereo
RelDistances: []   ---> used to specify the sub-trajectory lengths in the relative error, see below.
RelDistancePercentages: []

will analyze the following folders

├── vio_mono
│   ├── laptop_vio_mono_MH_01
│   ├── laptop_vio_mono_MH_03
│   ├── laptop_vio_mono_MH_05
│   ├── laptop_vio_mono_V2_01
│   ├── laptop_vio_mono_V2_02
│   └── laptop_vio_mono_V2_03
└── vio_stereo
    ├── laptop_vio_stereo_MH_01
    ├── laptop_vio_stereo_MH_03
    ├── laptop_vio_stereo_MH_05
    ├── laptop_vio_stereo_V2_01
    ├── laptop_vio_stereo_V2_02
    └── laptop_vio_stereo_V2_03



So the result will be a new folder with the following structure:
<platform>
├── <alg>
│   ├── <platform>_<alg>_<dataset1>
│   ├── <platform>_<alg>_<dataset2>
│   └── ......

each dataset will be named after the branch name taken under the <slam_logs> folder.
Find all the folders named in the format record_bXXX_TIMESTAMP
take all the branch names "bXXX" and create a new folder with the same name under the <platform>_<alg>_<dataset> folder.

For each folder record_bXXX_TIMESTAMP, under the logs folder, there will be files in the format: CameraTrajectory_TIMESTAMP.txt and KeyFrameTrajectory_TIMESTAMP.txt
While the ground truth is in under the folder gt and the file gt_wTc.txt (consider if it's needed to set the timestammp )


"""
