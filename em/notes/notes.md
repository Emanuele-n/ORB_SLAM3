# Calibration
how to calibrate the camera
```
- python get_calibration_images.py
- python calibration.py
```
calibration.yaml will be saved in em folder


# ORB-SLAM
how to run ORB-SLAM3 with the calibration.yaml file
```
./Examples/Monocular/mono_realtime Vocabulary/ORBvoc.txt ./em/calibration_Misumi_200p.yaml
```

also change 
```
    cap.set(CAP_PROP_FRAME_WIDTH, 200); 
    cap.set(CAP_PROP_FRAME_HEIGHT, 200);
    cap.set(CAP_PROP_FPS, 30); 
```
in mono_realtime.cc to match the camera resolution and fps

# Real time pose
Currently the executable is sending the Tcw matrix to localhost:12345
This is received by the navigation.py 

# Things to try
- change parameters to avoid tracking lost and initialization: 
increase the number of features detected and decrease the threshold of ORBmatcher (ORBextractor.nFeature, ORBextractor.scaleFactor, ORBextractor.nLevels, ORBextractor.iniThFAST, and ORBextractor.minThFAST) ([issue](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/863), [issue](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/757), [issue](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/736))

- include encoder data to track linear advancement of the robot

- test with known map and perform localization only

- try without loop closure since it should be used inside lungs ( [issue](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/802))