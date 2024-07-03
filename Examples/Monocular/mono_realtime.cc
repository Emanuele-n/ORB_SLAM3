#include <iostream>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "System.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_live path_to_vocabulary path_to_settings [path_to_video]" << endl;
        return 1;
    }

    // Initialize video source either from camera or from video file
    VideoCapture cap;
    if (argc == 4) {
        cap.open(argv[3]); // open the video file
    } else {
        cap.open(2); // change to 0 if you want to use the default camera
        cap.set(CAP_PROP_FRAME_WIDTH, 200);
        cap.set(CAP_PROP_FRAME_HEIGHT, 200);
        cap.set(CAP_PROP_FPS, 30);
    }

    if(!cap.isOpened())
    {
        cerr << "Failed to open video source" << endl;
        return -1;
    }

    // Test to get one frame
    Mat test_frame;
    cap >> test_frame;
    if(test_frame.empty())
    {
        cerr << "Failed to capture image!" << endl;
        return -1;
    }
    cout << "Test frame captured" << endl;
    cout << "Frame size: " << test_frame.size() << endl;
    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cout << endl << "-------" << endl;
    cout << "Start processing camera input..." << endl;

    Mat frame;
    for(;;)
    {
        cap >> frame; // get a new frame from camera
        if(frame.empty())
        {
            cerr << "Failed to capture image!" << endl;
            break;
        }

        double tframe = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now().time_since_epoch()).count();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(frame, tframe);

        // Exit if ESC key is pressed
        if(waitKey(30) >= 0) break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
