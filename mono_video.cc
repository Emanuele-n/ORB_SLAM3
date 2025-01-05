#include <iostream>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "System.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_realtime path_to_vocabulary path_to_settings path_to_video [-patient=true] [-encoder=true]" << endl;
        return 1;
    }

    bool patient_data = false;
    bool use_encoder = false;
    string CADPath = "";
    string centerlinePath = "";
    
    // Parse command line arguments
    for(int i = 4; i < argc; i++) {
        string arg = argv[i];
        if(arg == "-patient=true") {
            patient_data = true;
        }
        else if(arg == "-encoder=true") {
            use_encoder = true;
        }
    }

    // Initialize video source either from camera or from video file
    VideoCapture cap;
    cap.open(argv[3], cv::CAP_FFMPEG);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 3); // Set buffer size to reduce delay
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H', '2', '6', '4')); 
    

    if(!cap.isOpened())
    {
        cerr << "Failed to open video source" << endl;
        return -1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, patient_data, use_encoder);

    cout << endl << "-------" << endl;
    cout << "Start processing camera input..." << endl;

    // Desired frame rate (frames per second)
    double desiredFPS = 15.0;
    auto desiredFrameDuration = chrono::milliseconds(int(1000 / desiredFPS));

    Mat frame, resized_frame;
    auto lastTime = chrono::high_resolution_clock::now();
    for(;;)
    {   
        auto currentTime = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = currentTime - lastTime;
        if (elapsed >= desiredFrameDuration) {
            cap >> frame; // get a new frame from camera
            if(frame.empty())
            {
                cerr << "Failed to capture image!" << endl;
                break;
            }
            // Resize the frame manually
            resize(frame, resized_frame, Size(640, 360), 0, 0, INTER_LINEAR);
            
            double tframe = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now().time_since_epoch()).count();

            auto start = chrono::steady_clock::now();
            Sophus::SE3f Tcw = SLAM.TrackMonocular(resized_frame, tframe);
            auto end = chrono::steady_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
            // cout << "TrackMonocular: " << duration.count() << " milliseconds" << endl;
            // Write duration to log.txt
            // ofstream logFile("log.txt", ios_base::app);
            // if (logFile.is_open()) {
            //     logFile << "TrackMonocular duration: " << duration.count() << " milliseconds" << endl;
            //     logFile.close();
            // } else {
            //     cerr << "Unable to open log file" << endl;
            // }
            // std::cout << "Camera pose: " << Tcw.matrix() << std::endl;

            // Exit if ESC key is pressed
            if(waitKey(30) >= 0) break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
