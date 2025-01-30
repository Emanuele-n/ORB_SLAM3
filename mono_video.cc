#include <iostream>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "System.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include "External/ini.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./mono_realtime path_to_config.ini" << endl;
        return 1;
    }

    // Get the config file path
    string configFilePath = argv[1];
    mINI::INIFile file(configFilePath);
    mINI::INIStructure ini;
    file.read(ini);
    
    bool patient_data = false;
    bool use_encoder = false;
    bool use_viewer = false;
    string patient_data_path = "";
    
    if (ini["RUN"].get("patient") == "true") {
        patient_data = true;
        patient_data_path = ini["RUN"].get("patient_data");
    }

    if (ini["RUN"].get("encoder") == "true") {
        use_encoder = true;
    }

    if (ini["RUN"].get("viewer") == "true") {
        use_viewer = true;
    }

    string vocPath = ini["RUN"].get("vocabulary");
    string settingsPath = ini["RUN"].get("calibration");
    string videoPath = ini["RUN"].get("video");
    string logsPath = ini["RUN"].get("logs");

    // Initialize video source either from camera or from video file
    VideoCapture cap;
    cap.open(videoPath, cv::CAP_FFMPEG);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 3); // Set buffer size to reduce delay
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H', '2', '6', '4')); 
    
    if(!cap.isOpened())
    {
        cerr << "Failed to open video source" << endl;
        return -1;
    }

    // Count the number of frames in the video
    int numFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    int currentFrame = 0;

    // Vector to store the camera poses
    vector<Sophus::SE3f> trajectory;

    ORB_SLAM3::System SLAM(vocPath, settingsPath, ORB_SLAM3::System::MONOCULAR, use_viewer, patient_data, use_encoder, numFrames);

    cout << endl << "-------" << endl;
    cout << "Start processing camera input..." << endl;

    // // Desired frame rate (frames per second)
    // double desiredFPS = 15.0;
    // auto desiredFrameDuration = chrono::milliseconds(int(1000 / desiredFPS));

    Mat frame, resized_frame;
    // auto lastTime = chrono::high_resolution_clock::now();
    for(;;)
    {   
        currentFrame++;
        if (currentFrame == numFrames) {
            cout << "End of video" << endl;
            break;
        }
        
        // auto currentTime = chrono::high_resolution_clock::now();
        // chrono::duration<double> elapsed = currentTime - lastTime;
        // if (elapsed >= desiredFrameDuration) {
            cap >> frame; // get a new frame from camera
            // Check if it is the end of the video
            if(frame.empty())
            {
                cerr << "Failed to capture image" << endl;
                break;
            }
            // Resize the frame manually
            // resize(frame, resized_frame, Size(640, 360), 0, 0, INTER_LINEAR);
            
            double tframe = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now().time_since_epoch()).count();

            // auto start = chrono::steady_clock::now();
            // Sophus::SE3f Tcw = SLAM.TrackMonocular(resized_frame, tframe);
            Sophus::SE3f Tcw = SLAM.TrackMonocular(frame, tframe);
            trajectory.push_back(Tcw);
            // auto end = chrono::steady_clock::now();
            // auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
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
        // }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save the camera trajectory (invert all poses)
    std::string videoFileName = videoPath.substr(videoPath.find_last_of("/\\") + 1);
    size_t dotPos = videoFileName.find_last_of(".");
    std::string baseName = (dotPos == std::string::npos) ? videoFileName : videoFileName.substr(0, dotPos);
    std::string outFilename = logsPath + "/trajectory_" + baseName;
    ofstream f;
    f.open(outFilename.c_str());
    f << fixed;

    for(size_t i=0; i<trajectory.size(); i++)
    {
        Sophus::SE3f Twc = trajectory[i].inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << setprecision(9)
        << twc(0) << ", " << twc(1) << ", " << twc(2) << ", "
        << Rwc(0,0) << ", " << Rwc(0,1) << ", " << Rwc(0,2) << ", "
        << Rwc(1,0) << ", " << Rwc(1,1) << ", " << Rwc(1,2) << ", "
        << Rwc(2,0) << ", " << Rwc(2,1) << ", " << Rwc(2,2) << endl;
    }
    cout << "Camera trajectory saved to " << outFilename << endl;
    f.close();

    return 0;
}
