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

int initTCPConnection(const string& serverIP, int serverPort) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        cerr << "Error opening socket.\n";
        return -1;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(serverPort);

    if (inet_pton(AF_INET, serverIP.c_str(), &serv_addr.sin_addr) <= 0) {
        cerr << "Invalid address/ Address not supported.\n";
        return -1;
    }

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        cerr << "Connection Failed.\n";
        return -1;
    }

    return sockfd;
}

template<typename MatrixType>
std::string matrixToString(const MatrixType& matrix) {
    std::stringstream ss;
    ss << matrix;
    return ss.str();
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_live path_to_vocabulary path_to_settings [path_to_video]" << endl;
        return 1;
    }

    // Initialize the TCP connection
    string serverIP = "127.0.0.1";
    int serverPort = 12345;
    int sockfd = initTCPConnection(serverIP, serverPort);
    if (sockfd == -1) {
        cerr << "Failed to establish TCP connection." << endl;
        return 1;
    }

    // Initialize video source either from camera or from video file
    VideoCapture cap;
    if (argc == 4) {
        cap.open(argv[3]); // open the video file
    } else {
        cap.open(3); // change to 0 if you want to use the default camera
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

        // // Pass the image to the SLAM system
        // SLAM.TrackMonocular(frame, tframe);

        // Trying to print the camera pose
        Sophus::SE3f Tcw = SLAM.TrackMonocular(frame, tframe);
        // std::cout << "Camera pose: " << Tcw.matrix() << std::endl;

        // Eigen::Vector3f position = Tcw.translation();  // Position in world coordinates
        // Eigen::Quaternionf orientation = Tcw.unit_quaternion();  // Orientation as a quaternion 

        // // Print or log the position and orientation
        // std::cout << "Position: " << position.transpose() << std::endl;
        // std::cout << "Orientation (Quaternion): " << orientation.coeffs().transpose() << std::endl;

        // Send the camera pose to the server
        auto matrix = Tcw.matrix();
        std::string data = matrixToString(matrix);
        std::string message = "Tcw: " + data + "\n";
        if (send(sockfd, message.c_str(), message.length(), 0) < 0) {
            std::cerr << "Failed to send data.\n";
        }

        // Exit if ESC key is pressed
        if(waitKey(30) >= 0) break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
