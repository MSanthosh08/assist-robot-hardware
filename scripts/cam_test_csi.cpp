#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>   // for system()
#include <thread>
#include <chrono>

int main()
{
    std::cout << "🔄 Restarting nvargus-daemon..." << std::endl;

    int ret = system("sudo systemctl restart nvargus-daemon");

    if (ret != 0)
    {
        std::cerr << "❌ Failed to restart nvargus-daemon" << std::endl;
        return -1;
    }

    // ⏳ Give Argus time to come up
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ================= CAMERA PIPELINES =================

    std::string pipeline_left =
        "nvarguscamerasrc sensor-id=0 maxperf-enable=1 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=1";

    std::string pipeline_right =
        "nvarguscamerasrc sensor-id=1 maxperf-enable=1 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=1";

    cv::VideoCapture cap_left(pipeline_left, cv::CAP_GSTREAMER);

    // 🔥 CRITICAL: delay before opening second camera
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    cv::VideoCapture cap_right(pipeline_right, cv::CAP_GSTREAMER);

    if (!cap_left.isOpened() || !cap_right.isOpened())
    {
        std::cerr << "❌ Could not open one or both cameras!" << std::endl;
        return -1;
    }

    cv::Mat frameL, frameR, combined;

    while (true)
    {
        if (!cap_left.read(frameL) || !cap_right.read(frameR))
        {
            std::cerr << "❌ Frame read failed!" << std::endl;
            break;
        }

        cv::flip(frameL, frameL, 0);
        cv::flip(frameR, frameR, 0);

        cv::resize(frameL, frameL, cv::Size(320, 240));
        cv::resize(frameR, frameR, cv::Size(320, 240));

        cv::hconcat(frameL, frameR, combined);
        cv::imshow("Stereo Cameras", combined);

        if (cv::waitKey(1) == 'q')
            break;
    }

    cap_left.release();
    cap_right.release();
    cv::destroyAllWindows();

    return 0;
}
