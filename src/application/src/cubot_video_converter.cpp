//
// Created by plutoli on 2022-06-05.
//

#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    // 创建视频宽度/高度/帧率
    int width;
    int height;
    double fps;

    // 初始化视频并打开视频捕捉器
    cv::VideoCapture videoCapture;
    std::string sourceVideoFile = "/home/plutoli/data/test.avi";
    if (videoCapture.open(sourceVideoFile))
    {
        width = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
        height = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
        fps = videoCapture.get(cv::CAP_PROP_FPS);
    }
    else
    {
        return -1;
    }

    // 初始化并打开视频录制器
    cv::VideoWriter videoWriter;
    std::string destVideoFile = "/home/plutoli/data/test3.avi";
    if (!videoWriter.open(destVideoFile,
                          cv::VideoWriter::fourcc('M','J','P','G'),
                          fps,
                          cv::Size(width, height),
                          true))
    {
        return -1;
    }

    // 输出提示信息
    std::cout << "VideoConverter is working......" << std::endl;

    // 循环处理相机数据
    cv::Mat image;
    while (videoCapture.read(image))
    {
        videoWriter.write(image);
    }

    // 输出提示信息
    std::cout << "VideoConverter works completely" << std::endl;

    // 释放视频捕捉器和视频录制器
    videoWriter.release();
    videoCapture.release();

    return 0;
}