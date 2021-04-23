#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;

void readCameraParameters(cv::Mat cameraMatrix, cv::Mat distCoeff){
    std::cout << "1\n";
    double data1[9] = {686.7657535590566, 0, 326.0317535676435, 0, 685.3640361518685, 241.6442161756605, 0, 0, 1};
    double data2[5] = {0.08043360996652632, -0.2211039472166585, -0.002918644434508041, 0.004494523281025384, 0};
    std::cout << "2\n";
    for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) =data1[i*3+j];
            }
        }
    std::cout << "2.5\n";

    for (int i=0; i<5; i++) {
        distCoeff.at<double>(0,i) = data2[i];
    }
    std::cout << "3\n";
}

int main(int argc, char** argv )
{   

    cv::VideoCapture cap;
    if(!cap.open(0))
        return 0;
    std::cout << "Abriu a câmera" << std::endl;
    cv::Mat cameraMatrix(3,3, CV_64F), distCoeffs(1,5, CV_64F);
    // camera parameters are read from somewhere
    readCameraParameters(cameraMatrix, distCoeffs);
    std::cout << "camera matriz : \n" << cameraMatrix << std::endl;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Mat image, imageCopy;
    while (true) {
        cap >> image;
        image.copyTo(imageCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        // if at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i<ids.size(); i++)
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
        cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(30);
        if (key == 27)
            break;
    }
}

