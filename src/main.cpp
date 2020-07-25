#include <iostream>
#include <ostream>
#include <ctime>
#include <vector>
#include <unistd.h>

// Basler Stuff
#include <GenApi/GenApi.h>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

// OpenCV stuff
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace GenApi;
using namespace Pylon;

void calcBoardCornerPos(cv::Size boardSize, double squareSize, std::vector<cv::Point3f>& corners);
void calibratePoints(cv::Size boardSize, double squareSize, cv::Size imageSize, std::vector<std::vector<cv::Point2f>> foundPoints);

int main(int argc, char** argv) {
    double squareSize = 20;
    cv::Size boardSize(11, 7);
    int numFrames = 50;

    int c;
    while ((c = getopt(argc, argv, "s:x:y:f:h")) != -1) {
        switch (c) {
        case 's':
            squareSize = atof(optarg);
            break;
        case 'x':
            boardSize.width = atoi(optarg);
            break;
        case 'y':
            boardSize.height = atoi(optarg);
            break;
        case 'f':
            numFrames = atoi(optarg);
            break;
        case 'h':
            std::cerr << "usage: " << argv[0] << " [options]" << std::endl;
            std::cerr << "Options:" << std::endl;
            std::cerr << std::left << std::setw(10) << " -s" << " size of square in mm" << std::endl;
            std::cerr << std::left << std::setw(10) << " -x" << " chessboard columns" << std::endl;
            std::cerr << std::left << std::setw(10) << " -y" << " chessboard rows" << std::endl;
            std::cerr << std::left << std::setw(10) << " -f" << " calibration frames" << std::endl;
            return 0;
        case '?':
            if (isprint (optopt)) {
                fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            }
            else {
                fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
            }
            return 1;
        default:
            abort ();
        }
    }

    Pylon::PylonAutoInitTerm autoInitTerm;

    CTlFactory& TlFactory = CTlFactory::GetInstance();
    CDeviceInfo di;
    di.SetFriendlyName("CameraLeft (40022599)");
    CBaslerUniversalInstantCamera camera(TlFactory.CreateDevice(di));

    // Print the model name of the camera.
    std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;

    std::cout << "Square Size: " << squareSize << " mm" << std::endl;
    std::cout << "Board Size: " << boardSize.width << " x " << boardSize.height << std::endl;
    std::cout << "Calibration frames: " << numFrames << std::endl << std::endl;

    camera.Open();
    INodeMap& nodemap = camera.GetNodeMap();
    CEnumParameter(nodemap, "PixelFormat").SetValue("Mono8");
    camera.CenterX.SetValue(true);
    camera.CenterY.SetValue(true);
    CBooleanParameter(nodemap, "AcquisitionFrameRateEnable").SetValue(true);
    CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(5.0);
    camera.Close();

    camera.StartGrabbing(GrabStrategy_LatestImageOnly);
    CGrabResultPtr ptrGrabResult;

    std::vector<std::vector<cv::Point2f>> foundPoints;

    int height;
    int width;

    cv::namedWindow("Camera", 0);
    while (camera.IsGrabbing() && (foundPoints.size() < numFrames)) {
        camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded()) {
            height = ptrGrabResult->GetHeight();
            width = ptrGrabResult->GetWidth();

            cv::Mat imageMat(height, width, CV_8UC1, (uint8_t *) ptrGrabResult->GetBuffer());
            std::vector<cv::Point2f> chessboardPoints;

            bool foundBoard = cv::findChessboardCorners(imageMat, boardSize, chessboardPoints, cv::CALIB_CB_FAST_CHECK);
            if (foundBoard) {
                cv::drawChessboardCorners(imageMat, boardSize, chessboardPoints, foundBoard);
                cv::cornerSubPix(imageMat, chessboardPoints, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ));

                foundPoints.push_back(chessboardPoints);
                std::cout << "Found frame, total : " << foundPoints.size() << std::endl;
            }

            cv::imshow("Camera", imageMat);
            cv::resizeWindow("Camera", 600, 600);
            cv::waitKey(1);
        }
    }

    std::cout << "---" << std::endl;
    std::cout << "Done collecting points" << std::endl;
    std::cout << "Calibrating ..." << std::endl;

    calibratePoints(boardSize, squareSize, cv::Size(width, height), foundPoints);

    return 0;
}

void calibratePoints(cv::Size boardSize, double squareSize, cv::Size imageSize, std::vector<std::vector<cv::Point2f>> foundPoints) {
    std::vector<cv::Mat> rvecs, tvecs;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Mat stdDeviationsIntrinsics;
    cv::Mat stdDeviationsExtrinsics;
    cv::Mat perViewErrors;

    std::string outputPath = "./calibration.xml";

    std::vector<std::vector<cv::Point3f>> objPoints(1);
    calcBoardCornerPos(boardSize, squareSize, objPoints[0]);
    objPoints.resize(foundPoints.size(), objPoints[0]);

    double reprojErr = cv::calibrateCamera(objPoints, foundPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors);

    std::cout << "---" << std::endl;
    std::cout << "Calibration done, RMS: " << reprojErr << std::endl;

    std::ostringstream commentString;

    time_t now;
    std::time(&now);
    commentString << "Calibration date: " << std::asctime(localtime(&now)) << std::endl;
    commentString << "Number of frames: " << foundPoints.size() << std::endl;
    commentString << "RMS: " << reprojErr << std::endl;

    cv::FileStorage fs;
    fs.open("./calibration.xml", cv::FileStorage::WRITE);
    fs.writeComment(commentString.str());
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    std::cout << "Saved to " << outputPath << std::endl;
}

void calcBoardCornerPos(cv::Size boardSize, double squareSize, std::vector<cv::Point3f>& corners) {
    corners.clear();

    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
        }
    }
}
