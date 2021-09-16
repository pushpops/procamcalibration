#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/timer.hpp>
#include <numeric>
#include "Eigen/Dense"
#include <typeinfo>
#include <cmath>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#define PAT_ROW    (7)          /* パターンの行数 */
#define PAT_COL    (10)         /* パターンの列数 */
#define PAT_SIZE   (PAT_ROW*PAT_COL)
#define CHESS_SIZE (23)       /* パターン1マスの1辺サイズ[mm]23 */

using namespace std;
using namespace Eigen;

class CalibrationParameters
{
private:
public:
    vector<cv::Mat> rvecs,tvecs; // 各ビューの回転ベクトルと並進ベクトル
    cv::Mat cam_mat; // 内部パラメータ行列
    cv::Mat dist_coefs; // 歪み係数
};

//市松模様系の画像の親クラス
class CheckerboardImage{
private:

public:
    //ベクトルによって複数のシーン(平面板の角度)をまとめて扱う
    vector<cv::Mat> distim;//歪んだ画像
    vector<cv::Mat> undistim;//歪んでいない画像
    //全シーンの<<特徴点座標>のリスト>
    vector<vector<cv::Point2f>> distpoints;
    vector<vector<cv::Point2f>> undistpoints;
    vector<vector<cv::Point3f>> ccspoints;
    vector<vector<cv::Point3f>> wcspoints;
    CheckerboardImage();
};

class Calibration
{
public:
    Calibration(string inputpath);
    void calibration();
    inline void pm(int i);
    void transformcamtopro();
    cv::Mat rotationMatrix2cossin(cv::Mat R);
    cv::Mat cossin2rotationMatrix(cv::Mat cossin);
    //cv::Mat cossin の内容は以下
    //cosx cosy cosz
    //sinx siny sinz
    string username;
    CalibrationParameters cameraparam;
    CalibrationParameters projectorparam;
    cv::Mat c2pR,c2pt;

private:
    string inputpath;
    cv::Size pattern_size = cv::Size2i(PAT_COL, PAT_ROW);
    unsigned int allpoints;
    vector<cv::Point3f> object; //3次元空間座標(平面板)
    //ベクトルのindexはシーンのアイデンティティである
    unsigned int imgnum; //画像数
    vector<cv::Mat> inputImages;//入力元画像
    CheckerboardImage capturedBoard; //カメラ画像に写った平面板パターン
    CheckerboardImage capturedProjectedImage;//カメラ画像に映った投影画像
    CheckerboardImage projectedImage;//投影画像(原画)
    vector<vector<cv::Point2f>> reprojectedPoints;

    cv::Mat calculateCamera2ProjectorTransform(CalibrationParameters * cparam,CalibrationParameters * pparam, unsigned int i);
    vector<vector<cv::Point3f>> transformWcs2Ccs(vector<vector<cv::Point3f>> inpoints, CalibrationParameters * param);
    vector<vector<cv::Point2f>> transformWcs2Ics(vector<vector<cv::Point3f>> inpoints, CalibrationParameters * param, string icspath, string ccspath);//歪みなし
    vector<vector<cv::Point3f>> transformIcs2Wcs(vector<vector<cv::Point2f>> inpoints, CalibrationParameters * param, string wcspath, string ccspath);
    vector<vector<cv::Point2f>> transformCamICS2ProICS(vector<vector<cv::Point2f>> inpoints, CalibrationParameters * cparam,
                                                          CalibrationParameters * pparam, string pcspath);
    vector<vector<cv::Point2f>> findCorners(vector<cv::Mat>);
    CheckerboardImage calibrateLens(CheckerboardImage im, CalibrationParameters * param, bool distflag);
    tuple<cv::Mat,cv::Mat> separateImage(unsigned int);
    void calculateError(vector<vector<cv::Point2f>> data,vector<vector<cv::Point2f>> truth);
    //drawPoint(inputpath,outputpath,points,color)  ex.) fuga/hoge1.jpg → path="fuga/hoge"
    void drawPoint(string src, string dst, vector<vector<cv::Point2f>> points, cv::Scalar dot_S);
};

#endif // CALIBRATION_H
