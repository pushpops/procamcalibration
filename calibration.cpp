#include <calibration.h>

using namespace std;
namespace fs = boost::filesystem;

Calibration::Calibration(string inputpath)
{
    this->inputpath = inputpath;
    imgnum = 0;
    unsigned int i,j,k;

    struct passwd *pw = getpwuid(getuid());
    username = string(pw->pw_name);

    vector<string> dirs = {"3Dto2DCam","2DCamto3D","undistort","findCorners","projectPoints","undistortPoints"};
    vector<string> dirs1 = {"mkdir -p "+inputpath+"/calib_output/errorData/",
                            "mkdir -p "+inputpath+"/calib_output/board/",
                            "mkdir -p "+inputpath+"/calib_output/projected/",
                            "mkdir -p "+inputpath+"/calib_output/projected/3Dto2DPro",
                            "mkdir -p "+inputpath+"/calib_output/reprojectionC2P"
                           };
    for(i=0; i<dirs1.size(); i++)
    {
        system(dirs1[i].c_str());
    }
    for(i=0; i<dirs.size(); i++)
    {
        system((dirs1[1]+dirs[i]).c_str());
        system((dirs1[2]+dirs[i]).c_str());
    }
    ofstream board(inputpath+"/calib_output/board/board.txt");

    // 3次元空間座標の設定
    for (j = 0; j < PAT_ROW; j++)
    {
        for (k = 0; k < PAT_COL; k++)
        {
            cv::Point3f p(
                j * CHESS_SIZE,
                k * CHESS_SIZE,
                0.0);
            object.push_back(p);
            board << p << endl;
        }
    }

}

void Calibration::calibration()
{
    //BGR color for drawPoint()
    cv::Scalar GREEN(0, 255, 0);
    cv::Scalar YELLOW(0,250,240);
    cv::Scalar BLUE (255,0,0);
    cv::Scalar RED (0,0,255);
    cv::Scalar ORANGE (0,128,255);

    // オリジナル投影画像の読み込み
    cout << "reading original projection image..." << endl;
    string projectpath;
    projectpath = "/Users/"+username+"/Desktop/chessboard/cchessboard.PNG";
    cv::Mat projectedim = cv::imread(projectpath);

    cout << "end" << endl;

    //指定フォルダ内のカメラ画像読み込み
    cout << "reading checkerboard images..." << endl;
    cout << "inputpath: " << inputpath << endl;
    vector<string> filenames;
    fs::directory_iterator end;
    cv::Mat tmpimg;
    unsigned int i,j;
    for( fs::directory_iterator it( inputpath ); it != end; ++it )
    {
        if( !fs::is_directory( *it ) ){ //ディレクトリでない場合
            tmpimg = cv::imread(it->path().string());
            if ( !tmpimg.empty() ){ //パスのファイルが画像ファイルである場合
                filenames.push_back(inputpath+"/"+it->path().filename().string());
            }
        }
    }
    imgnum = static_cast<unsigned int>(filenames.size());
    sort(filenames.begin(), filenames.end());

    //imgnum = ; //先頭から必要な枚数だけの画像を用いる場合
    //filenames.assign(&filenames[0], &filenames[imgnum]);//先頭から必要な枚数だけの画像を用いる場合

    cout << "imgnum: " << imgnum << endl;

    for (i=0;i<filenames.size();i++) {
        cout << filenames[i] << endl;
        tmpimg = cv::imread(filenames[i]);
        cv::resize(tmpimg,tmpimg,cv::Size(640,480));
        inputImages.push_back(tmpimg);
        //R,Bプレーンで分離してそれぞれ保存する
        capturedBoard.distim.push_back(get<0>(separateImage(i)));
        capturedProjectedImage.distim.push_back(get<1>(separateImage(i)));
    }
    cout << "end" << endl;
    for (i = 0; i < imgnum; i++)
    {
       capturedBoard.wcspoints.push_back(object);
       projectedImage.undistim.push_back(projectedim);
       projectedImage.distim.push_back(projectedim);
    }

    //コーナー検出
    capturedBoard.distpoints = findCorners(capturedBoard.distim);
    capturedProjectedImage.distpoints = findCorners(capturedProjectedImage.distim);
    projectedImage.undistpoints = findCorners(projectedImage.distim);
    projectedImage.distpoints = projectedImage.undistpoints;

    //カメラキャリブレーション
    capturedBoard = calibrateLens(capturedBoard, &cameraparam, 1);

    ofstream fs(inputpath+"/calib_output/camera.txt");
    fs << "intrinsic" << endl;
    fs << cameraparam.cam_mat << endl << endl;
    fs << "distortion" << endl;
    fs << cameraparam.dist_coefs << endl << endl;
    for (i = 0; i < imgnum; i++)
    {
        fs << "rvec: IMG " << i+1 << endl;
        fs << cameraparam.rvecs[i] << endl << endl;
        fs << "tvec: IMG " << i+1 << endl;
        fs << cameraparam.tvecs[i] << endl << endl;
    }
    fs.close();

    //--------------------------------
    // カメラ歪み補正をcaptured board, projected imに施す
    //--------------------------------
    cout << "undistortion..." << endl;
    cv::Mat dst;
    for (i = 0; i < imgnum; i++)
    {
        //歪み補正後の画像
        cv::undistort(capturedBoard.distim[i],dst,cameraparam.cam_mat,cameraparam.dist_coefs);
        capturedBoard.undistim.push_back(dst);
        cv::imwrite(inputpath+"/calib_output/board/undistort/undst"+to_string(i+1)+".jpg", capturedBoard.undistim[i]);

        cv::undistort(capturedProjectedImage.distim[i],dst,cameraparam.cam_mat,cameraparam.dist_coefs);
        capturedProjectedImage.undistim.push_back(dst);
        cv::imwrite(inputpath+"/calib_output/projected/undistort/undst"+to_string(i+1)+".jpg", capturedProjectedImage.undistim[i]);

        //歪み補正後の座標点
        cv::Mat_<float>::const_iterator cit;
        cv::Mat pointMat;
        cv::Mat r;
        cv::Point2f undistpoint;
        vector<cv::Point2f> undistpointsB,undistpointsP;

        ofstream ICSboardUndist(inputpath+"/calib_output/board/undistortPoints/ICSboardUndist"+to_string(i+1)+".txt");
        ofstream capProjcorners(inputpath+"/calib_output/projected/findCorners/ICSprojected"+to_string(i+1)+".txt");
        ofstream capProjcornersUndist(inputpath+"/calib_output/projected/undistortPoints/ICSprojectedUndist"+to_string(i+1)+".txt");
        ofstream ICSboard(inputpath+"/calib_output/board/findCorners/ICSboard"+to_string(i+1)+".txt");
        for (j = 0; j < PAT_SIZE; j++)
        {
            ICSboard << capturedBoard.distpoints[i][j] << endl;
            //画像に写った平面板のコーナー位置の歪み補正
            cv::undistortPoints(cv::Mat(capturedBoard.distpoints[i][j]),
                                pointMat,cameraparam.cam_mat,
                                cameraparam.dist_coefs,r,
                                cameraparam.cam_mat);//カメラ画像に写った印刷画像
            undistpoint.x = pointMat.at<float>(0,0);
            undistpoint.y = pointMat.at<float>(0,1);
            undistpointsB.push_back(undistpoint);
            ICSboardUndist << undistpoint << endl;

            capProjcorners << capturedProjectedImage.distpoints[i][j] << endl;
            //画像に写った投影画像のコーナー位置の歪み補正
            cv::undistortPoints(cv::Mat(capturedProjectedImage.distpoints[i][j]),
                                pointMat,cameraparam.cam_mat,
                                cameraparam.dist_coefs,r,
                                cameraparam.cam_mat);//カメラ画像に写った投影画像
            undistpoint.x = pointMat.at<float>(0,0);
            undistpoint.y = pointMat.at<float>(0,1);
            undistpointsP.push_back(undistpoint);
            capProjcornersUndist << undistpoint << endl;
        }
        capturedBoard.undistpoints.push_back(undistpointsB);
        capturedProjectedImage.undistpoints.push_back(undistpointsP);
        ICSboard.close();
        ICSboardUndist.close();
        capProjcorners.close();
        capProjcornersUndist.close();
    }
    cout << "end" << endl;

    //-------------------------------
    //  座標系変換
    //-------------------------------

    string ccspath;
    string icspath;
    string wcspath;

    icspath = inputpath+"/calib_output/board/3Dto2DCam/ICSboardUndist";
    ccspath = inputpath+"/calib_output/board/3Dto2DCam/CCSboard";
    vector<vector<cv::Point2f>> reprojectedIcs = transformWcs2Ics(capturedBoard.wcspoints, &cameraparam, icspath, ccspath);

    //画像に写った印刷画像の点についてICSからWCSに変換して平面板の物体座標と一致するか確かめる．
    // wcspath = "/Users/"+username+"/Desktop/calib_output/board/2DCamto3D/WCSboard";
    // ccspath = "/Users/"+username+"/Desktop/calib_output/board/2DCamto3D/CCSboard";
    // transformIcs2Wcs(capturedBoard.undistpoints,&cameraparam,wcspath,ccspath);
    // //-----------------------------------------------------------
    //画像に写った投影画像の点についてWCSに変換したあとICSに再投影する
    //-----------------------------------------------------------

    ccspath = inputpath+"/calib_output/projected/2DCamto3D/CCSprojected";
    wcspath = inputpath+"/calib_output/projected/2DCamto3D/WCSprojected";
    vector<vector<cv::Point3f>> projectedWcs = transformIcs2Wcs(capturedProjectedImage.undistpoints, &cameraparam, wcspath, ccspath);
    capturedProjectedImage.wcspoints = projectedWcs;
    icspath = inputpath+"/calib_output/projected/3Dto2DCam/ICSprojectedUndist";
    wcspath = inputpath+"/calib_output/projected/3Dto2DCam/WCSprojected";
    vector<vector<cv::Point2f>> reprojectedWcs = transformWcs2Ics(capturedProjectedImage.wcspoints, &cameraparam, icspath, ccspath);

    //-----------------------------------------------------------
    //プロジェクタキャリブレーション
    //-----------------------------------------------------------
    cout << "calibrate projector ..." << endl;
    projectedImage.wcspoints = capturedProjectedImage.wcspoints;
    projectedImage = calibrateLens(projectedImage, &projectorparam, 1);
    ofstream fs1(inputpath+"/calib_output/projector.txt");
    fs1 << "intrinsic" << endl;
    fs1 << projectorparam.cam_mat << endl << endl;
    fs1 << "distortion" << endl;
    fs1 << projectorparam.dist_coefs << endl << endl;
    for (i = 0; i < imgnum; i++)
    {
        fs1 << "rvec: IMG " << i+1 << endl;
        fs1 << projectorparam.rvecs[i] << endl << endl;
        fs1 << "tvec: IMG " << i+1 << endl;
        fs1 << projectorparam.tvecs[i] << endl << endl;
    }
    fs1.close();
    cout << "projector calibration end" << endl;

    //-----------------------------------------------------------
    // 再投影と誤差の計算
    //-----------------------------------------------------------

    string picspath = inputpath+"/calib_output/reprojectionC2P/onprojectionIM";
    reprojectedPoints = transformCamICS2ProICS(capturedProjectedImage.undistpoints,&cameraparam, &projectorparam, picspath);
    for (i=0;i<imgnum;i++)
    {
        cv::imwrite(picspath+to_string(i+1)+".jpg", projectedImage.undistim[i]);
    }

    cout << "drawPoint... go \"plot\" window and push any key" << endl;
    drawPoint(picspath,
              picspath,
              reprojectedPoints,
              BLUE
              );
    calculateError(reprojectedPoints, projectedImage.undistpoints);

    //---------------------------------------------------
    // カメラ->プロジェクタの回転，並進を計算してフィールドに保存
    //---------------------------------------------------

    cv::Mat cal;
    cv::Mat cossin = cv::Mat::zeros(2,3,CV_64F);
    cv::Mat allt = cv::Mat::zeros(3,1,CV_64F);
    for (i=0; i< imgnum; i++){
        cal = calculateCamera2ProjectorTransform(&cameraparam,&projectorparam,i);
        cossin += rotationMatrix2cossin(cv::Mat(cal, cv::Rect(0,0,3,3)));
        allt += (cv::Mat(cal,cv::Rect(3,0,1,3)));
    }
    cossin = cossin/imgnum;

    c2pR = cossin2rotationMatrix(cossin);
    c2pt = allt/imgnum;

    ofstream outfile1(inputpath+"/calib_output/c2pTransParam.txt");
    outfile1 << "Calibration" << endl;
    outfile1 << "c2pR:" << endl;
    outfile1 << c2pR << endl << endl;
    outfile1 << "alpha:" << acos(cossin.at<double>(0,0))*180/M_PI << endl;
    outfile1 << "beta:" << acos(cossin.at<double>(0,1))*180/M_PI << endl;
    outfile1 << "gamma:" << acos(cossin.at<double>(0,2))*180/M_PI << endl << endl;
    outfile1 << "c2pt:" << endl;
    outfile1 << c2pt << endl;
    outfile1.close();

    //--------------------------------------------
    //　パラメータをin0ファイルに保存
    //--------------------------------------------
    cout << "カメラ座標系からプロジェクタ座標系への変換(外部パラメータ)" << endl;
    cout << "回転行列" << endl;
    cout << c2pR << endl;
    cout << "並進行列" << endl;
    cout << c2pt << endl;

    cout << "Calibaration: all process ended !" << endl;
}

cv::Mat Calibration::rotationMatrix2cossin(cv::Mat R){
    //必ず鋭角であると想定
    double cosx,sinx,cosy,siny,cosz,sinz;
    siny = -R.at<double>(2,0);
    cosy = sqrt(1-siny*siny);
    cosz = R.at<double>(0,0)/cosy;
    sinz = R.at<double>(1,0)/cosy;
    cosx = R.at<double>(2,2)/cosy;
    sinx = R.at<double>(2,1)/cosy;
    cv::Mat cossin = (cv::Mat_<double>(2,3) << cosx, cosy, cosz, sinx, siny, sinz);
    return cossin;
}

cv::Mat Calibration::cossin2rotationMatrix(cv::Mat cossin){
    //必ず鋭角であると想定
    double cosx = cossin.at<double>(0,0);
    double sinx = cossin.at<double>(1,0);
    double cosy = cossin.at<double>(0,1);
    double siny = cossin.at<double>(1,1);
    double cosz = cossin.at<double>(0,2);
    double sinz = cossin.at<double>(1,2);

    cv::Mat R = (cv::Mat_<double>(3,3) <<
               cosy*cosz, -sinz*cosx+cosz*siny*sinx, sinz*sinx+cosz*siny*cosx,
               sinz*cosy, cosz*cosx+sinz*siny*sinx,  -cosz*sinx+sinz*siny*cosx,
               -siny,     cosy*sinx,                 cosy*cosx
               );
    return R;
}

cv::Mat Calibration::calculateCamera2ProjectorTransform(CalibrationParameters * cparam,CalibrationParameters * pparam, unsigned int i){
    vector<vector<cv::Point2f>> reproPoints;

    cparam->cam_mat.convertTo(cparam->cam_mat,CV_64F);
    pparam->cam_mat.convertTo(pparam->cam_mat,CV_64F);
    cv::Mat bottomhm =  (cv::Mat_<double>(1,4) << 0,0,0,1);
    cv::Mat sidehm = (cv::Mat_<double>(3,1) << 0,0,0);
    cv::Mat identity = (cv::Mat_<double>(3,3) << 1,0,0,
                                                 0,1,0,
                                                 0,0,1);

    cv::Mat hmtc,hmtp; //identityに-tをつけてbottomhmをつける
    cv::hconcat(identity,-cparam->tvecs[i],hmtc);
    cv::vconcat(hmtc,bottomhm,hmtc); 
    cv::hconcat(identity,pparam->tvecs[i],hmtp);
    cv::vconcat(hmtp,bottomhm,hmtp);

    cv::Mat Rct, Rc;
    cv::Rodrigues(cparam->rvecs[i], Rc);
    Rct = Rc.t();
    cv::Mat hmRct; //Rにsidehmをつけてからbottomhmをつける
    cv::hconcat(Rct,sidehm,hmRct);
    cv::vconcat(hmRct,bottomhm,hmRct);

    cv::Mat Rp;
    cv::Rodrigues(pparam->rvecs[i], Rp);
    cv::Mat hmRp; //Rにsidehmをつけてからbottomhmをつける
    cv::hconcat(Rp,sidehm,hmRp);
    cv::vconcat(hmRp,bottomhm,hmRp);

    cv::Mat cICS2pICS;
    cICS2pICS = hmtp*hmRp*hmRct*hmtc;
    return cICS2pICS;
}

vector<vector<cv::Point2f>> Calibration::transformCamICS2ProICS(vector<vector<cv::Point2f>> inpoints, CalibrationParameters * cparam,
                                                      CalibrationParameters * pparam, string pcspath){

    vector<vector<cv::Point2f>> reproPoints;
    cv::Mat pcs_point,ccs_point,wcs_point,pics_point;
    cv::Mat R,t;

    cv::Mat bottomhm =  (cv::Mat_<double>(1,4) << 0,0,0,1);
    cv::Mat sidehm = (cv::Mat_<double>(3,1) << 0,0,0);

    cparam->cam_mat.convertTo(cparam->cam_mat,CV_64F);
    pparam->cam_mat.convertTo(pparam->cam_mat,CV_64F);

    double fx = cparam->cam_mat.at<double>(0,0);
    double cx = cparam->cam_mat.at<double>(0,2);
    double fy = cparam->cam_mat.at<double>(1,1);
    double cy = cparam->cam_mat.at<double>(1,2);

    cv::Mat hmAc = (cv::Mat_<double>(4,4) << 1/fx,  0,      0, -cx/fx,
                                             0,     1/fy,   0, -cy/fy,
                                             0,     0,      1,      0,
                                             0,     0,      0,      1);
    cv::Mat hmAp = pparam->cam_mat;
    cv::hconcat(hmAp,sidehm,hmAp);
    cv::vconcat(hmAp,bottomhm,hmAp);

    unsigned int i,j;
    //平均の計算
    cv::Mat trans;
    cv::Mat cossin = cv::Mat::zeros(2,3,CV_64F);
    cv::Mat allt = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat cal;
    for(i=0; i<imgnum; i++){
        cal = calculateCamera2ProjectorTransform(cparam,pparam,i);
        cossin += rotationMatrix2cossin(cv::Mat(cal, cv::Rect(0,0,3,3)));
        allt += (cv::Mat(cal,cv::Rect(3,0,1,3)));
    }
    R = cossin2rotationMatrix(cossin/imgnum);
    t = allt/imgnum;

    cv::hconcat(R,t,trans);
    cv::vconcat(trans,bottomhm,trans);

    //変換
    cout << "transform..." << endl;
    for(i=0; i<imgnum; i++){
        vector<cv::Point2f> points;

        //平均を使わない場合
//        trans = calculateCamera2ProjectorTransform(cparam,pparam,i);

        //ファイル出力
        ofstream pcs(pcspath+to_string(i+1)+".txt");
        ofstream sthmAc("/Users/maru/Desktop/out/hmAc/hmAc"+to_string(i+1)+".txt");
        ofstream sthmzc("/Users/maru/Desktop/out/hmzc/hmzc"+to_string(i+1)+".txt");
        ofstream sthmtrans("/Users/maru/Desktop/out/hmtrans/hmtrans"+to_string(i+1)+".txt");
        ofstream sthmAp("/Users/maru/Desktop/out/hmAp/hmAp"+to_string(i+1)+".txt");
        ofstream sthmzp("/Users/maru/Desktop/out/hmzp/hmzp"+to_string(i+1)+".txt");

        for(j=0; j<PAT_SIZE; j++){
            cv::Point2f point;
            cv::Mat p = (cv::Mat_<double>(4,1));
            p.at<double>(0,0) = double(inpoints[i][j].x);
            p.at<double>(1,0) = double(inpoints[i][j].y);
            p.at<double>(2,0) = 1;
            p.at<double>(3,0) = 1;

            p = hmAc * p;
            sthmAc << p << endl;

            //zcを各点ごとに求める
            double tc1,tc2,tc3;
            double rc13,rc23,rc33;
            double f1,f2;
            cv::Mat R;
            cv::Rodrigues(cparam->rvecs[i],R);
            rc13 = double(R.at<double>(0,2));
            rc23 = double(R.at<double>(1,2));
            rc33 = double(R.at<double>(2,2));
            tc1 = cparam->tvecs[i].at<double>(0,0);
            tc2 = cparam->tvecs[i].at<double>(1,0);
            tc3 = cparam->tvecs[i].at<double>(2,0);
            f1 = p.at<double>(0,0);
            f2 = p.at<double>(1,0);
            double zc = (tc1*rc13+tc2*rc23+tc3*rc33)/(f1*rc13+f2*rc23+rc33);
            cv::Mat hmzc = (cv::Mat_<double>(4,4) << zc,  0,    0,    0,
                                                     0,   zc,   0,    0,
                                                     0,   0,    zc,   0,
                                                     0,   0,    0,    1);

            p = hmzc * p;
            sthmzc << p << endl;

            p = trans * p;
            sthmtrans << p << endl;

            p = (1/p.at<double>(2,0))*p;
            sthmzp << p << endl;

            p = hmAp * p;
            sthmAp << p << endl;

            point.x = float(p.at<double>(0,0));
            point.y = float(p.at<double>(1,0));
            pcs << point << endl;
            points.push_back(point);
          }
           sthmAc.close();
           sthmAp.close();
           sthmzc.close();
           sthmzp.close();
           sthmtrans.close();
           pcs.close();
           reproPoints.push_back(points);
    }

    return reproPoints;
}

vector<vector<cv::Point3f>> Calibration::transformWcs2Ccs(vector<vector<cv::Point3f>> inpoints, CalibrationParameters * param){
    //歪みなし
        unsigned int i,j;
        cv::Mat R_chessboard_cam;
        cv::Point3d point;
        cv::Mat ccspoint;
        cv::Mat wpoint;
        vector<vector<cv::Point3f>> ccs3Dpoints(imgnum);
        cv::Point2f p;
        vector<vector<double>> zc(imgnum);

        for (i=0; i < imgnum; i++) {
            cv::Rodrigues(param->rvecs[i], R_chessboard_cam);
            cv::Mat cam_mat;
            ofstream file("/Users/maru/Desktop/projected/tw2cfunc"+to_string(i+1)+".txt");
            for (j=0; j < PAT_SIZE;j++) {
                wpoint = cv::Mat(inpoints[i][j]);
                wpoint.convertTo(wpoint,CV_64F);
                //WCS to CCS
                ccspoint = R_chessboard_cam*wpoint+param->tvecs[i];
                file << ccspoint << endl;
                zc[i].push_back(ccspoint.at<double>(2,0));
                //Point型にする
                point.x = ccspoint.at<double>(0,0);
                point.y = ccspoint.at<double>(1,0);
                point.z = ccspoint.at<double>(2,0);

                ccs3Dpoints[i].push_back(point);
            }
        }
        return ccs3Dpoints;
}

vector<vector<cv::Point2f>> Calibration::transformWcs2Ics(
        vector<vector<cv::Point3f>> inpoints, CalibrationParameters * param, string icspath, string ccspath)
{   //歪みなし
    unsigned int i,j;
    cv::Mat R_chessboard_cam;
    cv::Point3d point;
    cv::Mat pointonim;
    cv::Mat wpoint;
    vector<vector<cv::Point2f>> manual2Dpoints(imgnum);
    cv::Point2f p;
    vector<vector<double>> zc(imgnum);

    for (i=0; i < imgnum; i++) {
        vector<cv::Point3d> points;
        ofstream ccs(ccspath+to_string(i+1)+".txt");
        ofstream ics(icspath+to_string(i+1)+".txt");
        cv::Rodrigues(param->rvecs[i], R_chessboard_cam);
        cv::Mat cam_mat;

        for (j=0; j < PAT_SIZE;j++) {
            wpoint = cv::Mat(inpoints[i][j]);
            wpoint.convertTo(wpoint,CV_64F);
            //WCS to CCS
            pointonim = R_chessboard_cam*wpoint+param->tvecs[i];
            zc[i].push_back(pointonim.at<double>(2,0));
            //Point型にしてファイルに出力
            point.x = pointonim.at<double>(0,0);
            point.y = pointonim.at<double>(1,0);
            point.z = pointonim.at<double>(2,0);
            ccs << point << endl;

            //CCS to ICS
            cam_mat = param->cam_mat;
            cam_mat.convertTo(cam_mat,CV_64F);
            pointonim = cam_mat * pointonim;
            //ポイント型にしてファイルに出力
            point.x = pointonim.at<double>(0,0)/zc[i][j];
            point.y = pointonim.at<double>(1,0)/zc[i][j];
            point.z = pointonim.at<double>(2,0)/zc[i][j];
            ics << point << endl;

            p.x = float(point.x);
            p.y = float(point.y);
            manual2Dpoints[i].push_back(p);
        }
        ccs.close();
        ics.close();
    }
    return manual2Dpoints;
}

vector<vector<cv::Point3f>> Calibration::transformIcs2Wcs(
        vector<vector<cv::Point2f>> inpoints, CalibrationParameters * param, string wcspath, string ccspath)
{
    unsigned int i,j;
    vector<vector<cv::Point3f>> reproPoints;
    param->cam_mat.convertTo(param->cam_mat,CV_32F);
    vector<vector<double>> zc(imgnum);

    double fx = double(param->cam_mat.at<float>(0,0));
    double cx = double(param->cam_mat.at<float>(0,2));
    double fy = double(param->cam_mat.at<float>(1,1));
    double cy = double(param->cam_mat.at<float>(1,2));
    double t1,t2,t3;
    double r31,r32,r33;
    double f1,f2;
    double xc,yc;

    cv::Mat cam_point,R_chessboard_cam;
    cv::Mat R_cam_chessboard;
     for (i=0; i<imgnum; i++){
         vector<cv::Point3f> points;
         ofstream wcs(wcspath+to_string(i+1)+".txt");
         ofstream ccs(ccspath+to_string(i+1)+".txt");
//         ofstream fzc("/Users/maru/Desktop/zc/zc"+to_string(i+1)+".txt");
         t1 = double(param->tvecs[i].at<double>(0,0));
         t2 = double(param->tvecs[i].at<double>(0,1));
         t3 = double(param->tvecs[i].at<double>(0,2));

         cv::Rodrigues(param->rvecs[i], R_cam_chessboard);
         R_chessboard_cam = R_cam_chessboard.t();
         r31 = double(R_chessboard_cam.at<double>(2,0));
         r32 = double(R_chessboard_cam.at<double>(2,1));
         r33 = double(R_chessboard_cam.at<double>(2,2));

         for (j = 0; j < PAT_SIZE; j++) {
             f1 = (double(inpoints[i][j].x)-cx)/fx;
             f2 = (double(inpoints[i][j].y)-cy)/fy;
             zc[i].push_back((t1*r31+t2*r32+t3*r33)/(f1*r31+f2*r32+r33));
//             fzc << zc[i][j] << endl;

             xc = f1*zc[i][j];
             yc = f2*zc[i][j];
             cam_point = (cv::Mat_<double>(3,1) << xc, yc, zc[i][j]);

             ccs << cam_point << endl;

             cv::Mat intersection_point = R_chessboard_cam*(cam_point-param->tvecs[i]);

             cv::Point3d point;
             point.x = intersection_point.at<double>(0,0);
             point.y = intersection_point.at<double>(0,1);
             point.z = intersection_point.at<double>(0,2);

             wcs << point << endl;
             points.push_back(point);
         }

         reproPoints.push_back(points);
         wcs.close();
         ccs.close();
//         fzc.close();
     }
     return reproPoints;
}


CheckerboardImage::CheckerboardImage(){}

vector<vector<cv::Point2f>> Calibration::findCorners(vector<cv::Mat> srcImages)
{   //入力画像のチャネルが1つでは動かない
    unsigned int num;
    num = static_cast<unsigned int>(srcImages.size());
    vector<cv::Point2f> corners;
    allpoints = imgnum*PAT_SIZE;
    // ３次元の点を allpoints * 3 の行列(32Bit浮動小数点数:１チャンネル)に変換する
    // チェスボード（キャリブレーションパターン）のコーナー検出
    unsigned int i;
    int found_num = 0;
    vector<vector<cv::Point2f>> img_points;//戻り値
//    cv::namedWindow("FindCorners", cv::WINDOW_AUTOSIZE);
    for (i = 0; i < num; i++){
        auto found = cv::findChessboardCorners(srcImages[i], pattern_size, corners);
        if (found){
            cout << setfill('0') << setw(2) << i << "... ok" << endl;
            found_num++;
        }else{
            cerr << setfill('0') << setw(2) << i << "... fail" << endl;
        }
        //コーナー位置をサブピクセル精度に修正，描画
        cv::Mat src_gray = cv::Mat(srcImages[i].size(), CV_8UC1);
        cv::cvtColor(srcImages[i], src_gray, cv::COLOR_BGR2GRAY);
        cv::find4QuadCornerSubpix(src_gray, corners, cv::Size(3,3));
        cv::Mat markedImage = srcImages[i].clone(); //元画像にはコーナー位置を描画しない
        cv::drawChessboardCorners(markedImage, pattern_size, corners, found);
        img_points.push_back(corners);
//        cv::imwrite("/Users/"+username+"/Desktop/corners"+to_string(i+1)+".jpg", markedImage);
//        cv::imshow("FindCorners",markedImage);
//        cv::waitKey(0);
//        cv::destroyWindow("FindCorners");
    }
    cout << "find corners end" << endl;
    return img_points;
}

CheckerboardImage Calibration::calibrateLens(CheckerboardImage im, CalibrationParameters * param, bool distflag)
{
    //内部パラメータ，歪み係数の推定
    if(distflag == 1){
        cv::calibrateCamera(
            im.wcspoints,
            im.distpoints,
            im.distim[0].size(),
            param->cam_mat,
            param->dist_coefs,
            param->rvecs,
            param->tvecs
        );
    }else{
        cv::calibrateCamera(
            im.wcspoints,
            im.distpoints,
            im.distim[0].size(),
            param->cam_mat,
            param->dist_coefs,
            param->rvecs,
            param->tvecs
            ,CV_CALIB_ZERO_TANGENT_DIST
        );
    }
    cout << "calibrate lens end" << endl;
    return im;
}

tuple<cv::Mat,cv::Mat> Calibration::separateImage(unsigned int n)
{
    cv::Mat plane,project;
    vector<cv::Mat> planes;

    cv::split(inputImages[n], planes); //BGR
    planes[1] = 0;//G
    planes[2] = 0;//R
    merge(planes,plane);
    cv::imwrite(inputpath+"/calib_output/board/board"+to_string(n+1)+".jpg",plane);
    cv::split(inputImages[n], planes);
    planes[0] = 0;//B
    planes[1] = 0;//G
    merge(planes,project);
    cv::imwrite(inputpath+"/calib_output/projected/projected"+to_string(n+1)+".jpg",project);
    return make_tuple(plane,project);
}

void Calibration::drawPoint(string src, string dst, vector<vector<cv::Point2f>> points, cv::Scalar dot_S){

        int dot_r = 5;//半径
        int tickness = 2;
        unsigned int i,j;
        vector<cv::Mat> canvas;

        cv::namedWindow("plot", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

        for (i = 0; i < points.size() ;i++)
        {
            canvas.push_back(cv::imread(src+to_string(i+1)+".jpg"));

            for (j=0; j<PAT_SIZE; j++)
            {
                cv::circle(canvas[i], points[i][j] , dot_r, dot_S, tickness, cv::LINE_AA);

            }
            cv::imshow("plot", canvas[i]);
            cv::waitKey(0);
            cv::destroyWindow("plot");
            cv::imwrite(dst+to_string(i+1)+".jpg", canvas[i]);
        }
}

void Calibration::calculateError(vector<vector<cv::Point2f>>data,vector<vector<cv::Point2f>>truth){

    unsigned int i,j;
    double val;
    double sum = 0;

    for (i=0;i<imgnum;i++)
    {
        ofstream ofs(inputpath+"/calib_output/errorData/errorPic"+to_string(i+1)+".csv");
        for (j=0;j<PAT_SIZE;j++)
        {
            val = double(sqrt((data[i][j].x-truth[i][j].x)*(data[i][j].x-truth[i][j].x)+(data[i][j].y-truth[i][j].y)*(data[i][j].y-truth[i][j].y)));
            ofs << val << endl;
//            sum += val;
        }
//        ofs << "average: " << sum/PAT_SIZE << endl;
        sum = 0;
        ofs.close();
    }
}

