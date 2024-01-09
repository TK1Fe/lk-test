

#ifndef ORB_SLAM3_LK_H
#define ORB_SLAM3_LK_H

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <Frame.h>

using namespace std;
using namespace ORB_SLAM3;


using namespace cv;





vector< cv::Point2f > keypoints;
vector< cv::Point2f > nextkeypoints;
//vector<cv::Point3f> mappointInCurrentFrame;
std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> mappointInCurrentFrame;
//std::vector<Eigen::Vector3f> mappointInCurrentFrame;
vector<cv::Point2f> prev_keypoints;
cv::Mat last_color;
vector<cv::Point2f> next_keypoints;
Sophus::SE3f mVelocity;
// std::vector<cv::Mat> pyr_;
// std::vector<cv::Mat> last_pyr_;
/**非关键帧的情况下，使用光流法跟踪关键帧的特征点,使用 PNP_RANSAC 来计算相机位姿
 * 暂未解决 无法把所有帧的位姿加入CameraTrajectory.txt的问题
 * **/

/// @brief 
/// @param lastKeyFrame 
/// @param currentFrame 
/// @param lastFrame 
/// @param color 
/// @param lastColorIsKeyFrame 
/// @param K 
/// @param mDistCoef 
/// @param mTcw 
/// @param mnMatchesInliers 
/// @return 
Mat computeMtcwUseLK(KeyFrame *lastKeyFrame,Frame currentFrame,Frame lastFrame, Mat color, bool lastColorIsKeyFrame, Mat K, Mat mDistCoef, Sophus::SE3<float> &mTcw, int &mnMatchesInliers)
{
    //cout<<"in computematchuselk"<<endl;
    // int obsPlus = 0;
    // if(lastKeyFrame->mnId>3)
    //     obsPlus = 3;

    //cout << "Address of last_color image data in LK1: " << static_cast<void*>(last_color.data) << endl;

    if(last_color.empty())
    {
        last_color = color;
        return cv::Mat();
    }


    clock_t t1 = clock();
    /**上一帧是关键帧的情况**/
    if(lastColorIsKeyFrame || keypoints.empty())
    {
//        cout<<lastKeyFrame->mvKeysUn.size()<<endl;
        keypoints.clear();    //2D point
        mappointInCurrentFrame.clear();  //3D  point
        nextkeypoints.clear();
        //cout<<"lastKeyFrame->mvpMapPoints.size()"<<lastKeyFrame->mvpMapPoints.size()<<endl;
        for(int i=0;i<lastKeyFrame->mvpMapPoints.size();i++)//copy point from keyframe
        {
            
            if(lastKeyFrame->mvpMapPoints[i])
            {
                keypoints.push_back(lastKeyFrame->mvKeysUn[i].pt);
                //cv::Point3f pt3f;
                Eigen::Vector3f temp;

                if(lastFrame.isSet() && currentFrame.isSet())
                {
                    Sophus::SE3f LastTwc = lastFrame.GetPose().inverse();
                    // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                    mVelocity = currentFrame.GetPose() * LastTwc;
                }
                
                //Sophus::SE3f lTcw = mTcw;
                currentFrame.SetPose(mVelocity * lastFrame.GetPose());
                temp = lastKeyFrame->mvpMapPoints[i]->GetWorldPos().cast<float>();
                Eigen::Vector3f x3Dc =currentFrame.GetPose()*temp;
                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0/x3Dc(2);

                float u = currentFrame.fx*xc*invzc+currentFrame.cx;
                float v = currentFrame.fy*yc*invzc+currentFrame.cy;
                //cout<<"1"<<endl; 
                // pt3f.x = temp(0);
                // pt3f.y = temp(1);
                // pt3f.z = temp(2);
                nextkeypoints.emplace_back(u, v);
                
                mappointInCurrentFrame.push_back(temp);
            }
        }
    }
    else
    {
        // cout<<"next_keypoints.size()"<<next_keypoints.size()<<endl;
        // cout<<"keypoints.size()"<<keypoints.size()<<endl;
        // keypoints.clear();
        // keypoints=next_keypoints; //last track points
        // cout<<"next_keypoints.size()"<<next_keypoints.size()<<endl;
        // cout<<"keypoints.size()"<<keypoints.size()<<endl;
        for(int i=0;i<keypoints.size();i++)//copy point from keyframe
        {
            
            //if(lastKeyFrame->mvpMapPoints[i])
            //{
                //keypoints.push_back(lastKeyFrame->mvKeysUn[i].pt);
                //cout<<"1"<<endl;
                //cv::Point3f pt3f;
                Eigen::Vector3f temp;
                //Eigen::Vector3f temp_= mappointInCurrentFrame;
                //cv::Point3f temp;
                

                if(lastFrame.isSet() && currentFrame.isSet())
                {
                    Sophus::SE3f LastTwc = lastFrame.GetPose().inverse();
                    // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                    mVelocity = currentFrame.GetPose() * LastTwc;

                }
                
                //Sophus::SE3f lTcw = mTcw;
                currentFrame.SetPose(mVelocity * lastFrame.GetPose());
                //temp = lastKeyFrame->mvpMapPoints[i]->GetWorldPos();

                //temp = lastFrame.mvpMapPoints[i]->GetWorldPos();//TODO !maybe null point
                cout<<"i = "<<i<<endl;
                temp = mappointInCurrentFrame[i];
                cout<<"i = "<<i<<endl;
                Eigen::Vector3f x3Dc =currentFrame.GetPose()*temp;
                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0/x3Dc(2);

                float u = currentFrame.fx*xc*invzc+currentFrame.cx;
                float v = currentFrame.fy*yc*invzc+currentFrame.cy;

                nextkeypoints.emplace_back(u, v);
                
            //}
        }
    }
    clock_t t2 = clock();
    //cout<<"mappointInCurrentFrame = "<<mappointInCurrentFrame.size()<<endl;
    //cout<<"keypoints = "<<keypoints.size()<<endl;
    /**需要用到的数据*/
    vector<cv::Point2f> next_keypoints;
    next_keypoints.clear();
    for ( auto nextkp:nextkeypoints )
        next_keypoints.push_back(nextkp);

    cout<<"neXt keypionts size = "<<next_keypoints.size()<<endl;

    cout<<"keypoints.size()"<<keypoints.size()<<endl;
    prev_keypoints.clear();
    for ( auto kp:keypoints )
        prev_keypoints.push_back(kp);
    cout<<"pre keypionts size = "<<prev_keypoints.size()<<endl;
    
    clock_t t3 = clock();
    
    vector<unsigned char> status;//判断该点是否跟踪失败
    vector<float> error;
    Mat last_gray,gray;//LK光流法用于跟踪特征点的两帧
    cvtColor(last_color,last_gray,CV_BGR2GRAY);
    cvtColor(color,gray,CV_BGR2GRAY);

    cv::calcOpticalFlowPyrLK( last_gray, gray, prev_keypoints, next_keypoints, status, error ,cv::Size(21, 21), 0,
                            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                            (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS),1e-4);//计算光流



//TODOpyramid
    // std::vector<cv::Mat> pyr_;
    // std::vector<cv::Mat> last_pyr_;
    // int klt_win_size = 9;
    // int nklt_pyr_lvl = 6;
    // cv::Size winSize(klt_win_size,klt_win_size);
    // cv::buildOpticalFlowPyramid(gray, pyr_, winSize, nklt_pyr_lvl);
    // //cout<<"over build optf pyr"<<endl;

    // if(last_pyr_.empty()||lastColorIsKeyFrame){
    //     cv::buildOpticalFlowPyramid(last_gray, last_pyr_, winSize, nklt_pyr_lvl);
    // }
    // cout<<"over last build optf pyr"<<endl;
    
    // clock_t t4 = clock();

    // int nmax_iter = 30;
    // float fmax_px_precision = 0.01f;
    // cv::TermCriteria klt_convg_crit_(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, nmax_iter, fmax_px_precision);
    // cv::calcOpticalFlowPyrLK(last_pyr_, pyr_, prev_keypoints, next_keypoints, status, error, winSize,  nklt_pyr_lvl, klt_convg_crit_,
    //                              (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS)
    //     );
    // clock_t t5 = clock();
    // cout<<"over calc optf pyrlk"<<endl;






    /** 把跟丢的点删掉**/
    cout<<"next keypionts size = "<<next_keypoints.size()<<endl;
    cout<<"keypionts size = "<<keypoints.size()<<endl;
    int i=0;
    for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = keypoints.erase(iter);
            //iter = next_keypoints.erase(iter);
            //cout<<i<<endl;
            continue;
        }
        *iter = next_keypoints[i];//edit keypoints' coordinate
        iter++;
    }

    i=0;
    for ( auto iter=next_keypoints.begin(); iter!=next_keypoints.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = next_keypoints.erase(iter);
            //iter = next_keypoints.erase(iter);
            //cout<<i<<endl;
            continue;
        }
        iter++;
    }
    cout<<"keypionts size = "<<keypoints.size()<<endl;
    cout<<"next keypionts size = "<<next_keypoints.size()<<endl;
//TODO
    //Eigen::Vector3f temp_mappointInCurrentFrame;
    
    i = 0;
    for ( auto iter=mappointInCurrentFrame.begin(); iter!=mappointInCurrentFrame.end(); i++)//erase the match mappoint while the keypoint is erased
    {
        if ( status[i] == 0 )
        {
            iter = mappointInCurrentFrame.erase(iter);
            continue;
        }
        iter++;
    }
    
    /**使用PnPRansac计算位姿*/
    
    vector<cv::Mat> point3D;
    cv:Mat R_vector,T,R;
    vector<int> ransacInlier;
    clock_t t6 = clock();

    if(!(mappointInCurrentFrame.empty()||prev_keypoints.empty()||mDistCoef.empty()))
    {
        cout<<"pointNUM"<<mappointInCurrentFrame.size()<<endl;
        cout<<"pointNUM"<<prev_keypoints.size()<<endl;
        if(next_keypoints.size()<20)
        {
            cout<<"Optical flow need more points"<<endl;
            return cv::Mat();
        }
        cv::Mat mappointInCurrentFrame_cv;
        cv::eigen2cv(mappointInCurrentFrame, mappointInCurrentFrame_cv);
        if(mappointInCurrentFrame.size()==next_keypoints.size()){
            cv::solvePnPRansac(mappointInCurrentFrame_cv, next_keypoints, K, mDistCoef , R_vector, T, false, 50,3, 0.98, ransacInlier, SOLVEPNP_ITERATIVE);
        }
        else{
            cv::solvePnPRansac(mappointInCurrentFrame_cv, keypoints, K, mDistCoef , R_vector, T, false, 50,3, 0.98, ransacInlier, SOLVEPNP_ITERATIVE);
        }
        /*cv::Rodrigues(R_vector, R);
        Mat_<double> Rt = (Mat_<double>(4, 4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),T.at<double>(0),
                R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),T.at<double>(1),
                R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),T.at<double>(2),
                0, 0, 0, 1);
        cv::Mat Rt_float;
        Rt.convertTo(Rt_float,CV_32FC1);
        

        // Extract rotation and translation components from the 4x4 matrix
        cv::Mat R = Rt_float(cv::Rect(0, 0, 3, 3)).clone();  // Assuming rotation is in the top-left 3x3 submatrix
        cv::Mat T = Rt_float(cv::Rect(3, 0, 1, 3)).clone();  // Assuming translation is in the rightmost column

        // Convert rotation and translation to Sophus SE3 type
        Sophus::SE3f mTcw_se3(Sophus::Matrix3f(R.ptr<float>()), Eigen::Vector3f(T.ptr<float>()));
        mTcw = mTcw_se3;
        //mTcw = Rt_float;//位姿*/
        cv::Mat R;
        cv::Rodrigues(R_vector, R);

        // 将旋转和平移转换为 Sophus::SE3f
        Eigen::Matrix3f R_matrix;
        //cv::eigen2cv(R, R_matrix);

        // 将旋转矩阵从 double 转换为 float
        R_matrix << static_cast<float>(R.at<double>(0, 0)), static_cast<float>(R.at<double>(0, 1)), static_cast<float>(R.at<double>(0, 2)),
                static_cast<float>(R.at<double>(1, 0)), static_cast<float>(R.at<double>(1, 1)), static_cast<float>(R.at<double>(1, 2)),
                static_cast<float>(R.at<double>(2, 0)), static_cast<float>(R.at<double>(2, 1)), static_cast<float>(R.at<double>(2, 2));

        // 将平移向量从 double 转换为 float
        Eigen::Vector3f T_vector;
        //cv::cv2eigen(T, T_vector);
        T_vector << static_cast<float>(T.at<double>(0)), static_cast<float>(T.at<double>(1)), static_cast<float>(T.at<double>(2));

        // 设置 Sophus::SE3f
        mTcw = Sophus::SE3f(R_matrix, T_vector);
        //std::cout << "mTcw:\n" << mTcw.matrix() << std::endl;
    }
    clock_t t7 = clock();
    cout<<"push keypoints use time:"<<1000*(float)(t2-t1)/CLOCKS_PER_SEC<<"ms"<<endl;
    cout<<"push keypoints2 use time:"<<1000*(float)(t3-t2)/CLOCKS_PER_SEC<<"ms"<<endl;
    //cout<<"buildOpticalFlowPyramid use time:"<<1000*(float)(t4-t3)/CLOCKS_PER_SEC<<"ms"<<endl;
    //cout<<"calcOpticalFlowPyrLK use time:"<<1000*(float)(t5-t4)/CLOCKS_PER_SEC<<"ms"<<endl;
    //cout<<"erase outlier use time:"<<1000*(float)(t6-t5)/CLOCKS_PER_SEC<<"ms"<<endl;
    cout<<"solvePnPRansac use time:"<<1000*(float)(t7-t6)/CLOCKS_PER_SEC<<"ms"<<endl;

    if (prev_keypoints.size() == 0)
    {
        cout<<"LK -- all keypoints are lost."<<endl;
        return cv::Mat();
    }
    /** 画出 keypoints*/
    cv::Mat img_show = color.clone();
    int point = 0;
    int iterOfInlier = 0;
    for ( auto kp:next_keypoints )
    {
        for(iterOfInlier =0;iterOfInlier<ransacInlier.size();iterOfInlier++)
        {
            if(point == ransacInlier[iterOfInlier])
            {
                cv::circle(img_show, kp, 5, cv::Scalar(0, 255, 0), 1);
                cv::circle(img_show, kp, 1, cv::Scalar(0, 255, 0), -1);
            }
        }
        point++;
    }
    mnMatchesInliers = ransacInlier.size();
    cout<<"PNP inlier: "<<ransacInlier.size()<<endl;
    last_color = color;
    //last_pyr_=pyr_;
    keypoints = next_keypoints;
    //cout << "Address of last_color image data in LK2: " << static_cast<void*>(last_color.data) << endl;

    return img_show;//返回值是rgb图片，用于显示光流跟踪到的特征点
}


#endif //ORB_SLAM3_LK_H

