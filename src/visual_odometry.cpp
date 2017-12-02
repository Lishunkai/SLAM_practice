#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

VisualOdometry::VisualOdometry():
    state_(INITIALIZING), ref_(nullptr), curr_ (nullptr), map_(new Map), num_lost_(0), num_inliers_(0)
{
    // 从参数文件中读取参数
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry()
{}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_)
    {

    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = frame; // 将当前帧和参考帧都设置为现在的这一帧
        
        map_->insertKeyFrame(frame);

        // extract features from the first frame 
        extractKeyPoints();
        computeDescriptors();

        // compute the 3d position of features in the ref frame 
        setRef3DPoints();
        break; // switch中每个case后都要加一个break，使程序执行完此case后能跳出switch。
               // 否则，程序会在执行完此case后，一直按顺序执行其他case，直至switch的内容结束。
    }

    case OK:
    {
        curr_ = frame;
        extractKeyPoints();
        computeDescriptors();

        featureMatching();
        poseEstimationPnP();

        if (checkEstimatedPose() == true) // a good estimation
        {
            curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w 
            ref_ = curr_; // 更新状态，用于下一时刻的判断

            // compute the 3d position of features in the ref frame
            setRef3DPoints();
            num_lost_ = 0;
            if (checkKeyFrame() == true) // is a key-frame
                addKeyFrame();
        }
        else // bad estimation
        {
            num_lost_++;
            if (num_lost_ > max_num_lost_)
                state_ = LOST;
            return false;
        }
        break;
    }

    case LOST:
    {
        cout<<"VO has lost."<<endl;
        break;
    }
    
    }

    return true;
}

// 用ORB作为特征点提取和匹配
void VisualOdometry::extractKeyPoints()
{
    orb_->detect(curr_->color_, keypoints_curr_); // OpenCV3中将DetectAndCompute分开了
}

void VisualOdometry::computeDescriptors()
{
    orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_); // OpenCV3中将DetectAndCompute分开了
}

void VisualOdometry::featureMatching()
{
    // match desp_ref and desp_curr, use OpenCV's brute force matching(暴力匹配)
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING); // 汉明距离
    matcher.match(descriptors_ref_, descriptors_curr_, matches);
    
    // select the best matches
    float min_dis = std::min_element
    (
        matches.begin(), matches.end(),
        [] (const cv::DMatch& m1, const cv::DMatch& m2)
        {return m1.distance < m2.distance;}
    )->distance;

    feature_matches_.clear();
    for(cv::DMatch& m:matches) // :是什么意思？
    {
        if(m.distance<max<float>(min_dis*match_ratio_, 30.0)) // 若距离较小，则认为是匹配的
            feature_matches_.push_back(m);
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
}

void VisualOdometry::setRef3DPoints()
{
    // select the features with depth measurements 
    pts_3d_ref_.clear();
    descriptors_ref_ = Mat();
    for(size_t i=0; i<keypoints_curr_.size(); i++)
    {
        double d = ref_->findDepth(keypoints_curr_[i]); // 深度
        if(d > 0)
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
            pts_3d_ref_.push_back(cv::Point3f(p_cam(0,0), p_cam(1,0), p_cam(2,0)));
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for ( cv::DMatch m:feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
    }
    
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}

bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_ );
}

}