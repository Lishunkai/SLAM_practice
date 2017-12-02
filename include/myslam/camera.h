#ifndef CAMERA_H
#define CAMERA_H
#include "myslam/common_include.h"
// 把常用的头文件放在common_include.h中，这样就可以避免每次书写很长的一串include

namespace myslam
{
// 我们用命名空间将类包裹起来。命名空间可以防止我们不小心定义出别的库里同名的函数，是一种比较安全和规范的做法。

// Pinhole RGB-D camera model
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr; // 将智能指针定义成Camera的指针类型
    float fx_, fy_, cx_, cy_, depth_scale_; // camera intrinsics

    Camera();
    Camera(float fx, float fy, float cx, float cy, float depth_scale = 0):
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale)
    {}
    // 这段话这样写是什么意思？

    // coordinate transform: world, camera, pixel
    Vector3d world2camera(const Vector3d& p_w, const SE3& T_c_w);
    Vector3d camera2world(const Vector3d& p_c, const SE3& T_c_w); // 不是Twc？
    Vector2d camera2pixel(const Vector3d& p_c);
    Vector3d pixel2camera(const Vector2d& p_p, double depth = 1);
    Vector3d pixel2world(const Vector2d& p_p, const SE3& T_c_w, double depth = 1);
    Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w);
};

}

#endif // CAMERA_H