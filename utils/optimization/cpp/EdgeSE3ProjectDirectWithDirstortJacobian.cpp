#include <vector>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include"mex.h"

using namespace std;

class EdgeSE3ProjectDirectWithDistortJacobian{
public:
    std::vector<Eigen::Vector3d>x_worlds_;
    double cx_=0, cy_=0, fx_=0, fy_=0;                       // Camera intrinsics
    double k1_=0.0, k2_=0.0, k3_=0.0, p1_=0.0, p2_=0.0;      // Camera distortion coefficient;
    float *maskDT_;                                         // current image
    size_t image_width_;
    size_t image_height_;
    Eigen::Matrix<double, 6, 1> xi_;

EdgeSE3ProjectDirectWithDistortJacobian() {}

EdgeSE3ProjectDirectWithDistortJacobian(std::vector<Eigen::Vector3d> points, double *D, double *K, double *imageSize, float *maskDT, Eigen::Matrix<double, 6, 1> xi) {
    x_worlds_ = points;
    k1_ = D[0];
    k2_ = D[1];
    k3_ = D[2];
    p1_ = D[3];
    p2_ = D[4];

    fx_ = K[0];
    fy_ = K[4];
    cx_ = K[6];
    cy_ = K[7];

    image_width_ = size_t(imageSize[0]);
    image_height_ = size_t(imageSize[1]);

    maskDT_ = maskDT;
    
    xi_ = xi;

    // mexPrintf("image size: [%d, %d]\n",image_width_, image_height_);
}

public:
    Eigen::Matrix<double, 1, 6> jacobians(){
        Eigen::Matrix<double, 1, 6>_jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        int points_count = 0;
        for(auto x_world_ : x_worlds_){
            points_count++;
            _jacobianOplusXi = _jacobianOplusXi + jacobian(x_world_, xi_);
        }
        return _jacobianOplusXi/points_count;
    }

    std::vector<Eigen::Matrix<double, 1, 6>> jacobians_new(){
        std::vector<Eigen::Matrix<double, 1, 6>> _jacobianOplusXi;
        for(auto x_world_ : x_worlds_){
            Eigen::Matrix<double, 1, 6>current_jacobianOplusXi = jacobian(x_world_, xi_);
            _jacobianOplusXi.push_back(current_jacobianOplusXi);
        }
        return _jacobianOplusXi;
    }

protected:
    // get a gray scale value from current image (bilinear interpolated)
    inline float getPixelValue ( int x, int y )
    {
        if(x < 0 )
            x = 0;
        else if (x > image_width_ - 1)
        {
            x = image_width_ - 1;
        }

        if(y < 0 )
            y = 0;
        else if (y > image_height_ - 1)
        {
            y = image_height_ - 1;
        }

        return maskDT_[x * image_height_ + y];
    }

    inline Eigen::Matrix<double, 1, 6> jacobian(Eigen::Vector3d x_world_, Eigen::Matrix<double, 6, 1> xi){
        Eigen::Matrix<double, 1, 6>_jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();

        Eigen::Matrix<double, 4, 4>SE3Matrix = Sophus::SE3<double>::exp(xi).matrix();
        Eigen::Vector3d xyz_trans = SE3Matrix.block<3,3>(0,0) * x_world_ + SE3Matrix.block<3,1>(0,3);
        static bool if_printf = false;
        if(if_printf){
            mexPrintf("xi: [%f, %f, %f, %f, %f, %f]\n", xi[0], xi[1], xi[2], xi[3], xi[4], xi[5]);
            mexPrintf("lidar points: [%f %f %f], camera points: [%f, %f, %f]\n", x_world_[0], x_world_[1], x_world_[2], xyz_trans[0], xyz_trans[1], xyz_trans[2]);
        }
        
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        // 相空间坐标相对于外参的雅克比矩阵
        Eigen::Matrix<double, 3, 6> jacobian_norm_xyz_xi;
        jacobian_norm_xyz_xi(0,0) = 1;
        jacobian_norm_xyz_xi(0,1) = 0; 
        jacobian_norm_xyz_xi(0,2) = 0;
        jacobian_norm_xyz_xi(0,3) = 0;
        jacobian_norm_xyz_xi(0,4) = xyz_trans[2]; 
        jacobian_norm_xyz_xi(0,5) = -y;

        jacobian_norm_xyz_xi(1,0) = 0;
        jacobian_norm_xyz_xi(1,1) = 1; 
        jacobian_norm_xyz_xi(1,2) = 0;
        jacobian_norm_xyz_xi(1,3) = -xyz_trans[2]; 
        jacobian_norm_xyz_xi(1,4) = 0; 
        jacobian_norm_xyz_xi(1,5) = x;

        jacobian_norm_xyz_xi(2,0) = 0;
        jacobian_norm_xyz_xi(2,1) = 0; 
        jacobian_norm_xyz_xi(2,2) = 1;
        jacobian_norm_xyz_xi(2,3) = y;
        jacobian_norm_xyz_xi(2,4) = -x; 
        jacobian_norm_xyz_xi(2,5) = 0;

        // jacobian_norm_xyz_xi.block<3,3>(0,3) = Eigen::Matrix<double,3,3>::Identity();
        // jacobian_norm_xyz_xi(0,0) = 0;
        // jacobian_norm_xyz_xi(0,1) = xyz_trans[2]; 
        // jacobian_norm_xyz_xi(0,2) = -y;

        // jacobian_norm_xyz_xi(1,0) = -xyz_trans[2]; 
        // jacobian_norm_xyz_xi(1,1) = 0; 
        // jacobian_norm_xyz_xi(1,2) = x;

        // jacobian_norm_xyz_xi(2,0) = y;
        // jacobian_norm_xyz_xi(2,1) = -x; 
        // jacobian_norm_xyz_xi(2,2) = 0;

        if(if_printf){
            mexPrintf("jacobian_norm_xyz_xi: \n");
            mexPrintf("[%f, %f, %f, %f, %f, %f]\n", jacobian_norm_xyz_xi(0,0), jacobian_norm_xyz_xi(0,1), jacobian_norm_xyz_xi(0,2), jacobian_norm_xyz_xi(0,3), jacobian_norm_xyz_xi(0,4), jacobian_norm_xyz_xi(0,5));
            mexPrintf("[%f, %f, %f, %f, %f, %f]\n", jacobian_norm_xyz_xi(1,0), jacobian_norm_xyz_xi(1,1), jacobian_norm_xyz_xi(1,2), jacobian_norm_xyz_xi(1,3), jacobian_norm_xyz_xi(1,4), jacobian_norm_xyz_xi(1,5));
            mexPrintf("[%f, %f, %f, %f, %f, %f]\n", jacobian_norm_xyz_xi(2,0), jacobian_norm_xyz_xi(2,1), jacobian_norm_xyz_xi(2,2), jacobian_norm_xyz_xi(2,3), jacobian_norm_xyz_xi(2,4), jacobian_norm_xyz_xi(2,5));
        }

        // 归一化坐标相对于相空间点坐标的雅克比
        Eigen::Matrix<double, 2, 3> jacobian_norm_xy_xyz;
        double norm_x = xyz_trans[0]/xyz_trans[2];
        double norm_y = xyz_trans[1]/xyz_trans[2];
        jacobian_norm_xy_xyz(0,0) = invz;
        jacobian_norm_xy_xyz(0,1) = 0;
        jacobian_norm_xy_xyz(0,2) = -x*invz_2;
        jacobian_norm_xy_xyz(1,0) = 0;
        jacobian_norm_xy_xyz(1,1) = invz;
        jacobian_norm_xy_xyz(1,2) = -y*invz_2;

        if(if_printf){
            mexPrintf("jacobian_norm_xy_xyz: \n");
            mexPrintf("[%f, %f, %f]\n", jacobian_norm_xy_xyz(0,0), jacobian_norm_xy_xyz(0,1), jacobian_norm_xy_xyz(0,2));
            mexPrintf("[%f, %f, %f]\n", jacobian_norm_xy_xyz(1,0), jacobian_norm_xy_xyz(1,1), jacobian_norm_xy_xyz(1,2));
        }

        // 畸变坐标相对于归一化坐标的雅克比矩阵
        Eigen::Matrix<double, 2, 2> jacobian_distortion_xy_norm_xy;
        double r2 = (norm_x*norm_x + norm_y*norm_y);
        double a = k1_*2*norm_x + 2*k2_*r2*2*norm_x + 3*k3_*pow(r2,2)*2*norm_x;
        double b = 1 + k1_*r2 + k2_*pow(r2,2) + k3_*pow(r2,3);
        double c = k1_*2*norm_y + 2*k2_*r2 *2*norm_y + k3_ *3*pow(r2,2)*2*norm_y;

        double distortion_x = b * norm_x + 2 * p1_*norm_x*norm_y + p2_*(r2 + 2* norm_x*norm_x); 
        double distortion_y = b * norm_y + p1_* (r2 + 2*norm_y*norm_y) + 2*p2_*norm_x*norm_y;

        jacobian_distortion_xy_norm_xy(0,0) = a * norm_x + b + 2*p1_*norm_y + 6*p2_*norm_x;
        jacobian_distortion_xy_norm_xy(0,1) = c * norm_x + 2*p1_*norm_x + p2_*2*norm_y;
        jacobian_distortion_xy_norm_xy(1,0) = a * norm_y + p1_*2*norm_x + 2*p2_*norm_y;
        jacobian_distortion_xy_norm_xy(1,1) = c * norm_y + b + p1_*6*norm_y + 2*p2_*norm_x;

        if(if_printf){
            mexPrintf("jacobian_distortion_xy_norm_xy: \n");
            mexPrintf("[%f, %f]\n", jacobian_distortion_xy_norm_xy(0,0), jacobian_distortion_xy_norm_xy(0,1));
            mexPrintf("[%f, %f]\n", jacobian_distortion_xy_norm_xy(1,0), jacobian_distortion_xy_norm_xy(1,1));
        }

        // 图像坐标相对于畸变坐标的雅克比矩阵
        Eigen::Matrix<double, 2, 2> jacobian_uv_distortion_xy;
        double u = fx_*distortion_x + cx_;
        double v = fy_*distortion_y + cy_;
        jacobian_uv_distortion_xy(0,0) = fx_;
        jacobian_uv_distortion_xy(0,1) = 0;
        jacobian_uv_distortion_xy(1,0) = 0;
        jacobian_uv_distortion_xy(1,1) = fy_;

        if(if_printf){
            mexPrintf("jacobian_uv_distortion_xy: \n");
            mexPrintf("[%f, %f]\n", jacobian_uv_distortion_xy(0,0), jacobian_uv_distortion_xy(0,1));
            mexPrintf("[%f, %f]\n", jacobian_uv_distortion_xy(1,0), jacobian_uv_distortion_xy(1,1));
        }

        // 残差相对于图像坐标的雅克比矩阵
        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv ( 0, 0 ) = ( getPixelValue( floor(u+1), floor(v) ) - getPixelValue( floor(u-1), floor(v) ) ) /2;
        jacobian_pixel_uv ( 0, 1 ) = ( getPixelValue( floor(u), floor(v+1) ) - getPixelValue ( floor(u), floor(v-1) ) ) /2;

        if(if_printf){
            mexPrintf("jacobian_pixel_uv: \n");
            mexPrintf("[%f, %f]\n", jacobian_pixel_uv(0,0), jacobian_pixel_uv(0,1));
        }

        // 最终的雅克比矩阵
        _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_distortion_xy*jacobian_distortion_xy_norm_xy*jacobian_norm_xy_xyz*jacobian_norm_xyz_xi;

        if(if_printf){
            mexPrintf("_jacobianOplusXi: [%f, %f, %f, %f, %f, %f]\n", _jacobianOplusXi(0, 0), _jacobianOplusXi(0, 1), _jacobianOplusXi(0, 2), _jacobianOplusXi(0, 3), _jacobianOplusXi(0, 4), _jacobianOplusXi(0, 5));
            if_printf = false;
        }
        return _jacobianOplusXi;
    }

    inline Eigen::Matrix<double, 1, 6> jacobian2(Eigen::Vector3d x_world_, Eigen::Matrix<double, 6, 1> xi){
        Eigen::Matrix<double, 1, 6>_jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();

        Eigen::Matrix<double, 4, 4>SE3Matrix = Sophus::SE3<double>::exp(xi).matrix();
        Eigen::Vector3d xyz_trans = SE3Matrix.block<3,3>(0,0) * x_world_ + SE3Matrix.block<3,1>(0,3);
        // static bool if_printf = true;
        // if(if_printf){
        //     mexPrintf("xi: [%f, %f, %f, %f, %f, %f]\n", xi[0], xi[1], xi[2], xi[3], xi[4], xi[5]);
        //     mexPrintf("lidar points: [%f %f %f], camera points: [%f, %f, %f]\n", x_world_[0], x_world_[1], x_world_[2], xyz_trans[0], xyz_trans[1], xyz_trans[2]);
        //     if_printf = false;
        // }

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        float u = x*fx_*invz + cx_;
        float v = y*fy_*invz + cy_;

        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai ( 0,3 ) = - x*y*invz_2 *fx_;
        jacobian_uv_ksai ( 0,4 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
        jacobian_uv_ksai ( 0,5 ) = - y*invz *fx_;
        jacobian_uv_ksai ( 0,1 ) = invz *fx_;
        jacobian_uv_ksai ( 0,2 ) = 0;
        jacobian_uv_ksai ( 0,3 ) = -x*invz_2 *fx_;

        jacobian_uv_ksai ( 1,3 ) = - ( 1+y*y*invz_2 ) *fy_;
        jacobian_uv_ksai ( 1,4 ) = x*y*invz_2 *fy_;
        jacobian_uv_ksai ( 1,5 ) = x*invz *fy_;
        jacobian_uv_ksai ( 1,1 ) = 0;
        jacobian_uv_ksai ( 1,2 ) = invz *fy_;
        jacobian_uv_ksai ( 1,3 ) = -y*invz_2 *fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv ( 0, 0 ) = ( getPixelValue( floor(u+1), floor(v) ) - getPixelValue( floor(u-1), floor(v) ) ) /2;
        jacobian_pixel_uv ( 0, 1 ) = ( getPixelValue( floor(u), floor(v+1) ) - getPixelValue ( floor(u), floor(v-1) ) ) /2;

        // 最终的雅克比矩阵
        _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;

        static bool if_printf = true;
        if(if_printf){
            mexPrintf("_jacobianOplusXi: [%f, %f, %f, %f, %f, %f]\n", _jacobianOplusXi(0, 0), _jacobianOplusXi(0, 1), _jacobianOplusXi(0, 2), _jacobianOplusXi(0, 3), _jacobianOplusXi(0, 4), _jacobianOplusXi(0, 5));
            if_printf = false;
        }

        return _jacobianOplusXi;
    }
};