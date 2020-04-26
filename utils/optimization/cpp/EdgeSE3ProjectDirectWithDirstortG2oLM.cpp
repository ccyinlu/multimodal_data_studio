#include <vector>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include"mex.h"

using namespace std;
using namespace g2o;

class EdgeSE3ProjectDirectWithDirstortG2oLM: public BaseUnaryEdge< 1, double, VertexSE3Expmap>
{
    public:
        Eigen::Vector3d x_world_;                                   // 3D point in world frame
        float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0;                   // Camera intrinsics
        float k1_ = 0.0, k2_ = 0.0, k3_= 0.0, p1_ = 0.0, p2_ = 0.0; // Camera distortion coefficient;
        float *maskDT_ = nullptr;                                    // current image

        size_t image_width_;
        size_t image_height_;

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirectWithDirstortG2oLM() {}

    EdgeSE3ProjectDirectWithDirstortG2oLM(Eigen::Vector3d point, double *D, double *K, double *imageSize, float * maskDT) {
        x_world_ = point;
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

        // mexPrintf("image size: [%d, %d]\n",image_width_, image_height_);
    }

    virtual void computeError()
    {
        
        const VertexSE3Expmap* vertex  =static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        
        //将激光点云投影到相空间坐标
        Eigen::Vector3d x_local = vertex->estimate().map ( x_world_ );
        
        //归一化
        float x = x_local(0)/x_local(2);
        float y = x_local(1)/x_local(2);

        // //畸变
        float r2 = (x*x + y*y);
        float a = (1 + k1_*r2 + k2_*pow(r2,2) + k3_*pow(r2,3));
        float xx = a*x + 2*p1_*x*y + p2_*(r2 + 2*x*x);
        float yy = a*y + p1_*(r2+2*y*y) + 2*p2_*x*y;

        //内参变化
        int u = floor(fx_ * xx + cx_);
        int v = floor(fy_ * yy + cy_);
        // float u = fx_ * x + cx_;
        // float v = fy_ * y + cy_;
        
        // check x,y is in the image
        if ( u-2<0 || ( u+2 ) > image_width_ || ( v-2 ) <0 || ( v+2 ) > image_height_ )
        {
            _error ( 0,0 ) = 0.0;
            this->setLevel ( 1 );
        }
        else
        {
            _error ( 0,0 ) = getPixelValue ( u,v ) - _measurement;
        }
    }

    // plus in manifold
    
    virtual void linearizeOplus( )
    {
        // std::cout<<"distortion param: "<<k1_<<" "<<k2_<<" "<<k3_<<" "<<p1_<<" "<<p2_<<std::endl;
        if ( level() == 1 )
        {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }
        VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*> ( _vertices[0] );
        Eigen::Vector3d xyz_trans = vtx->estimate().map ( x_world_ );   // q in book

        {
            double x = xyz_trans[0];
            double y = xyz_trans[1];
            double invz = 1.0/xyz_trans[2];
            double invz_2 = invz*invz;
            
            // 相空间坐标相对于外参的雅克比矩阵
            Eigen::Matrix<double, 3, 6> jacobian_norm_xyz_xi;
            jacobian_norm_xyz_xi.block<3,3>(0,3) = Eigen::Matrix<double,3,3>::Identity();
            jacobian_norm_xyz_xi(0,0) = 0;
            jacobian_norm_xyz_xi(0,1) = xyz_trans[2]; 
            jacobian_norm_xyz_xi(0,2) = -y;

            jacobian_norm_xyz_xi(1,0) = -xyz_trans[2]; 
            jacobian_norm_xyz_xi(1,1) = 0; 
            jacobian_norm_xyz_xi(1,2) = x;

            jacobian_norm_xyz_xi(2,0) = y;
            jacobian_norm_xyz_xi(2,1) = -x; 
            jacobian_norm_xyz_xi(2,2) = 0;


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

            // 图像坐标相对于畸变坐标的雅克比矩阵
            Eigen::Matrix<double, 2, 2> jacobian_uv_distortion_xy;
            double u = fx_*distortion_x + cx_;
            double v = fy_*distortion_y + cy_;

            jacobian_uv_distortion_xy(0,0) = fx_;
            jacobian_uv_distortion_xy(0,1) = 0;
            jacobian_uv_distortion_xy(1,0) = 0;
            jacobian_uv_distortion_xy(1,1) = fy_;

            // 残差相对于图像坐标的雅克比矩阵
            Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

            jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
            jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;


            Eigen::Matrix<double,2,6> jacobian_uv_xi;
            jacobian_uv_xi = jacobian_uv_distortion_xy*jacobian_distortion_xy_norm_xy*jacobian_norm_xy_xyz*jacobian_norm_xyz_xi;
            // std::cout<<"kinggreat24 jacobian_uv_xi: "<<std::endl<<jacobian_uv_xi.matrix()<<std::endl;
            // std::cout<<"**************************************************"<<std::endl;
            // 最终的雅克比矩阵
            _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_distortion_xy*jacobian_distortion_xy_norm_xy*jacobian_norm_xy_xyz*jacobian_norm_xyz_xi;
        }
    }
    

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

     // dummy read and write functions because we don't care...
    virtual bool read ( std::istream& in ) {return false;}
    virtual bool write ( std::ostream& out ) const {return false;}
};