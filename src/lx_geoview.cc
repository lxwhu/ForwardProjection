#include "lx_geoview.h"
#include "lx_camera.h"

//#define _EIGEN

#ifdef _EIGEN
#include "Eigen/eigen.h"
#else
#include <assert.h>
#include <math.h>
#include <memory.h>
static inline void MatrixTranspose(const double* MatrixOrigin, double* MatrixNew, int m, int n)
{
    int i;
    int j;
    for (i = 0; i != n; i ++)
    {
        for (j = 0; j != m; j ++)
        {
            MatrixNew[i*m+j] = MatrixOrigin[j*n+i];
        }
    }
}
static inline void MatrixMultiply(const double* MatrixA,const  double* MatrixB, double* MatrixC, int ARow, int AColumn, int BColumn)
{
    int i;
    int j;
    int k;

    for(i = 0; i != ARow; i ++)
        for(j=0; j<BColumn; j++)
        {
            MatrixC[i*BColumn+j] = 0.0;
            for(k = 0; k<AColumn; k++)
                MatrixC[i*BColumn+j] += MatrixA[i*AColumn+k] * MatrixB[j+k*BColumn];
        }
}
#endif

static inline void     GenerateXrotMatrix(double a, double* rotation_matrix){
    memset(rotation_matrix, 0, sizeof(double)* 9);
    rotation_matrix[0] = 1;
    rotation_matrix[4] = cos(a);
    rotation_matrix[5] = -sin(a);
    rotation_matrix[7] = sin(a);
    rotation_matrix[8] = cos(a);
}
static inline void    GenerateYrotMatrix(double a, double* rotation_matrix){
    memset(rotation_matrix, 0, sizeof(double)* 9);
    rotation_matrix[4] = 1;
    rotation_matrix[0] = cos(a);
    rotation_matrix[2] = -sin(a);
    rotation_matrix[6] = sin(a);
    rotation_matrix[8] = cos(a);
}
static inline void    GenerateZrotMatrix(double a, double* rotation_matrix){
    memset(rotation_matrix, 0, sizeof(double)* 9);
    rotation_matrix[8] = 1;
    rotation_matrix[0] = cos(a);
    rotation_matrix[1] = -sin(a);
    rotation_matrix[3] = sin(a);
    rotation_matrix[4] = cos(a);
}

static const double kIdentityMatrix[9] = {1,0,0,0,1,0,0,0,1};

namespace xlingeo{
    void CalcRotationMatrix(double xangle, double yangle, double zangle, unsigned int rotation_order_list[3], double* rotation_matrix){
        assert(rotation_order_list[0]<3);
        assert(rotation_order_list[1]<3);
        assert(rotation_order_list[2]<3);
#ifdef _EIGEN
        EigenMatrixXd m3[4];
        m3[0] = eigen_malloc_matrix_double(3,3);
        m3[1] = eigen_malloc_matrix_double(3,3);
        m3[2] = eigen_malloc_matrix_double(3,3);
        m3[3] = eigen_malloc_matrix_double(3,3);

        GenerateXrotMatrix(xangle,eigen_matrix_data_double(m3[0]));
        GenerateYrotMatrix(yangle,eigen_matrix_data_double(m3[1]));
        GenerateZrotMatrix(zangle,eigen_matrix_data_double(m3[2]));

        eigen_matrix_multiply_double(m3[rotation_order_list[2]],m3[rotation_order_list[1]],m3[3]);
        eigen_matrix_multiply_double(m3[3],m3[rotation_order_list[0]],m3[rotation_order_list[2]]);

        memcpy(rotation_matrix,m3[rotation_order_list[2]],9*sizeof(double));

        eigen_free_matrix_double(m3[0]);
        eigen_free_matrix_double(m3[1]);
        eigen_free_matrix_double(m3[2]);
        eigen_free_matrix_double(m3[3]);
#else
        double m3[5][9];

        GenerateXrotMatrix(xangle,m3[0]);
        GenerateYrotMatrix(yangle,m3[1]);
        GenerateZrotMatrix(zangle,m3[2]);

        MatrixMultiply(m3[rotation_order_list[0]],m3[rotation_order_list[1]],m3[3],3,3,3);
        MatrixMultiply(m3[3],m3[rotation_order_list[2]],m3[4],3,3,3);

        memcpy(rotation_matrix,m3[4],9*sizeof(double));
#endif
    }

    GeoView::GeoView(){
        camera_ptr_ = new Camera;
    }
    GeoView::~GeoView(){
        if(camera_ptr_) delete camera_ptr_;
    }
    bool GeoView::set_camera_parameters(double focal_length, double x0, double y0, int width, int height){
        camera_ptr_->set_interior_parameters(focal_length,x0,y0);
        camera_ptr_->set_image_size(width,height);
        return true;
    }
    bool GeoView::set_exterior_parameters(double xs, double ys, double zs, double xangle, double yangle, double zangle, unsigned int rotation_order[3]){
        exterior_parameters_.set_position(xs,ys,zs);
        exterior_parameters_.set_orientation(xangle,yangle,zangle,rotation_order);
        return true;
    }
    bool GeoView::set_exterior_parameters(double xs, double ys, double zs, double rotation_matrix[9], bool bworldtocamera ){
        exterior_parameters_.set_position(xs,ys,zs);
        exterior_parameters_.set_rotation_matrix(rotation_matrix,bworldtocamera);
        return true;
    }
    void GeoView::WorldCoordsToPixelCoords(double wx,double wy, double wz, double* px, double* py){
        double cxyz[3];
        exterior_parameters_.WorldCoordsToCameraCoordVec(wx,wy,wz,cxyz,cxyz+1,cxyz+2);
        camera_ptr_->CameraCoordVecToPixelCoords(cxyz[0],cxyz[1],cxyz[2],px,py);
    }
    GeoView::Exterior::Exterior() :
        x_axis_angle_(0), y_axis_angle_(0), z_axis_angle_(0),
        xs_viewpoint_(0),ys_viewpoint_(0),zs_viewpoint_(0) {
        rotation_order_[0]=1; rotation_order_[1]=0; rotation_order_[2]=2;
        memcpy(rotation_matrix_,kIdentityMatrix,9*sizeof(double));
        memcpy(rotation_matrix_inv_,kIdentityMatrix,9*sizeof(double));
    };
    void GeoView::Exterior::set_rotation_matrix(double rotation_matrix[9], bool bworldtocamera){
        double *rot_mat, *rot_mat_inv;
        if(bworldtocamera) {
            rot_mat = rotation_matrix_; rot_mat_inv = rotation_matrix_inv_;
        }else{
            rot_mat = rotation_matrix_inv_; rot_mat_inv = rotation_matrix_;
        }
        memcpy(rot_mat,rotation_matrix,9*sizeof(double));

#ifdef _EIGEN
        EigenMatrixXd m1 = eigen_malloc_matrix_double(3,3);
        EigenMatrixXd m2 = eigen_malloc_matrix_double(3,3);
        memcpy(eigen_matrix_data_double(m1),rotation_matrix_,9*sizeof(double));
        eigen_matrix_transpose_double(m1,m2);
        memcpy(rot_mat_inv,eigen_matrix_data_double(m2),9*sizeof(double));
        eigen_free_matrix_double(m1);
        eigen_free_matrix_double(m2);
#else
        MatrixTranspose(rot_mat,rot_mat_inv,3,3);
#endif
    }
    void GeoView::Exterior::WorldCoordsToCameraCoordVec(double wx, double wy, double wz, double *cx, double *cy, double *cz){
        double obj_in_camera_coord[3] = {wx-xs_viewpoint_,wy-ys_viewpoint_,wz-zs_viewpoint_};
#ifdef _EIGEN

        EigenMatrixXd m = eigen_malloc_matrix_double(3,3);
        EigenVectorXd x = eigen_malloc_vector_double(3);

        memcpy(eigen_matrix_data_double(m),rotation_matrix_inv_,9*sizeof(double));
        memcpy(eigen_vector_data_double(x),obj_in_camera_coord,3*sizeof(double));

        eigen_matrix_multiply_double(m,x,x);
        double* data = eigen_vector_data_double(x);

        *cx = *data; *cy = *(data+1); *cz = *(data+2);

        eigen_free_matrix_double(m);
        eigen_free_vector_double(x);
#else
        double cxyz[3];
        MatrixMultiply(rotation_matrix_,obj_in_camera_coord,cxyz,3,3,1);
        *cx = cxyz[0]; *cy = cxyz[1]; *cz = cxyz[2];
#endif
    }
    void GeoView::Exterior::CameraCoordVecToWorldCoords(double cx, double cy, double cz, double wz, double *wx, double *wy){
#ifdef _EIGEN
        EigenMatrixXd m = eigen_malloc_matrix_double(3,3);
        EigenVectorXd x = eigen_malloc_vector_double(3);

        memcpy(eigen_matrix_data_double(m),rotation_matrix_,9*sizeof(double));
        double* data = eigen_vector_data_double(x);
        *data = cx; *(data+1) = cy; *(data+2) = cz;

        eigen_matrix_multiply_double(m,x,x);
        double wxyz[3];
        memcpy(wxyz,eigen_vector_data_double(x),3*sizeof(double));

        eigen_free_matrix_double(m);
        eigen_free_vector_double(x);
#else
        double cxyz[3] = {cx,cy,cz},wxyz[3];
        MatrixMultiply(rotation_matrix_inv_,cxyz,wxyz,3,3,1);
#endif
        double tmp = (wz-zs_viewpoint_)/ *(wxyz+2);
        *wx = *(wxyz)/tmp+xs_viewpoint_;
        *wy = *(wxyz+1)/tmp+ys_viewpoint_;
    }
}

#ifdef _EIGEN
#undef _EIGEN
#endif
