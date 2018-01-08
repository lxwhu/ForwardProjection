#ifndef LX_GEOVIEW_H
#define LX_GEOVIEW_H

namespace xlingeo{

    void CalcRotationMatrix(double xangle, double yangle, double zangle, unsigned int rotation_order_list[3], double* rotation_matrix);

    class GeoView{
    public:
        GeoView();
        virtual ~GeoView();

        bool set_camera_parameters(double focal_length, double x0, double y0, int width, int height);
        bool set_exterior_parameters(double xs, double ys, double zs, double xangle, double yangle, double zangle, unsigned int rotation_order[3]);

        void WorldCoordsToPixelCoords(double wx,double wy, double wz, double* px, double* py);
    protected:
        class Exterior{
        public:
            Exterior();

            void set_position(double xs, double ys, double zs){
                xs_viewpoint_ = xs; ys_viewpoint_ = ys; zs_viewpoint_ = zs;
            }
            void set_orientation(double xangle, double yangle, double zangle, unsigned int rotation_order[3]){
                x_axis_angle_ = xangle; y_axis_angle_ = yangle; z_axis_angle_ = zangle;
                rotation_order_[0] = rotation_order[0];
                rotation_order_[1] = rotation_order[1];
                rotation_order_[2] = rotation_order[2];
                UpdateRotationMatrix();
            }

            double x_axis_angle() const  { return x_axis_angle_; }
            double y_axis_angle() const  { return y_axis_angle_; }
            double z_axis_angle() const  { return z_axis_angle_; }
            double xs_viewpoint() const  { return xs_viewpoint_; }
            double ys_viewpoint() const  { return ys_viewpoint_; }
            double zs_viewpoint() const  { return zs_viewpoint_; }

            void x_axis_angle(int a) { x_axis_angle_ = a; }
            void y_axis_angle(int a) { y_axis_angle_ = a; }
            void z_axis_angle(int a) { z_axis_angle_ = a; }
            void xs_viewpoint(int a) { xs_viewpoint_ = a; }
            void ys_viewpoint(int a) { ys_viewpoint_ = a; }
            void zs_viewpoint(int a) { zs_viewpoint_ = a; }

            void UpdateRotationMatrix();

            void WorldCoordsToCameraCoordVec(double wx, double wy, double wz, double *cx, double *cy, double *cz);
            void CameraCoordVecToWorldCoords(double cx, double cy, double cz, double wz, double *wx, double *wy);
        private:
            double x_axis_angle_;
            double y_axis_angle_;
            double z_axis_angle_;

            double xs_viewpoint_;
            double ys_viewpoint_;
            double zs_viewpoint_;

            unsigned int rotation_order_[3];

            double rotation_matrix_[9];
            double rotation_matrix_inv_[9];
        } exterior_parameters_;
    private:
        class Camera* camera_ptr_;
    };
}

#endif
