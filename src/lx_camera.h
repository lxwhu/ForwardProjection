#ifndef LX_CAMERA_H
#define LX_CAMERA_H

namespace xlingeo{
    class Camera
    {
    public:
        Camera();
        enum PixelCoordOrigin{kLeftUp,kLeftDown,kRightUp,kRightDown};
        Camera( PixelCoordOrigin pixel_coord_origin);
        virtual ~Camera(){};

        int width() const { return width_; }
        int height() const { return height_; }
        double focal_length() const { return focal_length_; }

        void set_interior_parameters(double focal_length, double x0, double y0){
            focal_length_ = focal_length; x0_ = x0; y0_ = y0;
            UpdatePrincipalpointInPixelcoord();
        }

        void set_image_size(int width, int height){
            width_ = width; height_ = height;
            UpdatePrincipalpointInPixelcoord();
        }

        void FilmCoordsToPixelCoords(double fx, double fy, double* px, double* py){
            *px = x_axis_towards_*fx + x0_in_pixelcoord_;
            *py = y_axis_towards_*fy + y0_in_pixelcoord_;
        };
        void PixelCoordsToFilmCoords(double px, double py, double* fx, double* fy){
            *fx = x_axis_towards_*(px-x0_in_pixelcoord_);
            *fy = y_axis_towards_*(py-y0_in_pixelcoord_);
        }

        void CameraCoordVecToFilmCoords(double cx, double cy, double cz, double* fx, double* fy){
            *fx = -focal_length_*cx/cz;
            *fy = -focal_length_*cy/cz;
        }
        void FileCoordsToCameraCoordVec(double fx, double fy, double *cx, double *cy, double *cz){
            *cx = fx; *cy = fy; *cz = -focal_length_;
        }

        void PixelCoordsToCameraCoordVec(double px, double py, double* cx, double* cy, double* cz){
            PixelCoordsToFilmCoords(px,py,cx,cy);
            FileCoordsToCameraCoordVec(*cx,*cy,cx,cy,cz);
        }
        void CameraCoordVecToPixelCoords(double cx, double cy, double cz, double* px, double* py){
            CameraCoordVecToFilmCoords(cx,cy,cz,px,py);
            FilmCoordsToPixelCoords(*px,*py,px,py);
        }

        void UpdatePrincipalpointInPixelcoord(){
            x0_in_pixelcoord_ = CalcPpInPc(x0_,width_>>1,x_axis_towards_);
            y0_in_pixelcoord_ = CalcPpInPc(y0_,height_>>1,y_axis_towards_);
        }
    protected:
        double CalcPpInPc(double pp_to_origin, double origin_to_pc, int axis_towards){
            return origin_to_pc+axis_towards*pp_to_origin;
        }
    private:
        double focal_length_;
        double pixel_size_;
        double x0_;
        double y0_;
        int width_;
        int height_;

        int x_axis_towards_;
        int y_axis_towards_;
        double x0_in_pixelcoord_;
        double y0_in_pixelcoord_;
    };
}

#endif
