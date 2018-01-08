#include "lx_camera.h"

namespace xlingeo{
    Camera::Camera():
        focal_length_(0), pixel_size_(0),
        x0_(0), y0_(0),
        width_(0), height_(0),
        x0_in_pixelcoord_(0), y0_in_pixelcoord_(0){
        x_axis_towards_ = 1; y_axis_towards_ = -1;
    }
    Camera::Camera(PixelCoordOrigin pixel_coord_origin){
        Camera::Camera();
        switch(pixel_coord_origin){
        case kLeftUp:
            x_axis_towards_ = 1; y_axis_towards_ = -1; break;
        case kLeftDown:
            x_axis_towards_ = 1; y_axis_towards_ = 1; break;
        case kRightDown:
            x_axis_towards_ = -1; y_axis_towards_ = 1; break;
        case kRightUp:
            x_axis_towards_ = -1; y_axis_towards_ = -1; break;
        }
    };
}
