#include "lx_geoview.h"

#include <string>
#include <vector>
#include <limits>
#include <sys/stat.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

typedef struct tagPosData{
    char name[128];
    double xs; double ys; double zs;
    double xangle; double yangle; double zangle;
    static unsigned int rotation_order[3];

}PosData;
unsigned int PosData::rotation_order[3] = {1,0,2};

typedef struct tagPoint3D{
    char name[128];
    double x; double y; double z;
}Point3D;

static inline const char* strfilename(const char* path){
    const char* pl = strrchr(path,'\\');
    const char* pr = strrchr(path,'/');
    if(pl){
        if(!pr) return pl;
        return pl>pr?pl:pr;
    }
    return pr;
}

static inline int IsDirectory(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0)
        return 0;
#ifdef _WIN32
    return (statbuf.st_mode & _S_IFDIR) != 0;
#else
    return S_ISDIR(statbuf.st_mode);
#endif
}
static inline int IsFile(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0)
        return 0;
#ifdef _WIN32
    return (statbuf.st_mode & _S_IFREG) != 0;
#else
    return S_ISREG(statbuf.st_mode);
#endif
}

static inline bool IsExist (const char* name) {
    struct stat buffer;
    return (stat (name, &buffer) == 0);
}

static bool ValidateFile(const char* flagname, const std::string& value) {
    if (IsExist(value.c_str()))
    {
        return true;
    }
    printf("Invalid file for --%s: %s\n", flagname, value.c_str());
    return false;
}
DEFINE_string(camera_file, "",
              "camera file including interior parameters");
DEFINE_validator(camera_file, &ValidateFile);
DEFINE_string(pos_file, "",
              "POS file including GPS and IMU");
DEFINE_validator(pos_file, &ValidateFile);
DEFINE_string(output, "",
              "file/directory for forward projection informations ");
DEFINE_string(camera_tag, "*",
              "camera tag shared by POS data ID. The default tag is the first character of camera file name");

static bool ValidateRotationOrder(const char* flagname, const std::string& value) {
    const char* p = value.c_str();
    int bFlags[3] = {0};
    for (int i = 0; i < 3; p++, ++i)
    {
        int t = 0;
        if(*p>='x'&&*p<='z') t = int(*p-'x');
        else if(*p>='X'&&*p<='Z') t = int(*p-'X');
        else {
           printf("Invalid label for --%s: %s\n", flagname, p);
           return false;
        }
        PosData::rotation_order[i] = t; bFlags[t] = 1;
    }
    if(!bFlags[0]||!bFlags[1]||!bFlags[2]){
        printf("Incomplete information for --%s: %s\n", flagname, p);
        return false;
    }
    return true;
}
DEFINE_string(rotation_order, "yxz",
              "camera rotation order");
DEFINE_validator(rotation_order, &ValidateRotationOrder);

static bool ValidateSize(const char* flagname, GFLAGS_NAMESPACE::int32 value) {
    if (value > 0)
    {
        return true;
    }
    printf("Invalid value for --%s: %d\n", flagname, (int)value);
    return false;
}
// DEFINE_int32(width,0,
//              "image width");
// DEFINE_validator(width, &ValidateSize);
// DEFINE_int32(height,0,
//              "image height");
// DEFINE_validator(height, &ValidateSize);
DEFINE_int32(tolerance, std::numeric_limits<GFLAGS_NAMESPACE::int32>::max(),
             "tolerance distance from image boundary");

DEFINE_bool(angle_degree, true, "the angle type of the POS data");

static bool ReadCameraFile(
    const char* lpstrFilePath,
    double* focal_length, double* x0, double* y0,
    int* width, int* height
    ){
    FILE* fp = fopen(lpstrFilePath,"r");
    if(!fp) return false;

    char strline[512];

    fgets(strline,512,fp);
    if( sscanf(strline, "%lf%lf%lf%d%d",focal_length, x0, y0, width, height) < 5) { fclose(fp); return false; }

    fclose(fp);
    return true;
}

static bool ReadPosFile(const char* lpstrFilePath, std::vector<PosData>& pos_data, const char* camera_tag, bool bangledegree){
    const double degree_to_arc = 3.1415926/180;
    FILE* fp = fopen(lpstrFilePath,"r"); if(!fp) return false;

    char strline[512]; PosData data;
    if (camera_tag) {
        do{
            fgets(strline, 512, fp);
            if( sscanf(strline,"%s%lf%lf%lf%lf%lf%lf",data.name, &data.xs,&data.ys,&data.zs,&data.xangle,&data.yangle,&data.zangle) < 7) continue;
            if(!strstr(data.name,camera_tag)) continue;
            if(bangledegree) {
                data.xangle *= degree_to_arc;
                data.yangle *= degree_to_arc;
                data.zangle *= degree_to_arc;
            }
            pos_data.push_back(data);
        }while(!feof(fp));
    }else{
        do{
            fgets(strline, 512, fp);
            if( sscanf(strline,"%s%lf%lf%lf%lf%lf%lf",data.name, &data.xs,&data.ys,&data.zs,&data.xangle,&data.yangle,&data.zangle) < 7) continue;
            if(bangledegree) {
                data.xangle *= degree_to_arc;
                data.yangle *= degree_to_arc;
                data.zangle *= degree_to_arc;
            }
            pos_data.push_back(data);
        }while(!feof(fp));
    }
    fclose(fp);
    return pos_data.size()>0;
}
static bool ReadPoint3dFile(const char* lpstrFilePath, std::vector<Point3D>& point3d){
    FILE* fp = fopen(lpstrFilePath,"r"); if(!fp) return false;
    char strline[512]; Point3D data;
    do{
        fgets(strline,512,fp);
        if( sscanf(strline,"%s%lf%lf%*lf%*lf%*lf%*lf%lf",data.name, &data.y,&data.x,&data.z) < 4) continue;
        point3d.push_back(data);
    }while(!feof(fp));

    fclose(fp);
    return point3d.size()>0;
}

int main(int argc, char *argv[])
{
    FLAGS_alsologtostderr = true;
    std::string usage("This program projects the 3D point to image.  Usage:\n");
    usage = usage + argv[0] + " <3D point file> ";
    gflags::SetUsageMessage(usage);
    gflags::SetVersionString("1.0.0");

    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    char point_3d_file[512] = "";
    if ( argc<2 || !IsFile(argv[1]))
    {
        LOG(ERROR) << "No input 3d point file!";
        return 1;
    }
    strcpy(point_3d_file,argv[1]);

    const char* lpstrcameratag = FLAGS_camera_tag.c_str();
    double tolerance_distance = FLAGS_tolerance;
    if (!strcmp(lpstrcameratag,"*")) {
        lpstrcameratag = NULL;
    }else if(strlen(lpstrcameratag)<1){
        lpstrcameratag = strfilename(FLAGS_camera_file.c_str());
        if(lpstrcameratag){
            FLAGS_camera_tag = *(lpstrcameratag+1);
            lpstrcameratag = FLAGS_camera_tag.c_str();
        }
    }

    VLOG(2)<< "camera tag = "<< FLAGS_camera_tag;
    VLOG(2)<< "tolerance distance = "<< FLAGS_tolerance;
    VLOG(2)<< "rotation order = "<< PosData::rotation_order[0] << "->" << PosData::rotation_order[1] << "->" << PosData::rotation_order[2];

    double focal_length, x0, y0;
    // int width = FLAGS_width; int height = FLAGS_height;
    int width, height;
    std::vector<PosData> pos_data;
    std::vector<Point3D> point3d;
    xlingeo::GeoView geoview;

    VLOG(1)<< "Reading camera file ...";
    if(!ReadCameraFile(FLAGS_camera_file.c_str(), &focal_length, &x0, &y0, &width, &height)){
        LOG(ERROR)<< "Cannot extract camera information from "<< FLAGS_camera_file;
        return 1;
    }
    VLOG(1)<< "camera interior (focal, x0, y0) = ("<< focal_length<< ", "<< x0<< ", "<< y0<<")";
    VLOG(1)<< "image size (width, height) = (" << width << ", "<< height<<")";
    geoview.set_camera_parameters(focal_length, x0, y0, width, height);

    VLOG(1)<< "Reading POS file ...";
    if (!ReadPosFile(FLAGS_pos_file.c_str(),pos_data, lpstrcameratag, FLAGS_angle_degree))
    {
        LOG(ERROR)<< "Cannot extract POS information form "<< FLAGS_pos_file;
        return 1;
    }
    int pos_data_size = (int)pos_data.size();
    VLOG(1)<< "POS num = "<< pos_data_size;

    VLOG(1)<< "Reading 3D point file ...";
    if (!ReadPoint3dFile(point_3d_file, point3d))
    {
        LOG(ERROR)<< "Cannot extract 3D points information form "<< point_3d_file;
        return 1;
    }
    int point3d_size = (int)point3d.size();
    VLOG(1)<< "3D point num = "<< point3d.size();

    PosData* pData = pos_data.data();
    for (int i = 0; i < pos_data_size ; pData++, ++i)
    {
        VLOG(0)<< "["<<pData->name<<"]";
        int pt_count = 0;
        geoview.set_exterior_parameters(pData->xs,pData->ys,pData->zs,pData->xangle,pData->yangle, pData->zangle, PosData::rotation_order);
        Point3D* pPoint = point3d.data();
        for (int j = 0; j < point3d_size;pPoint++, ++j)
        {
            double px, py;
            geoview.WorldCoordsToPixelCoords(pPoint->x,pPoint->y,pPoint->z,&px,&py);
            if (px>-tolerance_distance&&px<width+tolerance_distance
                &&py>-tolerance_distance&&py<height+tolerance_distance) {
                VLOG(0)<< pPoint->name<<"\t\t"<< px<< "\t\t"<<py;
                pt_count++;
            }
        }
    }

    return 0;
}
