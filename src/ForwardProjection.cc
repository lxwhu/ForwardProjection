#include "lx_geoview.h"

#include <string>
#include <vector>
#include <limits>
#include <iostream>
#include <iomanip>
#include <fstream>
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

typedef struct tagImagePoint{
    char name[128];
    char imagename[128];
    double c;
    double r;
}ImagePoint;

static inline const char* strfilename(const char* path){
    const char* pl = strrchr(path,'\\');
    const char* pr = strrchr(path,'/');
    if(pl){
        if(!pr) return pl;
        return pl>pr?pl:pr;
    }
    return pr;
}

#ifdef _WIN32
#define ISDIR(statbuf) (statbuf.st_mode & _S_IFDIR) != 0
#define ISREG(statbuf) (statbuf.st_mode & _S_IFREG) != 0;
#else
#define ISDIR(statbuf) S_ISDIR(statbuf.st_mode)
#define ISREG(statbuf) S_ISREG(statbuf.st_mode)
#endif


static inline int IsDirectory(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0)
        return 0;
    return ISDIR(statbuf);
}
static inline int IsFile(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0)
        return 0;
    return ISREG(statbuf);
}

static inline bool IsExist (const char* name) {
    struct stat buffer;
    return (stat (name, &buffer) == 0);
}

static inline char* AddPathMark(char* path){
    int length = strlen(path);
    char* p = path+length-1;
    if(*p=='/'||*p=='\\') return p;
    *(p+1) = '/'; *(p+2)=0;
    return p+1;
}

#ifdef _WIN32
#include <windows.h>
static inline bool CreateDir(const char* path){
    WIN32_FIND_DATA fd; HANDLE hFind = ::FindFirstFile(path,&fd);
    if ( hFind!=INVALID_HANDLE_VALUE ){ ::FindClose(hFind); ::CreateDirectory(path,NULL); return true; }
    char strPath[512]; strcpy( strPath,path);
    char* pSplit = (char*)strfilename(strPath);
    if ( !pSplit ) return true; else *pSplit = 0;
    if ( !CreateDir(strPath) ) return false;
    return ::CreateDirectory(path,NULL);
}
#else
static inline bool CreateDir(const char* path){
    char strPath[512],*pSplit;
    if ( IsExist(path) ) return true;
    pSplit = (char*)strfilename(strPath);

    if ( !pSplit ) return true; else *pSplit = 0;
    if ( !CreateDir(strPath) ) return false;
    return (mkdir( path,S_IRWXU )==0);
}
#endif

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
DEFINE_string(backwards, "", "apply backwards projection to image points");

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
    double degree_to_arc = 1; if(bangledegree) degree_to_arc = 3.1415926/180;
    FILE* fp = fopen(lpstrFilePath,"r"); if(!fp) return false;

    char strline[512]; PosData data;
    if (camera_tag) {
        while(1){
            fgets(strline,512,fp); if(feof(fp)) break;
            if( sscanf(strline,"%s%lf%lf%lf%lf%lf%lf",data.name, &data.xs,&data.ys,&data.zs,&data.xangle,&data.yangle,&data.zangle) < 7) continue;
            if(!strstr(data.name,camera_tag)) continue;
            pos_data.push_back(data);
        }
    }else{
        while(1){
            fgets(strline,512,fp); if(feof(fp)) break;
            if( sscanf(strline,"%s%lf%lf%lf%lf%lf%lf",data.name, &data.xs,&data.ys,&data.zs,&data.xangle,&data.yangle,&data.zangle) < 7) continue;
            pos_data.push_back(data);
        }
    }
    fclose(fp);

    int sz = (int) pos_data.size();
    PosData* pData = pos_data.data();
    for (int i = 0; i < sz; pData++,++i) {
        pData->xangle = pData->xangle*degree_to_arc;
        pData->yangle = -pData->yangle*degree_to_arc;
        pData->zangle = pData->zangle*degree_to_arc;
    }
    return pos_data.size()>0;
}
static bool ReadPoint3dFile(const char* lpstrFilePath, std::vector<Point3D>& point3d){
    FILE* fp = fopen(lpstrFilePath,"r"); if(!fp) return false;
    char strline[512]; Point3D data;
    while(1){
        fgets(strline,512,fp); if(feof(fp)) break;
        if( sscanf(strline,"%s%lf%lf%*lf%*lf%*lf%*lf%*lf%lf",data.name, &data.y,&data.x,&data.z) < 4) continue;
//        if( sscanf(strline,"%s%lf%lf%lf",data.name, &data.x,&data.y,&data.z) < 4) continue;
//        data.z = 30;
        point3d.push_back(data);
    }

    fclose(fp);
    return point3d.size()>0;
}

static bool ReadImagePointsFile(const char* lpstrFilePath,std::vector<ImagePoint>& imagepoint ){
    FILE* fp = fopen(lpstrFilePath,"r"); if(!fp) return false;
    char strline[512]; ImagePoint data;
    while(1){
        fgets(strline,512,fp); if(feof(fp)) break;
        if(strline[0]!=' '&&strline[0]!='\t'){
            if( sscanf(strline,"%s%s%*lf%*lf%lf%lf",data.imagename,data.name, &data.c,&data.r) < 4) continue;
        }else{
            if( sscanf(strline,"%s%*lf%*lf%lf%lf",data.name, &data.c,&data.r) < 3) continue;
        }
        imagepoint.push_back(data);
    }

    fclose(fp);
    return imagepoint.size()>0;
}

static inline PosData* FindPosData(std::vector<PosData>& pos_data, const char* imagename){
    int sz = pos_data.size();
    PosData* pData = pos_data.data();
    for (int i = 0; i < sz;pData++, ++i)
    {
        if(!strcmp(imagename,pData->name)) return pData;
    }
    return NULL;
}
static inline Point3D* FindPoint3D(std::vector<Point3D>& point3d, const char* name){
    int sz = point3d.size();
    Point3D* pData = point3d.data();
    for (int i = 0; i < sz;pData++, ++i)
    {
        if(!strcmp(name,pData->name)) return pData;
    }
    return NULL;
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

    char point_file[512] = "";
    if ( argc<2 || !IsFile(argv[1]))
    {
        LOG(ERROR) << "No input 3d point file!";
        return 1;
    }
    strcpy(point_file,argv[1]);

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
    std::vector<ImagePoint> image_point;
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

    int pointnum = 0;
    VLOG(1)<< "Reading 3D points file ...";
    if (!ReadPoint3dFile(point_file, point3d))
    {
        LOG(ERROR)<< "Cannot extract 3D points information from "<< point_file;
        return 1;
    }
    pointnum = (int)point3d.size();
    VLOG(1)<< "3D points num = "<< pointnum;
    if(!FLAGS_backwards.empty()){
        VLOG(1)<< "Reading image points file ...";
        if (!ReadImagePointsFile(FLAGS_backwards.c_str(), image_point)) {
            LOG(ERROR)<< "Cannot extract image points information from "<< point_file;
            return 1;
        }
        pointnum = (int)image_point.size();
        VLOG(1)<< "image points num = "<< pointnum;
    }

    char output_filepath[512];
    const char* lpstroutpath = FLAGS_output.c_str(); char* output_filename = NULL;
    std::ofstream output_file; std::ostream* out_info = &std::cout; std::ios_base::openmode file_mode = std::ofstream::out | std::ofstream::app;
    if( strlen(lpstroutpath)>0 ) {
        if( IsDirectory(lpstroutpath) ){
            strcpy(output_filepath,lpstroutpath); lpstroutpath = output_filepath;
            output_filename =  AddPathMark(output_filepath)+1;
        }else{
            output_file.open(lpstroutpath,file_mode);
            if(output_file.is_open()){
                out_info = &output_file;
            }else lpstroutpath = NULL;
        }
    }else{ lpstroutpath = NULL; }
    VLOG(1)<< "output type = "<< std::string(lpstroutpath?(output_filename?"directory":"file"):"cout");

    if(FLAGS_backwards.empty()){
        VLOG(1)<< "Forward project all 3D points to images ...";
        PosData* pData = pos_data.data();
        for (int i = 0; i < pos_data_size ; pData++, ++i)
        {
            if(output_filename) {
                sprintf(output_filename,"%s.txt",pData->name);
                output_file.open(lpstroutpath,file_mode);
                if(output_file.is_open()){
                    out_info = &output_file;
                }else out_info = &std::cout;
            }
            else *out_info<< "["<<pData->name<<"]"<<std::endl;

            int pt_count = 0;
            double rot_mat[9];
            xlingeo::CalcRotationMatrix(pData->xangle,pData->yangle, pData->zangle, PosData::rotation_order, rot_mat);
            geoview.set_exterior_parameters(pData->xs,pData->ys,pData->zs, rot_mat, false);
            Point3D* pPoint = point3d.data();
            for (int j = 0; j < pointnum;pPoint++, ++j)
            {
                double px, py;
                geoview.WorldCoordsToPixelCoords(pPoint->x,pPoint->y,pPoint->z,&px,&py);
                if (px>-tolerance_distance&&px<width+tolerance_distance
                    &&py>-tolerance_distance&&py<height+tolerance_distance) {
                    *out_info<< pPoint->name<<"\t\t"<< px<< "\t\t"<< height-1 - py<< "\t\t"<<py <<std::endl;
                    pt_count++;
                }
            }
            if(output_filename) {
                output_file.close();
                if(pt_count==0) std::remove(output_filepath);
            }
        }
    }else{
        VLOG(1)<< "Backward project all image points to world ...";
        ImagePoint* pData = image_point.data();
        for (int i = 0; i < pointnum; pData++,++i)
        {
            double x,y;
            PosData* pPos = FindPosData(pos_data,pData->imagename);
            Point3D* pPoint = FindPoint3D(point3d,pData->name);
            double rot_mat[9];
            xlingeo::CalcRotationMatrix(pPos->xangle,pPos->yangle, pPos->zangle, PosData::rotation_order, rot_mat);
            geoview.set_exterior_parameters(pPos->xs,pPos->ys,pPos->zs, rot_mat, false);
            geoview.PixelCoordsToWorldCoords(pData->c,pData->r,pPoint->z,&x,&y);
            out_info->setf(std::ios::fixed);
            *out_info << std::setprecision(3) << pData->imagename << "\t" << pData->name << "\t"
                     << pPoint->x << "\t\t"<< pPoint->y << "\t\t" << x << "\t\t" << y<<std::endl;
        }
    }

    return 0;
}

#undef ISDIR
#undef ISREG
