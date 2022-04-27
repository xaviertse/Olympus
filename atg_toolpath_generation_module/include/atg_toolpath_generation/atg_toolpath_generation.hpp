#ifndef ATG_TOOLPATH_GENERATION_HPP
#define ATG_TOOLPATH_GENERATION_HPP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <QtWidgets>

namespace ATG_toolpath_generation {
class ATG_TPG
{


  public:
    ATG_TPG();
    virtual ~ATG_TPG();

    int     external_on_      ;
    int     internal_on_      ;
    float   resolution_       ;
    float   step_size_        ;
    float   offset_           ;
    float   path_rotate_      ;
    int     downsample_       ;
    int     ksearch_tp_       ;
    int     normal_flip_      ;
    float   section_range_min_;
    float   section_range_max_;
    int     reverse_toolpath_ ;
    int     forced_resolution_;
    float   hole_patch_size_  ;
    float   lift_height_=50   ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ReadTXT(std::string file_name);
    Eigen::Matrix4f Bounding_Box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Detect_Boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    int zigzag_toolpath(std::string filename);//input params to be filled, should return PointCloud data and Normals
    int Point(std::string filename);//input params to be filled, should return PointCloud data and Normals
    int contour_toolpath(std::string filename);//input params to be filled, should return PointCloud data and Normals
    void emit_signal (std::string signal_name, std::string msg = "");
//    int Meridian_Masking                  (PointCloud<PointXYZ>::Ptr cloud, int External_on, int Internal_on, double zoom_size, Eigen::Vector4f centroid = {0,0,0,0});//xavier changes to user hole selection
//    int Chuck_Masking                  (PointCloud<PointXYZ>::Ptr cloud, int External_on, int Internal_on, double chuck_speed, double feedrate, double zoom_size, Eigen::Vector4f centroid = {0,0,0,0});//xavier changes to user hole selection
//    int Load_Point_Cloud_Smoothen_Boundary(PointCloud<PointXYZ>::Ptr cloud, double zoom_size, Eigen::Vector4f centroid = {0,0,0,0});
//    int Contour_Masking                   (PointCloud<PointXYZ>::Ptr cloud, double zoom_size, Eigen::Vector4f centroid = {0,0,0,0});
//    int Zigzag_Masking                    (PointCloud<PointXYZ>::Ptr cloud, double zoom_size, Eigen::Vector4f centroid = {0,0,0,0});
//    int Spiral_Masking                    (PointCloud<PointXYZ>::Ptr cloud, double zoom_size, Eigen::Vector4f centroid = {0,0,0,0});
//    int Zigzag_Hole                       (PointCloud<PointXYZ>::Ptr cloud, double zoom_size, double path_rotate,int ksearch_tp, Eigen::Vector4f centroid = {0,0,0,0}, bool normal_flip = 0);

//  signals:
//    void progressUpdated(int value);
};
}
#endif // ATG_TOOLPATH_GENERATIONE_HPP
