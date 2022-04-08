#ifndef ATG_SURFACE_IDENTIFICATION_HPP
#define ATG_SURFACE_IDENTIFICATION_HPP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace ATG_surface_identification {


class ATG_SURF_ID
{
public:
    ATG_SURF_ID();
    virtual ~ATG_SURF_ID();

    double param_smoothness     = 4.0;
    double param_curvature      = 1.0;
    double param_minClusteSize  = 100;
    double param_KSearch        = 7.0;
    double param_neighbour      = 50.0;

    void segmentation(std::string filename);
    pcl::PointCloud<pcl::PointXYZ>::Ptr  load_point_cloud_data(std::string file_name, bool make_mini=0);
    void emit_signal (std::string signal_name, std::string msg = "");

protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_;

private:

};
}
#endif // ATG_SURFACE_IDENTIFICATION_HPP
