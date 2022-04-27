#include <include/atg_surface_identification/atg_surface_identification.hpp>
#include "../include/atg_surface_identification/converter.hpp"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/kdtree/kdtree.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

namespace ATG_surface_identification
{
  ATG_SURF_ID::ATG_SURF_ID(){}
  ATG_SURF_ID::~ATG_SURF_ID(){}

  void ATG_SURF_ID::segmentation(std::string filename)
  {
    float emit_percent = 0;
    //emit 10%
    emit_signal ("segmentation_signal", "percent\n"+std::to_string(emit_percent));
    //reset cluster folder
    system("rosrun atg_surface_identification remove_files.py data/coupons/clusters");//may need to cmake py into devel for deployment
    //system("python src/atg_surface_identification_module/scripts/remove_files.py data/coupons/clusters");
    std::cout << "Removing the old cluster files....\n";

    //begin loading cloud
    g_cloud_=load_point_cloud_data(filename);
    Eigen::Vector4f centroid;              //josh added to cal centroid into array
    pcl::compute3DCentroid(*g_cloud_, centroid);    //josh added to cal centroid into array
    //emit 10%
    emit_percent = 10;
    emit_signal ("segmentation_signal", "percent\n"+std::to_string(emit_percent));

    //compute normal and region growing to get clusters
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator; //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//original normalestimator function
    normal_estimator.setNumberOfThreads (12);//new for OMP function, speeds up calculation
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (g_cloud_);
    normal_estimator.setKSearch (param_KSearch);//50
   // normal_estimator.setNeighbour (param_Neighbour);//50
    normal_estimator.setViewPoint (centroid[0], centroid[1], centroid[2]);
    std::cout<<"Normal estimation Centroid position for segmentation=[" + std::to_string(centroid[0])+","+std::to_string(centroid[1])+","+std::to_string(centroid[2])+"]\n";
    normal_estimator.compute (*normals);

    //pcl::IndicesPtr indices (new std::vector <int>);
    //pcl::PassThrough<pcl::PointXYZ> pass;
    //pass.setInputCloud (cloud_input);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits (0.0, 1.0);
    //pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (param_minClusteSize);//reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (param_neighbour);//100
    reg.setInputCloud (g_cloud_);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold(param_smoothness/180.0 * M_PI); //reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(param_curvature);//reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    //emit progressUpdated(50);
    emit_percent = 60;
    emit_signal ("segmentation_signal", "percent\n"+std::to_string(emit_percent));

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

    //extract clusters individually and save
    for (int i = 0; i < clusters.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
      for (int j = 0; j < clusters[i].indices.size(); j++)
      {
        pcl::PointXYZ p;
        p.x = g_cloud_->points[clusters[i].indices[j]].x;
        p.y = g_cloud_->points[clusters[i].indices[j]].y;
        p.z = g_cloud_->points[clusters[i].indices[j]].z;
        temp->points.push_back(p);
      }
      temp->width = temp->points.size();
      temp->height = 1;

      pcl::PointCloud<pcl::PointNormal>::Ptr cluster(new pcl::PointCloud<pcl::PointNormal>);
      for (int j = 0; j < clusters[i].indices.size(); j++)
      {
        pcl::PointNormal p;
        p.x = g_cloud_->points[clusters[i].indices[j]].x;
        p.y = g_cloud_->points[clusters[i].indices[j]].y;
        p.z = g_cloud_->points[clusters[i].indices[j]].z;
        p.normal_x = normals->points[clusters[i].indices[j]].normal_x;
        p.normal_y = normals->points[clusters[i].indices[j]].normal_y;
        p.normal_z = normals->points[clusters[i].indices[j]].normal_z;
        p.curvature = normals->points[clusters[i].indices[j]].curvature;
        cluster->points.push_back(p);
      }

      std::cout << "cluster " + std::to_string(i) + " has " << cluster->points.size() << " points." << std::endl;
      cluster->width = cluster->points.size();
      cluster->height = 1;
      if(!cluster->empty())
      {
        pcl::io::savePCDFile("data/coupons/clusters/cluster_"+std::to_string(i)+".pcd", *cluster);
        //pcl::io::savePCDFileASCII ("data/coupons/clusters/cluster_normal_"+std::to_string(i)+".pcd", *cluster);
      }

      emit_signal ("segmentation_signal", "percent\n"+std::to_string(emit_percent+20*(i/float(clusters.size()))));
      //system("python src/scripts/edit_pcd.py data/plane_surfaces.pcd");

    }

    //save rejected points as last cluster....start, extract both points and normals into PointNormal for saving
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_remain_n(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr              inliers_r(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ>  extract;
    pcl::ExtractIndices<pcl::Normal>  extract_n;
    for(int i = 0; i<clusters.size();i++) //append all indices from cluster into
    {
      pcl::PointIndices::Ptr inliers_rc(new pcl::PointIndices( clusters[i]));
      for (int j = 0;j<inliers_rc->indices.size();j++)
      {
        inliers_r->indices.push_back(inliers_rc->indices[j]);
      }
    }
    extract.setInputCloud (g_cloud_);
    extract.setIndices    (inliers_r);
    extract.setNegative   (true);
    extract.filter        (*cloud_remain);
    extract_n.setInputCloud (normals);
    extract_n.setIndices    (inliers_r);
    extract_n.setNegative   (true);
    extract_n.filter        (*cloud_remain_n);
    pcl::PointCloud<pcl::PointNormal>::Ptr cluster_n(new pcl::PointCloud<pcl::PointNormal>);
    for (int j = 0; j < cloud_remain->size(); j++)
    {
      pcl::PointNormal p;
      p.x = cloud_remain->points[j].x;
      p.y = cloud_remain->points[j].y;
      p.z = cloud_remain->points[j].z;
      p.normal_x  = cloud_remain_n->points[j].normal_x;
      p.normal_y  = cloud_remain_n->points[j].normal_y;
      p.normal_z  = cloud_remain_n->points[j].normal_z;
      p.curvature = cloud_remain_n->points[j].curvature;
      cluster_n->points.push_back(p);
    }
    cluster_n->width = cluster_n->points.size();
    cluster_n->height = 1;
    if(!cloud_remain->empty())
    {
      pcl::io::savePCDFile("data/coupons/clusters/rejected_points.pcd", *cluster_n);//cluster_"+std::to_string(clusters.size())
    }
    std::cout << "remaining cluster has " << cluster_n->points.size() << " points." << std::endl;
    emit_percent=80;
    emit_signal ("segmentation_signal", "percent\n"+std::to_string(emit_percent));
    //save rejected points as last cluster....end

    g_cloud_->clear();   //clears memory
    g_cloud_.reset();    //clears memory
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ATG_SURF_ID::load_point_cloud_data(std::string file_name,bool make_mini)
  {
    std::string src_filename, dst_filename;
    std::string tmp_pcd_filename = "data/coupons/tmp.pcd";   //josh added for sharefile write error, pcd cannot write w filelock
    std::string cmd, output_dir;
    Converter Conv_file_format;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZ>);


    std::cout <<"---------------------------------------------------------------------------\n";
    std::cout <<"-------------------------Load Point Cloud From File------------------------\n";
    std::cout <<"---------------------------------------------------------------------------\n";
    //ui.progressBar->setValue(3); ui.progressBar->update();

    src_filename = file_name;
    size_t f = src_filename.find(".txt");
    if (src_filename.substr(src_filename.find_last_of(".") + 1) == "obj")//input file is OBJ, read and convert into temp pcd
    {
      Conv_file_format.obj2pcd(src_filename,tmp_pcd_filename);
      if ( pcl::io::loadPCDFile <pcl::PointXYZ> (tmp_pcd_filename, *cloud_origin) == -1)
      {
        std::cout << "Failed to read OBJ file from: " << src_filename << std::endl;
        exit(1);
      }
      std::cout <<"visualize the raw point cloud data" << std::endl;
    }
    else if(src_filename.substr(src_filename.find_last_of(".") + 1) == "pcd")//input is PCD, just read
    {
      if ( pcl::io::loadPCDFile <pcl::PointXYZ> (src_filename, *cloud_origin) == -1)
      {
        std::cout << "Failed to read PCD file from: " << src_filename << std::endl;
        exit(1);
      }
      std::cout <<"visualize the raw point cloud data" << std::endl;
    }
    else
    {
      std::cout << "Cloud reading failed. Wrong file format" << src_filename << std::endl;
      exit(1);
    }
    pcl::io::savePCDFileASCII <pcl::PointXYZ> ("coupon_whole.pcd", cloud_origin);
    return (cloud_origin);
  }

  void ATG_SURF_ID::emit_signal (std::string signal_name, std::string msg)
  {
    std::ofstream fsignal;
    fsignal.open("data/coupons/"+signal_name+".txt",std::ofstream::out);
    fsignal << msg ;
    fsignal.close();
  }

}
