#include <include/atg_toolpath_generation/atg_toolpath_generation.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
<<<<<<< HEAD
<<<<<<< HEAD

=======
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
>>>>>>> 9845d2bd0d7cd3b3d63106eb00234e3d05d9ac86
=======

>>>>>>> 07aca541621e5cba057452ac8435109ff36ed434
namespace ATG_toolpath_generation
{
  ATG_TPG::ATG_TPG(){}
  ATG_TPG::~ATG_TPG(){}

  pcl::PointCloud<pcl::PointXYZ>::Ptr ATG_TPG::ReadTXT(std::string file_name)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(file_name);
    if (!file.is_open())
    {
      std::cerr << "Cannot open file ..." << std::endl;
      system("pause");
      return cloud;
    }

    else
    {
      pcl::PointXYZ point;
      int col = 0;
      float f = 0; //The coordinate value
      std::string line; //1 line in the file
      std::array<float, 3> temp; //Temporary array to store 1 point worth of coordinates

      //Loop through every line in the file and process the values accordingly
      for (int i = 0; std::getline(file, line); i++)
      {
        if (line.empty()) continue;

        std::replace(line.begin(), line.end(), ';', ' ');

        std::istringstream iss(line);
        ////Get the float values then store it in the temp array for a single point
        while (iss >> f)
        {
          temp[col] = f;
          col++;
          if (col == 3) break;
        }

        point.x = temp[0];
        point.y = temp[1];
        point.z = temp[2];
        cloud->points.push_back(point); //Add that single point to the vector of points
        col = 0;
      }

      cloud->width = (int)cloud->points.size();
      cloud->height = 1;
    }

    return cloud;
  }

  Eigen::Matrix4f ATG_TPG::Bounding_Box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);

//j    // Get the minimum and maximum points of the transformed cloud.
//j    pcl::PointXYZ minPoint, maxPoint;
//j    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
//j    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

//j    // Final transform
//j    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
//j    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    pcl::io::savePCDFile("data/coupons/cloudPointsProjected.pcd", *cloudPointsProjected);
    //system("python src/scripts/edit_pcd.py data/coupons/cloudPointsProjected.pcd");
    system("rosrun atg_toolpath_generation pcd2txt.py data/coupons/cloudPointsProjected.pcd");
    std::cout << "Saving cloudPointsProjected.pcd done!\n";

    //find boundary points
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points(new pcl::PointCloud<pcl::PointXYZ>);
    boundary_points = Detect_Boundary(cloudPointsProjected);
//    std::cout << "boundary_points has "<< boundary_points->points.size() <<" points\n";

//    boundary_points->width = boundary_points->points.size();
//    boundary_points->height = 1;
    pcl::io::savePCDFile("data/coupons/transformed_boundarypoints.pcd", *boundary_points);
    system("rosrun atg_toolpath_generation pcd2txt.py data/coupons/transformed_boundarypoints.pcd");
    //system("python src/scripts/edit_pcd.py data/coupons/transformed_boundarypoints.pcd");
    std::cout << "Saving transformed_boundarypoints.pcd done!\n";
//j    system("python src/scripts/BoundingBox.py");   //generate the bounding box in 4 points

//j    pcl::PointCloud<pcl::PointXYZ>::Ptr BB_projected(new pcl::PointCloud<pcl::PointXYZ>);
//j    BB_projected = ReadTXT("data/coupons/BB_projected.txt");

//j    pcl::PointCloud<pcl::PointXYZ>::Ptr BB(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f projectionTransform_inverse;
    projectionTransform_inverse = projectionTransform.inverse();
//j    pcl::transformPointCloud(*BB_projected, *BB, projectionTransform_inverse); //transforming back to the original coordinate system
//j    pcl::io::savePCDFile("data/coupons/BB.pcd", *BB);
//j    system("python src/scripts/edit_pcd.py data/coupons/BB.pcd"); //save the boundary box with repect to the original coordinate system

    std::cout << "Returned BBox Matrix :\n" << projectionTransform_inverse << std::endl;
    return projectionTransform_inverse;



  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ATG_TPG::Detect_Boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // Calculate Normals
    std::cout << "Calculate Normals ...\n";
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(100);
    normal_estimator.compute(*normals);

    //calculate boundary;
    std::cout << "Calculate boundary points ..." << std::endl;
    pcl::PointCloud<pcl::Boundary> boundary;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est;
    boundary_est.setInputCloud(cloud);
    boundary_est.setInputNormals(normals);
    //boundary_est.setRadiusSearch(0.2);
    boundary_est.setKSearch(100);
    //boundary_est.setAngleThreshold(M_PI/4);
    boundary_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    boundary_est.compute(boundary);

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < cloud->points.size(); i++)
    {
     if (boundary[i].boundary_point == 1)
      {
        boundary_points->points.push_back(cloud->points[i]);
      }
    }
    boundary_points->width = boundary_points->points.size();
    boundary_points->height = 1;
    std::cout << "Computation finished." << std::endl;
    return boundary_points;
  }

<<<<<<< HEAD
<<<<<<< HEAD
=======
//-------------------------------------------------------------------------POINT-----------------------------------------------------------
  int ATG_TPG::Point(std::string filename)
  {

    // ===== 1. Load variables
    // ===== 2. Load cluster
    // ===== 3. Find cluster bounding box, and also transform and data into bounding box, saved in cloudPointsProjected.txt
    // ===== 4. Toolpath Generation, python script, saves toolpath within boundingbox into sampling_tool_path.txt
    // ===== 5. Loades saved toolpath and transform into actual location
    // ===== 6. Loads coupon surface into kdtreeFlann and check each point in toolpath
    // ===== 6.1 calculate for offset direction for each point
    // ===== 6.2 calculate for individual point's normal from coupon data, and flip for correction
    // ===== 6.3 calculate for individual point's quat and rpy

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2), cloud_origin (new pcl::PCLPointCloud2);

    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *cloud) == -1)
    {
      std::cout << "Failed to read PCD file from: " << filename << std::endl;
      exit(1);
    }
    //int sorted_on = 1;//permanent on now
    //double offset = 0;//superceded
    double depth = 0;

    // ===== 1. Load variables, done before this function call, this is just for displaying the variables
    // =======================
    std::cout << "external_on_      : " << std::to_string(external_on_      ) << std::endl;
    std::cout << "internal_on_      : " << std::to_string(internal_on_      ) << std::endl;
    std::cout << "resolution_       : " << std::to_string(resolution_       ) << std::endl;
    std::cout << "step_size_        : " << std::to_string(step_size_        ) << std::endl;
    std::cout << "offset_           : " << std::to_string(offset_           ) << std::endl;
    std::cout << "path_rotate_      : " << std::to_string(path_rotate_      ) << std::endl;
    std::cout << "downsample_       : " << std::to_string(downsample_       ) << std::endl;
    std::cout << "ksearch_tp_       : " << std::to_string(ksearch_tp_       ) << std::endl;
    std::cout << "normal_flip_      : " << std::to_string(normal_flip_      ) << std::endl;
    std::cout << "section_range_min_: " << std::to_string(section_range_min_) << std::endl;
    std::cout << "section_range_max_: " << std::to_string(section_range_max_) << std::endl;
    std::cout << "reverse_toolpath_ : " << std::to_string(reverse_toolpath_ ) << std::endl;
    std::cout << "forced_resolution_: " << std::to_string(forced_resolution_) << std::endl;
    std::cout << "hole_patch_size_  : " << std::to_string(hole_patch_size_  ) << std::endl;
    std::cout << "lift_height_      : " << std::to_string(lift_height_      ) << std::endl;


    std::string output_filename = "data/coupons/tool_path_back_projected_w_lift_n_trigger.txt";                           //final tool path file
    std::ofstream fout(output_filename);

    pcl::io::loadPCDFile ("data/coupons/coupon_whole.pcd", *cloud_origin);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_origin);
    sor.setLeafSize(2, 2, 2);
    sor.filter(*cloud_filtered_blob);
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_downsampled);
    // Calculate Normals
//    std::cout << "Calculate Normals ...\n";
//    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod(tree);
//    normal_estimator.setInputCloud(cloud_downsampled);
//    normal_estimator.setKSearch(100);
//    normal_estimator.compute(*normals);


    pcl::PointCloud<pcl::PointNormal>::Ptr toolpath_w_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointNormal waypoint_w_normal;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if(1)
        {
          float progresspct = 50+50*i/(cloud->points.size()-1);
          emit_signal ("toolpath_signal", "percent\n"+std::to_string(progresspct));
        }
         //Radial search for estimating neighborhood indices
         pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
         kdtree.setInputCloud (cloud_downsampled);
         pcl::PointXYZ searchPoint = cloud->points[i];
         float radius = 3;
         std::vector<int> pointIdxRadiusSearch;
         std::vector<float> pointRadiusSquaredDistance;
         kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

     //Compute normal of a single point
         //pcl::PointCloud<pcl::PointNormal>::Ptr toolpath_w_normal(new pcl::PointCloud<pcl::PointNormal>);
         float curvature;
         Eigen::Vector4f pt_normal;
         pcl::computePointNormal(*cloud_downsampled,pointIdxRadiusSearch,pt_normal,curvature);


    //     if ( kdtree.nearestKSearch (tool_path_back_projected->points[i], ksearch_tp_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
     //    {
          // float curvature;//1mm offset of envelope from selected points
      //     pcl::computePointNormal(*cloud_downsampled,pointIdxRadiusSearch,pt_normal,curvature);//(*target_surface,pointIdxRadiusSearch,pt_normal,curvature);
     //    }


         //pcl::PointNormal waypoint_w_normal;
         waypoint_w_normal.normal_x = pt_normal[0];//n.normal_x = normals->points[idx].normal_x;//josh replace for target pt normal compute
         waypoint_w_normal.normal_y = pt_normal[1];//n.normal_y = normals->points[idx].normal_y;//josh replace for target pt normal compute
         waypoint_w_normal.normal_z = pt_normal[2];
         waypoint_w_normal.x = cloud->points[i].x;
         waypoint_w_normal.y = cloud->points[i].y;
         waypoint_w_normal.z = cloud->points[i].z;

         //--------------------------code ------------xavier getting the quaternion-----------------------
         Eigen::Vector3d norm(waypoint_w_normal.normal_x, waypoint_w_normal.normal_y, waypoint_w_normal.normal_z);
         Eigen::Quaterniond q;
         q.setFromTwoVectors(Eigen::Vector3d(0,0,1), norm);
         q.normalize();
         //std::cout << "This normal" << norm.transpose() << std::endl;
         //std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec().transpose() << std::endl;
         Eigen::Matrix<double,3,3> rotationMatrix;
         rotationMatrix = q.toRotationMatrix();
         //std::cout << "This normal" << rotationMatrix << std::endl;
         //Eigen::AngleAxisd newAngleAxis(rotationMatrix);
         //Eigen::Vector3d euler = rotationMatrix.eulerAngles(0, 1, 2);

         //manual calculations, still valid
         Eigen::Vector3d angle;
         double roll  = atan2( rotationMatrix(2,1),rotationMatrix(2,2) );
         double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  );
         double yaw   = atan2( rotationMatrix(1,0),rotationMatrix(0,0) );
         Eigen::Vector3d quat;
         quat=q.vec();
         angle[0]=roll*180/(std::atan(1.0)*4);
         if (angle[0]>180){
         //   angle[0]=angle[0]-360;
         }
         angle[1]=pitch*180/(std::atan(1.0)*4);
         if (angle[1]>180){
         //  angle[1]=angle[1]-360;
         }
         angle[2]=yaw*180/(std::atan(1.0)*4);
         if (angle[2]>180){
           //angle[2]=angle[2]-360;
         }

         pcl::PointNormal waypoint_jump_w_normal;
         //float lift_height_ = 0.05;
         waypoint_jump_w_normal.x = waypoint_w_normal.x - waypoint_w_normal.normal_x*lift_height_;
         waypoint_jump_w_normal.y = waypoint_w_normal.y - waypoint_w_normal.normal_y*lift_height_;
         waypoint_jump_w_normal.z = waypoint_w_normal.z - waypoint_w_normal.normal_z*lift_height_;
         waypoint_jump_w_normal.normal_x = waypoint_w_normal.normal_x;
         waypoint_jump_w_normal.normal_y = waypoint_w_normal.normal_y;
         waypoint_jump_w_normal.normal_z = waypoint_w_normal.normal_z;
         waypoint_jump_w_normal.curvature = waypoint_w_normal.curvature;


         fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                 waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                 angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                 q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
         toolpath_w_normal->push_back(waypoint_jump_w_normal);


         fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                 waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                 angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                 q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '1'<<'\n';
         toolpath_w_normal->push_back(waypoint_w_normal);



         fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                 waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                 angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                 q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
         toolpath_w_normal->push_back(waypoint_jump_w_normal);


      }//end of for loop
    pcl::io::savePCDFile("data/coupons/tool_path_back_projected_w_lift.pcd", *toolpath_w_normal);
    fout.close(); //close the file

    //float progresspct =100;
    //emit_signal ("toolpath_signal", "percent\n"+std::to_string(progresspct));
    return 0;
  }

//--------------------------------------------------------------------POINT-----------------------------------------------------

>>>>>>> 9845d2bd0d7cd3b3d63106eb00234e3d05d9ac86
=======
>>>>>>> 07aca541621e5cba057452ac8435109ff36ed434
  int ATG_TPG::contour_toolpath(std::string filename)
  {
    // ===== 1. Load variables
    // ===== 2. Load cluster
    // ===== 3. Find cluster bounding box, and also transform and data into bounding box, saved in cloudPointsProjected.txt
    // ===== 4. Toolpath Generation, python script, saves toolpath within boundingbox into sampling_tool_path.txt
    // ===== 5. Loades saved toolpath and transform into actual location
    // ===== 6. Loads coupon surface into kdtreeFlann and check each point in toolpath
    // ===== 6.1 calculate for offset direction for each point
    // ===== 6.2 calculate for individual point's normal from coupon data, and flip for correction
    // ===== 6.3 calculate for individual point's quat and rpy


    // ===== 1. Load variables, done before this function call, this is just for displaying the variables
    // =======================
    std::cout << "external_on_      : " << std::to_string(external_on_      ) << std::endl;
    std::cout << "internal_on_      : " << std::to_string(internal_on_      ) << std::endl;
    std::cout << "resolution_       : " << std::to_string(resolution_       ) << std::endl;
    std::cout << "step_size_        : " << std::to_string(step_size_        ) << std::endl;
    std::cout << "offset_           : " << std::to_string(offset_           ) << std::endl;
    std::cout << "path_rotate_      : " << std::to_string(path_rotate_      ) << std::endl;
    std::cout << "downsample_       : " << std::to_string(downsample_       ) << std::endl;
    std::cout << "ksearch_tp_       : " << std::to_string(ksearch_tp_       ) << std::endl;
    std::cout << "normal_flip_      : " << std::to_string(normal_flip_      ) << std::endl;
    std::cout << "section_range_min_: " << std::to_string(section_range_min_) << std::endl;
    std::cout << "section_range_max_: " << std::to_string(section_range_max_) << std::endl;
    std::cout << "reverse_toolpath_ : " << std::to_string(reverse_toolpath_ ) << std::endl;
    std::cout << "forced_resolution_: " << std::to_string(forced_resolution_) << std::endl;
    std::cout << "hole_patch_size_  : " << std::to_string(hole_patch_size_  ) << std::endl;
    std::cout << "lift_height_      : " << std::to_string(lift_height_      ) << std::endl;

    // ===== 2. Load cluster
    // =======================
    clock_t time1 = clock();//start timer
    pcl::PointCloud<pcl::PointXYZ>::Ptr coupon_data(new pcl::PointCloud<pcl::PointXYZ>);

    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *coupon_data) == -1)
    {
      std::cout << "Failed to read PCD file from: " << filename << std::endl;
      exit(1);
    }
    //int sorted_on = 1;//permanent on now
    //double offset = 0;//superceded
    double depth = 0;

//    //there are 2 methods of offsetting outwards,
//    //enabling this uses the offset calculated from the normal direction on the code after 2d analysis, not stable as the offset may be inwards depending of the toolpath cw or cc dir
//    //disabling this uses the 2d image dilate to offset outwards for calculation, not stable in terms when on curve as the z calculated will only be the nearest point's z instead of following the curve arc
//    if (Offset>0)      //offset used in normal calculation's offset, offset_py used in dilate\erode in py, only sending in -ve into py as +ve offset no surface for proper normal calculation
//    {                  //offset used in normal calculation's offset, offset_py used in dilate\erode in py, only sending in -ve into py as +ve offset no surface for proper normal calculation
//      offset = Offset; //offset used in normal calculation's offset, offset_py used in dilate\erode in py, only sending in -ve into py as +ve offset no surface for proper normal calculation
//      Offset=0;        //offset used in normal calculation's offset, offset_py used in dilate\erode in py, only sending in -ve into py as +ve offset no surface for proper normal calculation
//    }                  //offset used in normal calculation's offset, offset_py used in dilate\erode in py, only sending in -ve into py as +ve offset no surface for proper normal calculation

//    pcl::io::savePCDFile("data/coupons/simplecoupon.pcd", *cloud);
//    system("python src/scripts/edit_pcd.py data/coupons/simplecoupon.pcd");              // save the data after segmentation .pcd and .txt files

    // ===== 3. Find cluster bounding box, and also transform and data into bounding box, saves in cloudPointsProjected.txt
    // =======================
    clock_t time2 = clock();//read timer
    std::cout <<"debug point 2" << std::endl;
    Eigen::Matrix4f transform_matrix;
    transform_matrix = Bounding_Box(coupon_data);//(target_surface);

    // ===== 4. Toolpath Generation, python script, saves toolpath within boundingbox into sampling_tool_path.txt
    // =======================
    std::cout << "Running Toolpath Generation ...\n";
    std::string cmd;
    cmd="rosrun atg_toolpath_generation Tool_Path_Generation_Contours.py " //cmd ="python src/scripts/Tool_Path_Generation_Contours.py "
        + std::to_string(external_on_)      + " " //1
        + std::to_string(internal_on_)      + " " //2
        + std::to_string(1)                 + " " //3
        + std::to_string(resolution_)       + " " //4
        + std::to_string(forced_resolution_)+ " " //5
        + std::to_string(hole_patch_size_)  + " " //6
        + std::to_string(offset_)           + " " //7
        + std::to_string(downsample_)       + " " //8
        + std::to_string(path_rotate_)      + " " //9
        + std::to_string(reverse_toolpath_) + " " //10
        + std::to_string(section_range_min_)+ " " //11
        + std::to_string(section_range_max_);     //12
    system(cmd.c_str());

    clock_t time3 = clock();//read timer
    std::cout <<"Timing c, Tool_Path_Generation_Contour time: " << std::fixed << std::setprecision(4) << float(time3-time2)/float(CLOCKS_PER_SEC) << "s" << std::endl;

    // ===== 5. Loades saved toolpath and transform into actual location
    // =======================
    pcl::PointCloud<pcl::PointXYZ>::Ptr tool_path_projected(new pcl::PointCloud<pcl::PointXYZ>);
    tool_path_projected = ReadTXT("data/coupons/sampling_tool_path.txt");                           //tranformation of the generated toolpath
    pcl::PointCloud<pcl::PointXYZ>::Ptr tool_path_back_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*tool_path_projected, *tool_path_back_projected, transform_matrix);                          //convert the projected toolpath to toolpath in global coordinate frame
    //tool_path = tool_path_projected;   //no transformation needed

    // ===== 6. Loads coupon surface into kdtreeFlann and check each point in toolpath
    // ===== 6.1 calculate for offset direction for each point

    // =======================
    std::string output_filename = "data/coupons/tool_path_back_projected_w_lift_n_trigger.txt";                           //final tool path file
    std::ofstream fout(output_filename);
    std::cout <<"debug point 3" << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud_open; //josh, instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal
    kdtree_cloud_open.setInputCloud (coupon_data);//(target_surface);  //josh, instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal

    //with offset starts here

    pcl::PointCloud<pcl::PointNormal>::Ptr toolpath_w_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointNormal waypoint_w_normal;

    int tp_size = tool_path_back_projected->points.size();
//    int tp_size_5pct = tp_size/16;

    //for calculating
    int wp=0; //for tracking how many contours, e.g. looking out for how many holes on surface
    int sw=0; //for indicating if current to next point is far and needs lifting appends
    for (int i = 0; i < tp_size; i++)
    {
//      //need to emit 20-100% every 5%
//      if (tp_size>16){ //cater to data with less than 16 waypoints
//        if (i % tp_size_5pct==0){
//          int progresspct = 20+i/tp_size_5pct*5;
//          //emit progressUpdated(progresspct);
//          emit_signal ("toolpath_signal", std::to_string(progresspct));
<<<<<<< HEAD
<<<<<<< HEAD
//          std::cout << "\rEMIT progressUpdate"<< progresspct <<" ";
=======
//          std::cout << "\rEMIT progressUpdate"<< progresspct     normal_estimator.setInputCloud(cloud);
>>>>>>> 9845d2bd0d7cd3b3d63106eb00234e3d05d9ac86
=======
//          std::cout << "\rEMIT progressUpdate"<< progresspct <<" ";
>>>>>>> 07aca541621e5cba057452ac8435109ff36ed434
//        }
//      }
      //need to emit 50-100%
      if(1)
      {
        float progresspct = 50+50*i/(tp_size-1);
        emit_signal ("toolpath_signal", "percent\n"+std::to_string(progresspct));
      }

      std::cout << "\rProcessing "<<std::to_string(i+1)<<"/"<<std::to_string(tp_size)<<" on data";//<<output_filename<<" ";
      if(i+1==tp_size) std::cout <<"\rProcessed "<<std::to_string(tp_size)<<" pts on data\n";// "<<output_filename<<"\n";

      // ===== 6.1 calculate for offset direction for each point
      float a;
      float b;
      pcl::Normal offsetnorm;
      if (i<-1+tool_path_back_projected->points.size())
      {
        a=tool_path_back_projected->points[i].y-tool_path_back_projected->points[i+1].y;
        b=tool_path_back_projected->points[i+1].x-tool_path_back_projected->points[i].x;
        if(a==0&&b==0){
          offsetnorm.normal_x = 0;
          offsetnorm.normal_y = 0;
          offsetnorm.normal_z = 1;//-30;
        }
        else{
          offsetnorm.normal_x = a/sqrt(a*a+b*b);
          offsetnorm.normal_y = b/sqrt(a*a+b*b);
          offsetnorm.normal_z = 1;//-30;
        }
      }
      else
      {
        a=tool_path_back_projected->points[i-1].y-tool_path_back_projected->points[i].y;
        b=tool_path_back_projected->points[i].x-tool_path_back_projected->points[i-1].x;
        if(a==0&&b==0){
          offsetnorm.normal_x = 0;
          offsetnorm.normal_y = 0;
          offsetnorm.normal_z = 1;//-30;
        }
        else{
          offsetnorm.normal_x = a/sqrt(a*a+b*b);
          offsetnorm.normal_y = b/sqrt(a*a+b*b);
          offsetnorm.normal_z = 1;//-30;
        }
      }

      // ===== 6.2 calculate for individual point's normal from coupon data, and flip for correction
      //josh, START - instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      Eigen::Vector4f pt_normal;
      if ( kdtree_cloud_open.nearestKSearch (tool_path_back_projected->points[i], ksearch_tp_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {
        float curvature;//1mm offset of envelope from selected points
        pcl::computePointNormal(*coupon_data,pointIdxRadiusSearch,pt_normal,curvature);//(*target_surface,pointIdxRadiusSearch,pt_normal,curvature);
      }
      //josh, END - instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal
      waypoint_w_normal.normal_x = pt_normal[0];//josh replace for target pt normal compute
      waypoint_w_normal.normal_y = pt_normal[1];//josh replace for target pt normal compute
      waypoint_w_normal.normal_z = pt_normal[2];//josh replace for target pt normal compute
      waypoint_w_normal.curvature = pt_normal[3];
      if (waypoint_w_normal.normal_z<0) //if (nor3<0)//josh replace for target pt normal compute
      {
          waypoint_w_normal.normal_x *= -1;//n.normal_x = nor1*(-1);//josh replace for target pt normal compute
          waypoint_w_normal.normal_y *= -1;//n.normal_y = nor2*(-1);//josh replace for target pt normal compute
          waypoint_w_normal.normal_z *= -1;//n.normal_z = nor3*(-1);//josh replace for target pt normal compute
          waypoint_w_normal.curvature *= -1;
      }
      if (normal_flip_)     //user defined normal flip
      {                    //user defined normal flip
          waypoint_w_normal.normal_x *= -1;//user defined normal flip
          waypoint_w_normal.normal_y *= -1;//user defined normal flip
          waypoint_w_normal.normal_z *= -1;//user defined normal flip
          waypoint_w_normal.curvature *= -1;
      }                    //user defined normal flip
      //save waypoint x,y,z plus outwards/inwards offset, disabled offset as it is calculated within py above
      waypoint_w_normal.x = tool_path_back_projected->points[i].x + /*offset*/0*offsetnorm.normal_x;  //p2.x = toolpath->points[i*10].x;//10000*n.normal_x//*simpleCoupon.cluster_tool_path_normals->points[i].normal_x
      waypoint_w_normal.y = tool_path_back_projected->points[i].y + /*offset*/0*offsetnorm.normal_y;  //p2.y = toolpath->points[i*10].y;//10000*n.normal_y
      waypoint_w_normal.z = tool_path_back_projected->points[i].z + depth*offsetnorm.normal_z;  //*n.normal_z

      // ===== 6.3 calculate for individual point's quat and rpy
      //--------------------------code ------------xavier getting the quaternion-----------------------
      Eigen::Vector3d norm(waypoint_w_normal.normal_x, waypoint_w_normal.normal_y, waypoint_w_normal.normal_z);
      Eigen::Quaterniond q;
      q.setFromTwoVectors(Eigen::Vector3d(0,0,1), norm);
      q.normalize();
      //std::cout << "This normal" << norm.transpose() << std::endl;
      //std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec().transpose() << std::endl;
      Eigen::Matrix<double,3,3> rotationMatrix;
      rotationMatrix = q.toRotationMatrix();
      //std::cout << "This normal" << rotationMatrix << std::endl;
      //Eigen::AngleAxisd newAngleAxis(rotationMatrix);
      //Eigen::Vector3d euler = rotationMatrix.eulerAngles(0, 1, 2);

      //manual calculations, still valid
      Eigen::Vector3d angle;
      double roll  = atan2( rotationMatrix(2,1),rotationMatrix(2,2) );
      double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  );
      double yaw   = atan2( rotationMatrix(1,0),rotationMatrix(0,0) );
      Eigen::Vector3d quat;
      quat=q.vec();
      angle[0]=roll*180/(std::atan(1.0)*4);
      if (angle[0]>180){
      //   angle[0]=angle[0]-360;
      }
      angle[1]=pitch*180/(std::atan(1.0)*4);
      if (angle[1]>180){
      //  angle[1]=angle[1]-360;
      }
      angle[2]=yaw*180/(std::atan(1.0)*4);
      if (angle[2]>180){
        //angle[2]=angle[2]-360;
      }
      //std::cout << "Euler from quaternion----------------------- "<< std::endl << roll << ';' << pitch << ';'<< yaw << std::endl;
      //std::cout << " roll, pitch, yaw in degrees++++++++++++++++++"<< std::endl << angle << std::endl;

      //add lift/jump point on first point or first point after big gap (sw) into toolpath
      if (i==0 || sw==1)
      {
        pcl::PointNormal waypoint_jump_w_normal;
        waypoint_jump_w_normal.x = waypoint_w_normal.x - waypoint_w_normal.normal_x*lift_height_;
        waypoint_jump_w_normal.y = waypoint_w_normal.y - waypoint_w_normal.normal_y*lift_height_;
        waypoint_jump_w_normal.z = waypoint_w_normal.z - waypoint_w_normal.normal_z*lift_height_;
        waypoint_jump_w_normal.normal_x = waypoint_w_normal.normal_x;
        waypoint_jump_w_normal.normal_y = waypoint_w_normal.normal_y;
        waypoint_jump_w_normal.normal_z = waypoint_w_normal.normal_z;
        waypoint_jump_w_normal.curvature = waypoint_w_normal.curvature;
        fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
        toolpath_w_normal->push_back(waypoint_jump_w_normal);

        //add current waypoint into toolpath
        fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '1'<<'\n';
        toolpath_w_normal->push_back(waypoint_w_normal);
      }
      else
      {
        //add current waypoint into toolpath
        fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '1'<<'\n';
        toolpath_w_normal->push_back(waypoint_w_normal);
      }
      //xavier added the hold masking exit point, save (not add) first point or first point after big gap for additional exit point to close first pt - last pt gap
      if (i==0 || sw==1)
      {
        std::ofstream f22("data/coupons/firstpoint.txt");
        f22 << waypoint_w_normal.x << '\n' <<        waypoint_w_normal.y << '\n' <<        waypoint_w_normal.z << '\n' <<
               waypoint_w_normal.normal_x << '\n' << waypoint_w_normal.normal_y << '\n' << waypoint_w_normal.normal_z << '\n' << waypoint_w_normal.curvature << "\n" <<
               angle[0] << '\n' << angle[1] << '\n' << angle[2] << '\n' <<
               1    << '\n' << 2    << '\n' << 3    << '\n' << 4    << '\n';
                //q.w()    << '\n' << q.x()    << '\n' << q.y()    << '\n' << q.z()    << '\n';
         f22.close();
         sw=0;
      }

      //add jump when point is too far apart
      float gap_size = resolution_*(downsample_+1);//+1 so that the jump criteria is just slightly higher than normal expected pt2pt dist
      pcl::PointXYZ p1, p2;
      p1.x = tool_path_back_projected->points[i].x;
      p1.y = tool_path_back_projected->points[i].y;
      p1.z = tool_path_back_projected->points[i].z;
      p2.x = tool_path_back_projected->points[i+1].x; //p2.x = toolpath->points[i*2].x;  //p2.x = toolpath->points[i*10].x;
      p2.y = tool_path_back_projected->points[i+1].y; //p2.y = toolpath->points[i*2].y;  //p2.y = toolpath->points[i*10].y;
      p2.z = tool_path_back_projected->points[i+1].z; //p2.z = toolpath->points[i*2].z;  //p2.z = toolpath->points[i*10].z;
      double pt2pt_dist = sqrt(pow(fabs(p1.x-p2.x),2)+pow(fabs(p1.y-p2.y),2)+pow(fabs(p1.z-p2.z),2));
<<<<<<< HEAD
<<<<<<< HEAD
      //std::cout << "pt2pt = " << pt2pt_dist << "  gap_size = " << gap_size << "\n";
=======
      //std::cout << "pt2pt = " << pt2pt_disDetect_Boundaryt << "  gap_size = " << gap_size << "\n";
>>>>>>> 9845d2bd0d7cd3b3d63106eb00234e3d05d9ac86
=======
      //std::cout << "pt2pt = " << pt2pt_dist << "  gap_size = " << gap_size << "\n";
>>>>>>> 07aca541621e5cba057452ac8435109ff36ed434
      if (i<tp_size-1 && (pt2pt_dist>gap_size))//10
      {
        float /*x11, y11, z11, n11, n12, n13,*/ a11, a12, a13, q11, q12, q13, q14, d11;
        wp++;
        sw=1;

        std::ifstream f22("data/coupons/firstpoint.txt");
        f22 >> waypoint_w_normal.x;// x11;
        f22 >> waypoint_w_normal.y;// y11;
        f22 >> waypoint_w_normal.z;// z11;
        f22 >> waypoint_w_normal.normal_x;// n11;
        f22 >> waypoint_w_normal.normal_y;// n12;
        f22 >> waypoint_w_normal.normal_z;// n13;
        f22 >> waypoint_w_normal.curvature;
        f22 >> a11; f22 >> a12; f22 >> a13;
        f22 >> q11; f22 >> q12; f22 >> q13; f22 >> q14;
        f22.close();

        fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                a11 << ';' << a12 << ';' << a13 << ';' <<
                q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';
        toolpath_w_normal->push_back(waypoint_w_normal);

//        fout << x11 << ';' << y11 << ';' << z11 << ';' << n11 << ';' << n12 << ';'<< n13 <<';'<<
//             a11 << ';' << b11 << ';' << c11 << ';' <<
//             q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';

        pcl::PointNormal waypoint_jump_w_normal;
        waypoint_jump_w_normal.x = waypoint_w_normal.x - waypoint_w_normal.normal_x*lift_height_;
        waypoint_jump_w_normal.y = waypoint_w_normal.y - waypoint_w_normal.normal_y*lift_height_;
        waypoint_jump_w_normal.z = waypoint_w_normal.z - waypoint_w_normal.normal_z*lift_height_;
        waypoint_jump_w_normal.normal_x = waypoint_w_normal.normal_x;
        waypoint_jump_w_normal.normal_y = waypoint_w_normal.normal_y;
        waypoint_jump_w_normal.normal_z = waypoint_w_normal.normal_z;

        fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                a11 << ';' << a12 << ';' << a13 << ';' <<
                q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';
        toolpath_w_normal->push_back(waypoint_jump_w_normal);

//        fout << x11 - n11*50 << ';' << y11- n12*50 << ';' << z11 - n13*50 << ';' << n11 << ';' << n12 << ';'<< n13 <<';'<<
<<<<<<< HEAD
<<<<<<< HEAD
//             a11 << ';' << b11 << ';' << c11 << ';' <<
=======
//             a11 << ';' << b11 << ';' << c11Detect_Boundary << ';' <<
>>>>>>> 9845d2bd0d7cd3b3d63106eb00234e3d05d9ac86
=======
//             a11 << ';' << b11 << ';' << c11 << ';' <<
>>>>>>> 07aca541621e5cba057452ac8435109ff36ed434
//             q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<<'\n';

      }

      //xavier close loop using first point and offset up
      if (i> tp_size-2)
      {
        float /*x11, y11, z11, n11, n12, n13,*/ a11, a12, a13, q11, q12, q13, q14;

        std::ifstream f22("data/coupons/firstpoint.txt");
        std::string line;
        f22 >> waypoint_w_normal.x;// x11;
        f22 >> waypoint_w_normal.y;// y11;
        f22 >> waypoint_w_normal.z;// z11;
        f22 >> waypoint_w_normal.normal_x;// n11;
        f22 >> waypoint_w_normal.normal_y;// n12;
        f22 >> waypoint_w_normal.normal_z;// n13;
        f22 >> waypoint_w_normal.curvature;
        f22 >> a11; f22 >> a12; f22 >> a13;
        f22 >> q11; f22 >> q12; f22 >> q13; f22 >> q14;
        f22.close();
        fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                a11 << ';' << a12 << ';' << a13 << ';' <<
                q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';
        toolpath_w_normal->push_back(waypoint_w_normal);
//        fout << x11 << ';' << y11 << ';' << z11 << ';' << n11 << ';' << n12 << ';'<< n13 <<';'<<
//             a11 << ';' << b11 << ';' << c11 << ';' <<
//             q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';

        pcl::PointNormal waypoint_jump_w_normal;
        waypoint_jump_w_normal.x = waypoint_w_normal.x - waypoint_w_normal.normal_x*lift_height_;
        waypoint_jump_w_normal.y = waypoint_w_normal.y - waypoint_w_normal.normal_y*lift_height_;
        waypoint_jump_w_normal.z = waypoint_w_normal.z - waypoint_w_normal.normal_z*lift_height_;
        waypoint_jump_w_normal.normal_x = waypoint_w_normal.normal_x;
        waypoint_jump_w_normal.normal_y = waypoint_w_normal.normal_y;
        waypoint_jump_w_normal.normal_z = waypoint_w_normal.normal_z;
        waypoint_jump_w_normal.curvature = waypoint_w_normal.curvature;
        fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                a11 << ';' << a12 << ';' << a13 << ';' <<
                q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';
        toolpath_w_normal->push_back(waypoint_jump_w_normal);
//        fout << x11 - n11*50 << ';' << y11- n12*50 << ';' << z11 - n13*50 << ';' << n11 << ';' << n12 << ';'<< n13 <<';'<<
//             a11 << ';' << b11 << ';' << c11 << ';' <<
//             q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<<'\n';
      }
      //josh Not closing loop, offset up
      /*if (i> tp_size-2)
      {
        //float x11, y11, z11, n11, n12, n13, a11, b11, c11, q11, q12, q13, q14;

        //x11= waypoint_w_normal.x;
        //y11= waypoint_w_normal.y;
        //z11= waypoint_w_normal.z;
        //n11= waypoint_w_normal.normal_x;
        //n12= waypoint_w_normal.normal_y ;
        //n13= waypoint_w_normal.normal_z;
        //a11= angle[0];
        //b11= angle[1];
        //c11= angle[2];
        //q11= q.w();
        //q12= quat[0];
        //q13= quat[1];
        //q14= quat[2];


        fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
        toolpath_w_normal->push_back(waypoint_w_normal);

//        fout << x11 << ';' << y11 << ';' << z11 << ';' << n11 << ';' << n12 << ';'<< n13 <<';'<<
//             a11 << ';' << b11 << ';' << c11 << ';' <<
//             q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';

        pcl::PointNormal waypoint_jump_w_normal;
        waypoint_jump_w_normal.x = waypoint_w_normal.x - waypoint_w_normal.normal_x*lift_height_;
        waypoint_jump_w_normal.y = waypoint_w_normal.y - waypoint_w_normal.normal_y*lift_height_;
        waypoint_jump_w_normal.z = waypoint_w_normal.z - waypoint_w_normal.normal_z*lift_height_;
        waypoint_jump_w_normal.normal_x = waypoint_w_normal.normal_x;
        waypoint_jump_w_normal.normal_y = waypoint_w_normal.normal_y;
        waypoint_jump_w_normal.normal_z = waypoint_w_normal.normal_z;

        fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
        toolpath_w_normal->push_back(waypoint_jump_w_normal);

//      fout << x11- n11*50 << ';' << y11 - n12*50 << ';' << z11 - n13*50 << ';' << n11 << ';' << n12 << ';'<< n13 <<';'<<
//           a11 << ';' << b11 << ';' << c11 << ';' <<
//           q11 << ';' << q12 << ';' << q13 << ';' << q14 << ';'<<'0'<< '\n';
      }*/

    }//end for loop
    //printf("-----------total hole equal to = %d\n",wp);

   pcl::io::savePCDFile("data/coupons/tool_path_back_projected_w_lift.pcd", *toolpath_w_normal);
   //system("python src/scripts/edit_pcd.py data/coupons/sampling_tool_path_offset.pcd");

    fout.close(); //close the file

    std::cout <<"debug point 4" << std::endl;
    clock_t time7 = clock();//read timer
    std::cout <<"Timing c, compute lift and normal: " << std::fixed << std::setprecision(4) << float(time7-time3)/float(CLOCKS_PER_SEC) << "s" << std::endl;
    std::cout <<"Total time taken for toolpath routine: " << std::fixed << std::setprecision(4) << float(time7-time1)/float(CLOCKS_PER_SEC) << "s" << std::endl;

    //with offset ends here

/*  //without offset starts here
    for (int i = 0; i < tool_path->points.size(); i++)
    {
      int idx;
      f >> idx;                                                  //f is the sampling point id in the data vector
      //just checking idx value
      std::cout <<"i = " << i << "\tidx = " << idx << std::endl;
      Normal n;
      if (normals->points[idx].normal_z > 0)                     //get all tool path point normal as positive
      {
        n.normal_x = normals->points[idx].normal_x;
        n.normal_y = normals->points[idx].normal_y;
        n.normal_z = normals->points[idx].normal_z;
      }
      else
      {
        n.normal_x = -normals->points[idx].normal_x;
        n.normal_y = -normals->points[idx].normal_y;
        n.normal_z = -normals->points[idx].normal_z;
      }
      tool_path_normals->points.push_back(n);                         //save the normals in tool_path_normals, which will be show in the viewer later

      string cmd = "python src/scripts/RotationMatrix.py " + to_string(n.normal_x) + ' ' + to_string(n.normal_y) + ' ' + to_string(n.normal_z);  //convert normal angles to quaternion
      system(cmd.c_str());

      ifstream f1("data/coupons/tmp.txt");
      float alpha, beta, q1, q2, q3, q4; //read the six values from tmp.txt
      f1 >> alpha;
      f1 >> beta;
      f1 >> q1; f1 >> q2; f1 >> q3; f1 >> q4;
      f1.close();   //Save tool path with position, normal, tool orientation and quaternion
      fout << tool_path->points[i].x << ';' << tool_path->points[i].y << ';' << tool_path->points[i].z << ';' <<
        n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << alpha << ';' << beta << ";0;" <<
        q1 << ';' << q2 << ';' << q3 << ';' << q4 << '\n';
    }
    f.close();
    fout.close(); //close the file
    std::cout <<"debug point 5" << std::endl;

*/  //without offset ends here


    return 0;
  }


  int ATG_TPG::zigzag_toolpath(std::string filename)
  {
    // ===== 1. Load variables
    // ===== 2. Load cluster
    // ===== 3. Find cluster bounding box, and also transform and data into bounding box, saved in cloudPointsProjected.txt
    // ===== 4. Toolpath Generation, python script, saves toolpath within boundingbox into sampling_tool_path.txt
    // ===== 5. Loades saved toolpath and transform into actual location
    // ===== 6. Loads coupon surface into kdtreeFlann and check each point in toolpath
    // ===== 6.1 calculate for offset direction for each point
    // ===== 6.2 calculate for individual point's normal from coupon data, and flip for correction
    // ===== 6.3 calculate for individual point's quat and rpy


    // ===== 1. Load variables, done before this function call, this is just for displaying the variables
    // =======================
    std::cout << "external_on_      : " << std::to_string(external_on_      ) << std::endl;
    std::cout << "internal_on_      : " << std::to_string(internal_on_      ) << std::endl;
    std::cout << "resolution_       : " << std::to_string(resolution_       ) << std::endl;
    std::cout << "step_size_        : " << std::to_string(step_size_        ) << std::endl;
    std::cout << "offset_           : " << std::to_string(offset_           ) << std::endl;
    std::cout << "path_rotate_      : " << std::to_string(path_rotate_      ) << std::endl;
    std::cout << "downsample_       : " << std::to_string(downsample_       ) << std::endl;
    std::cout << "ksearch_tp_       : " << std::to_string(ksearch_tp_       ) << std::endl;
    std::cout << "normal_flip_      : " << std::to_string(normal_flip_      ) << std::endl;
    std::cout << "section_range_min_: " << std::to_string(section_range_min_) << std::endl;
    std::cout << "section_range_max_: " << std::to_string(section_range_max_) << std::endl;
    std::cout << "reverse_toolpath_ : " << std::to_string(reverse_toolpath_ ) << std::endl;
    std::cout << "forced_resolution_: " << std::to_string(forced_resolution_) << std::endl;
    std::cout << "hole_patch_size_  : " << std::to_string(hole_patch_size_  ) << std::endl;
    std::cout << "lift_height_      : " << std::to_string(lift_height_      ) << std::endl;

    // ===== 2. Load cluster
    // =======================
    clock_t time1 = clock();//start timer
    pcl::PointCloud<pcl::PointXYZ>::Ptr coupon_data(new pcl::PointCloud<pcl::PointXYZ>);

    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *coupon_data) == -1)
    {
      std::cout << "Failed to read PCD file from: " << filename << std::endl;
      exit(1);
    }

    // ===== 3. Find cluster bounding box, and also transform and data into bounding box, saves in cloudPointsProjected.txt
    // =======================
    clock_t time2 = clock();//read timer
    std::cout <<"debug point 2 " << std::endl;
    Eigen::Matrix4f transform_matrix;
    transform_matrix = Bounding_Box(coupon_data);                                        //bounding box calculation and point cloud data projection

    // ===== 4. Toolpath Generation, python script, saves toolpath within boundingbox into sampling_tool_path.txt
    // =======================
    std::cout << "Running Toolpath Generation ...\n";
    std::string cmd;
    cmd="rosrun atg_toolpath_generation Tool_Path_Generation_Zigzag.py "
        + std::to_string(external_on_)      + " " //1
        + std::to_string(internal_on_)      + " " //2
        + std::to_string(1)                 + " " //3
        + std::to_string(resolution_)       + " " //4
        + std::to_string(forced_resolution_)+ " " //5
        + std::to_string(hole_patch_size_)  + " " //6
        + std::to_string(step_size_)        + " " //7
        + std::to_string(offset_)           + " " //8
        + std::to_string(downsample_)       + " " //9
        + std::to_string(path_rotate_)      + " " //10
        + std::to_string(reverse_toolpath_) + " " //11
        + std::to_string(section_range_min_)+ " " //11
        + std::to_string(section_range_max_);     //12
      system(cmd.c_str());

    clock_t time3 = clock();//read timer
    std::cout <<"Timing c, Tool_Path_Generation_Zigzag time: " << std::fixed << std::setprecision(4) << float(time3-time2)/float(CLOCKS_PER_SEC) << "s" << std::endl;

    try //try catch error in pcl open, sometimes no cloud created, change zoom size
    {
      // ===== 5. Loades saved toolpath and transform into actual location
      // =======================
      pcl::PointCloud<pcl::PointXYZ>::Ptr tool_path_projected(new pcl::PointCloud<pcl::PointXYZ>);
      tool_path_projected = ReadTXT("data/coupons/sampling_tool_path.txt");                           //tranformation of the generated toolpath
      pcl::PointCloud<pcl::PointXYZ>::Ptr tool_path_back_projected(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*tool_path_projected, *tool_path_back_projected, transform_matrix);                          //convert the projected toolpath to toolpath in global coordinate frame
      //tool_path = tool_path_projected;   //no transformation needed

      // ===== 6. Loads coupon surface into kdtreeFlann and check each point in toolpath
      // ===== 6.1 calculate for offset direction for each point

      // =======================
      std::string output_filename = "data/coupons/tool_path_back_projected_w_lift_n_trigger.txt";                           //final tool path file
      std::ofstream fout(output_filename);
      std::cout <<"debug point 3" << std::endl;

      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud_open; //josh, instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal
      kdtree_cloud_open.setInputCloud (coupon_data);  //josh, instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal

      pcl::PointCloud<pcl::PointNormal>::Ptr toolpath_w_normal(new pcl::PointCloud<pcl::PointNormal>);
      pcl::PointNormal waypoint_w_normal;

      int tp_size = tool_path_back_projected->points.size();
//      int tp_size_5pct = tp_size/16;

        for (int i = 0; i < tp_size; i++)
        {
//          //need to emit 20-100% every 5%
//          if (tp_size>16){ //cater to data with less than 16 waypoints
//            if (i % tp_size_5pct==0){
//              int progresspct = 20+i/tp_size_5pct*5;
//              //emit progressUpdated(progresspct);
//              std::cout << "\rEMIT progressUpdate"<< progresspct <<"";
//            }
//          }
          //need to emit 50-100%
          if(1)
          {
            float progresspct = 50+50*i/(tp_size-1);
            emit_signal ("toolpath_signal", "percent\n"+std::to_string(progresspct));
          }
          std::cout << "\rProcessing "<<std::to_string(i+1)<<"/"<<std::to_string(tp_size)<<" on data";
          if(i+1==tp_size) std::cout <<"\rProcessed "<<std::to_string(tp_size)<<" pts on data\n";


          // ===== 6.2 calculate for individual point's normal from coupon data, and flip for correction

          //josh, START - instead of using normalestimationOMPL for whole target_surface, compute targeted point's normal
          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          Eigen::Vector4f pt_normal;
          if ( kdtree_cloud_open.nearestKSearch (tool_path_back_projected->points[i], ksearch_tp_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
          {
            float curvature;//1mm offset of envelope from selected points
            pcl::computePointNormal(*coupon_data,pointIdxRadiusSearch,pt_normal,curvature);
          }

          waypoint_w_normal.normal_x = pt_normal[0];//n.normal_x = normals->points[idx].normal_x;//josh replace for target pt normal compute
          waypoint_w_normal.normal_y = pt_normal[1];//n.normal_y = normals->points[idx].normal_y;//josh replace for target pt normal compute
          waypoint_w_normal.normal_z = pt_normal[2];//n.normal_z = normals->points[idx].normal_z;//josh replace for target pt normal compute
          if (waypoint_w_normal.normal_z<0)                                  //josh added temporary for nor3>0 condition
          {                                                  //josh added temporary for nor3>0 condition
              waypoint_w_normal.normal_x *= -1;//n.normal_x = normals->points[idx].normal_x*-1; //josh added temporary for nor3>0 condition
              waypoint_w_normal.normal_y *= -1;//n.normal_y = normals->points[idx].normal_y*-1; //josh added temporary for nor3>0 condition
              waypoint_w_normal.normal_z *= -1;//n.normal_z = normals->points[idx].normal_z*-1; //josh added temporary for nor3>0 condition
          }                                                  //josh added temporary for nor3>0 condition
          if (normal_flip_)     //user defined normal flip
          {                    //user defined normal flip
              waypoint_w_normal.normal_x *= -1;//user defined normal flip
              waypoint_w_normal.normal_y *= -1;//user defined normal flip
              waypoint_w_normal.normal_z *= -1;//user defined normal flip
          }                    //user defined normal flip
          waypoint_w_normal.x = tool_path_back_projected->points[i].x;
          waypoint_w_normal.y = tool_path_back_projected->points[i].y;
          waypoint_w_normal.z = tool_path_back_projected->points[i].z;


          //--------------------------code ------------xavier getting the quaternion-----------------------
          Eigen::Vector3d norm(waypoint_w_normal.normal_x, waypoint_w_normal.normal_y, waypoint_w_normal.normal_z);
          Eigen::Quaterniond q;
          q.setFromTwoVectors(Eigen::Vector3d(0,0,1), norm);
          q.normalize();
          //std::cout << "This normal" << norm.transpose() << std::endl;
          //std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec().transpose() << std::endl;
          Eigen::Matrix<double,3,3> rotationMatrix;
          rotationMatrix = q.toRotationMatrix();
          //std::cout << "This normal" << rotationMatrix << std::endl;
          //Eigen::AngleAxisd newAngleAxis(rotationMatrix);
          //Eigen::Vector3d euler = rotationMatrix.eulerAngles(0, 1, 2);

          //manual calculations, still valid
          Eigen::Vector3d angle;
          double roll  = atan2( rotationMatrix(2,1),rotationMatrix(2,2) );
          double pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  );
          double yaw   = atan2( rotationMatrix(1,0),rotationMatrix(0,0) );
          Eigen::Vector3d quat;
          quat=q.vec();
          angle[0]=roll*180/(std::atan(1.0)*4);
          if (angle[0]>180){
          //   angle[0]=angle[0]-360;
          }
          angle[1]=pitch*180/(std::atan(1.0)*4);
          if (angle[1]>180){
          //  angle[1]=angle[1]-360;
          }
          angle[2]=yaw*180/(std::atan(1.0)*4);
          if (angle[2]>180){
            //angle[2]=angle[2]-360;
          }
          //std::cout << "Euler from quaternion----------------------- "<< std::endl << roll << ';' << pitch << ';'<< yaw << std::endl;
          //std::cout << " roll, pitch, yaw in degrees++++++++++++++++++"<< std::endl << angle << std::endl;

          pcl::PointNormal waypoint_jump_w_normal;
          waypoint_jump_w_normal.x = waypoint_w_normal.x - waypoint_w_normal.normal_x*lift_height_;
          waypoint_jump_w_normal.y = waypoint_w_normal.y - waypoint_w_normal.normal_y*lift_height_;
          waypoint_jump_w_normal.z = waypoint_w_normal.z - waypoint_w_normal.normal_z*lift_height_;
          waypoint_jump_w_normal.normal_x = waypoint_w_normal.normal_x;
          waypoint_jump_w_normal.normal_y = waypoint_w_normal.normal_y;
          waypoint_jump_w_normal.normal_z = waypoint_w_normal.normal_z;
          waypoint_jump_w_normal.curvature = waypoint_w_normal.curvature;
          if (i==0)
          {
            fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                    waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                    angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                    q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
            toolpath_w_normal->push_back(waypoint_jump_w_normal);
//          fout << final_zigzag_tool_path->points[i].x - 60*n.normal_x << ';' << final_zigzag_tool_path->points[i].y - 60*n.normal_y << ';' << final_zigzag_tool_path->points[i].z - 60*n.normal_z  << ';' <<
//                     n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                     q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] <<';' << '0'<<'\n';

            //add current waypoint into toolpath
            fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                    waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                    angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                    q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '1'<<'\n';
            toolpath_w_normal->push_back(waypoint_w_normal);
//            fout << final_zigzag_tool_path->points[i].x << ';' << final_zigzag_tool_path->points[i].y << ';' << final_zigzag_tool_path->points[i].z  << ';' <<
//                       n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                       q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] <<';' << '1'<<'\n';
          }

          if(i<tp_size-1)
          {
            float gap_size = step_size_*5;//+1 so that the jump criteria is just slightly higher than normal expected pt2pt dist
            pcl::PointNormal p1, p2;
            p1.x = tool_path_back_projected->points[i].x;
            p1.y = tool_path_back_projected->points[i].y;
            p1.z = tool_path_back_projected->points[i].z;
            p2.x = tool_path_back_projected->points[i+1].x; //p2.x = toolpath->points[i*2].x;  //p2.x = toolpath->points[i*10].x;
            p2.y = tool_path_back_projected->points[i+1].y; //p2.y = toolpath->points[i*2].y;  //p2.y = toolpath->points[i*10].y;
            p2.z = tool_path_back_projected->points[i+1].z; //p2.z = toolpath->points[i*2].z;  //p2.z = toolpath->points[i*10].z;
            double pt2pt_dist = sqrt(pow(fabs(p1.x-p2.x),2)+pow(fabs(p1.y-p2.y),2)+pow(fabs(p1.z-p2.z),2));

            //if (sqrt(pow((final_zigzag_tool_path->points[i].x-final_zigzag_tool_path->points[i+1].x),2.0)+pow((final_zigzag_tool_path->points[i].y-final_zigzag_tool_path->points[i+1].y),2.0)+pow((final_zigzag_tool_path->points[i].z-final_zigzag_tool_path->points[i+1].z),2.0))>Step_Size*5)//10)//without cbrt > 25
            if(pt2pt_dist>gap_size)
            {
              fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                      waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                      angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                      q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
              toolpath_w_normal->push_back(waypoint_w_normal);
//            fout << final_zigzag_tool_path->points[i].x << ';' << final_zigzag_tool_path->points[i].y << ';' << final_zigzag_tool_path->points[i].z  << ';' <<
//                       n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                       q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] <<';' << '0'<<'\n';

              fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                      waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                      angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                      q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
              toolpath_w_normal->push_back(waypoint_jump_w_normal);
//            fout << final_zigzag_tool_path->points[i].x-20*n.normal_x << ';' << final_zigzag_tool_path->points[i].y-20*n.normal_y << ';' << final_zigzag_tool_path->points[i].z-20*n.normal_z << ';' <<
//                     n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                     q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] << ';' << '0' <<'\n';
              //using p2 to store next point
              p2.normal_x = waypoint_w_normal.normal_x; p2.x -= lift_height_*p2.normal_x;
              p2.normal_y = waypoint_w_normal.normal_y; p2.y -= lift_height_*p2.normal_y;
              p2.normal_z = waypoint_w_normal.normal_z; p2.z -= lift_height_*p2.normal_z;
              fout << p2.x  << ';' << p2.y << ';' << p2.z << ';' <<
                      p2.normal_x << ';' << p2.normal_y << ';' << p2.normal_z << ';' <<
                      angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                      q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
              toolpath_w_normal->push_back(p2);
//            fout << final_zigzag_tool_path->points[i+1].x-20*n.normal_x << ';' << final_zigzag_tool_path->points[i+1].y-20*n.normal_y << ';' << final_zigzag_tool_path->points[i+1].z-20*n.normal_z << ';' <<
//                     n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                     q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] << ';' << '0' <<'\n';
            }
            else
            {
              fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                      waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                      angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                      q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '1'<<'\n';
              toolpath_w_normal->push_back(waypoint_w_normal);
//              fout << final_zigzag_tool_path->points[i].x << ';' << final_zigzag_tool_path->points[i].y << ';' << final_zigzag_tool_path->points[i].z  << ';' <<
//                         n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                         q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] <<';' << '1'<<'\n';
            }
          }

          if (i> tp_size-2)
          {
            fout << waypoint_w_normal.x << ';' <<        waypoint_w_normal.y << ';' <<        waypoint_w_normal.z << ';' <<
                    waypoint_w_normal.normal_x << ';' << waypoint_w_normal.normal_y << ';' << waypoint_w_normal.normal_z << ';' <<
                    angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
                    q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
            toolpath_w_normal->push_back(waypoint_w_normal);
//            fout << final_zigzag_tool_path->points[i].x << ';' << final_zigzag_tool_path->points[i].y << ';' << final_zigzag_tool_path->points[i].z << ';' <<
//                       n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                       q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] << ';' << '0' <<'\n';

            fout << waypoint_jump_w_normal.x << ';' <<        waypoint_jump_w_normal.y << ';' <<        waypoint_jump_w_normal.z << ';' <<
                    waypoint_jump_w_normal.normal_x << ';' << waypoint_jump_w_normal.normal_y << ';' << waypoint_jump_w_normal.normal_z << ';' <<
                    angle[0] << ';' << angle[1] << ';' << angle[2] << ';' <<
                    q.w()    << ';' << q.x()    << ';' << q.y()    << ';' << q.z()    << ';' << '0'<<'\n';
            toolpath_w_normal->push_back(waypoint_jump_w_normal);
//            fout << final_zigzag_tool_path->points[i].x-60*n.normal_x << ';' << final_zigzag_tool_path->points[i].y-60*n.normal_y << ';' << final_zigzag_tool_path->points[i].z-60*n.normal_z << ';' <<
//                       n.normal_x << ';' << n.normal_y << ';' << n.normal_z << ';' << angle[0] << ';' << angle[1] << ';'<< angle[2] <<';'<<
//                       q.w() << ';' << quat[0] << ';' << quat[1] << ';' << quat[2] << ';' << '0' <<'\n';

          }
//------------------------------------------------------------------------------------------code--------------------------

        }

        pcl::io::savePCDFile("data/coupons/tool_path_back_projected_w_lift.pcd", *toolpath_w_normal);
        //replaced with pt search //f.close();
        fout.close(); //close the file

        std::cout <<"debug point 4" << std::endl;
        clock_t time7 = clock();//read timer
        std::cout <<"Timing c, compute normal: " << std::fixed << std::setprecision(4) << float(time7-time3)/float(CLOCKS_PER_SEC) << "s" << std::endl;
        std::cout <<"Total time taken for toolpath routine: " << std::fixed << std::setprecision(4) << float(time7-time1)/float(CLOCKS_PER_SEC) << "s" << std::endl;

//        cluster_target_surface = target_surface;
//        cluster_tool_path = final_zigzag_tool_path;
//        cluster_tool_path_normals =tool_path_normals;
        //with offset ends here

        return 0;
    }//end try
    catch(...)//default catch handler
    {
      std::string msg = "Zig Zag Failed, Please change zoom size or modify surface points.";
      emit_signal ("toolpath_signal", "error msg\n"+msg);
      std::cout << msg << std::endl;
      //QMessageBox::information(NULL, "Oops", "Zig Zag Failed, Please change zoom size or modify surface points.");
    }

  }

  void ATG_TPG::emit_signal (std::string signal_name, std::string msg)
  {
    std::ofstream fsignal;
    fsignal.open("data/coupons/"+signal_name+".txt",std::ofstream::out);
    fsignal << msg ;
    fsignal.close();
  }
}
