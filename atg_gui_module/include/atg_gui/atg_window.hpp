#ifndef ATG_WINDOW_HPP
#define ATG_WINDOW_HPP

#include <QMainWindow>
#include "ui_atg_window.h"
#include "qnode.hpp"

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

namespace atg_gui
{
  class ATG_Window : public QMainWindow
  {
    Q_OBJECT

  public:
    /*explicit*/ ATG_Window(int argc, char** argv, QWidget *parent = 0);
    ~ATG_Window();

    /*********************
    ** Variable saving through QSettings
    **********************/
    void readQSettings(); //josh added, read settings at program start
    void writeQSettings();//josh added, save settings whenever value change
    void readQSettings_TCP_only(); //reads only TCP and eeTCP
    void readQSettings_ENV_OBJ_only();
    void readQSettings_Robot_IP_only();
    void closeEvent(QCloseEvent *event); // Overloaded function

    /******************************************
    ** point cloud functions
    *******************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr  load_point_cloud_data(std::string file_name, bool make_mini=0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_manipulate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float x, float y, float z, float rx, float ry, float rz, float scale);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_union(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_ori, int mode=0);
    void recalculate_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, int cluster_no_r, int k_Search, pcl::PointXYZ centroid);

    /*********************
    ** Logging
    **********************/
    enum LogLevel {
      Debug,
      Info,
      Warn,
      Error,
      Fatal
    };
    QStringListModel* loggingModel() { return &logging_model; }
    void log(const LogLevel &level, const std::string &msg);
    const std::string currentDateTime();
    void updateLoggingView();

  public Q_SLOTS:

    //slots for tab control << and >> buttons
    void pushButton_processNextPrev_clicked();
    void pushButton_quit_clicked();

    //slots for file tab
    void pushButton_close_clicked();
    void pushButton_open_clicked(QString assigned_inputfilename="");

    //slots for segmentation tab
    void pushButton_segmentation_clicked();          //slot for seg button
    void seg_thread();                               //part 2 of seg, not a slot actually..
    std::vector<std::string> read_write_fsignal(std::string signal, char action,std::string msg=""); //not a slot actually..
    void read_pcd_in_cluster_folder();                              //not a slot actually..
    void remove_seg_labels();                                       //not a slot actually..
    void cluster_no_ctrl(int value,char action='v');  //slot for cluster spinboxes and function for enable/disable actions
    void pushButton_visualize_clusters_clicked();
    void pushButton_clear_clusters_clicked();
    void pushButton_load_backup_clusters_clicked ();
    void pushButton_backup_curr_clusters_clicked ();

    void pushButton_draw_create_clicked() ;
    void pushButton_draw_subtract_clicked() ;
    void pushButton_draw_append_clicked() ;
    void clusters_no_min_max(int min,int max);                                          //josh added for interactive click
    bool draw_cluster(int mode, int draw_cluster_no);

    //slots for robot settings tab
    void comboBox_tool_selection_clicked(QString value);
    void comboBox_env_selection_clicked(QString value);
    void comboBox_robot_selection_clicked(QString value);
    void pushButton_read_robot_joints_clicked();
    void read_joints_thread();

    void robot_TCP_changed(double value);
    void flip_surface_normal();
    void pushButton_viz_svd_clicked     ();
    void pushButton_cal_svd_clicked     ();
    void pushButton_close_svd_clicked   ();
    void pushButton_transfer_svd_clicked();

    void radioButton_nkns_spindle_clicked();
    void doubleSpinBox_nsks_spindle_zone_radius_changed(double value);

    void doubleSpinBox_masking_blend_radius_changed(double value);

    void doubleSpinBox_impedance_blend_radius_changed(double);

    void doubleSpinBox_sandblasting_zone_radius_changed(double);

    void double_KSearch_diameter_changed(double);
    void double_KSearch_contact_threshold_changed(double);

    //slots for toolpath tab
    void comboBox_toolpath_selection_textChanged(QString Text_comboBox);
    void pushButton_plot_toolpath_clicked();
    void toolpath_thread();                               //part 2 of toolpath, not a slot actually..
    void pushButton_draw_simple_toolpath_clicked ();
    void pushButton_clear_simple_toolpath_clicked();
    void pushButton_queue_clicked       ();
    void pushButton_reset_queue_clicked ();

    //slots for robot tab
    void pushButton_robotSetup_clicked();
    void pushButton_robotSim_clicked();
    void pushButton_robotRun_clicked();
    void pushButton_runScript_clicked();
    void RunScriptFunction(std::string scriptname);
    void pushButton_saveScriptAs_clicked();
    void pushButton_refreshScript_clicked();


    //slots for mouse events on visualizer
    void mouseEventProcess (const pcl::visualization::MouseEvent& event);               //josh added for interactive click
    void areaPickingEventProcess (const pcl::visualization::AreaPickingEvent& event);   //josh added for interactive click
    void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event); //josh added for interactive click
    void keyboardEventProcess(const pcl::visualization::KeyboardEvent& event);          //josh added for interactive click
    void comboBox_interaction_mode_TextChanged(QString text_comboBox);
    void interactive_tab_signal(int tab_no);
    void pushButton_IM_cluster_select_clicked();
    void line_draw_mode(std::string event_name, pcl::PointXYZ point = pcl::PointXYZ(0,0,0));

  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ui_cloud_;

  private:
    Ui::ATG_Window ui;
    atg_gui::QNode qnode;
    QStringListModel logging_model;

    QString ex_data_file_dir_ = "/home/data/";                           //external data folder
    QString config_file_dir_  = "/home/data/cache/atg_configuration.ini";//external data/cache/config file

    QString in_data_file_dir_ = "data/coupons/";//internal data folder, this resolves to  <package dir>/data/coupons/

    QString g_open_filename_ = "";
    QString g_short_filename_ = "";
    pcl::KdTreeFLANN<pcl::PointXYZ> g_kdtree_cloud_open_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_opened_;
    pcl::PointXYZ g_cloud_opened_centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_cluster_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_toolpath_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_hover_mouse_point_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_hover_mouse_area_ ;
    std::string g_keyboardkey_ = "";

    std::vector<pcl::PointXYZ> cluster_centroids_; //store cluster's centroid pose for 3dText/label position;
    QTimer *g_seg_timer_;  //for segmentation timer
    QTimer *g_tpg_timer_;  //for toolpath gen timer
    QTimer *g_rjv_timer_;  //for read joint values timer
    //std::atomic seg_thread_connected_ = false;
  };
}
#endif // ATG_WINDOW_H
