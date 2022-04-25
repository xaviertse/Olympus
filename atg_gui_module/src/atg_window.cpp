#include "../include/atg_gui/atg_window.hpp"
#include "../include/atg_gui/converter.hpp"
#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <string>
#include <ros/package.h> //for finding named package directories

//Point Cloud Library
//#include <pcl/point_types.h>
//#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

#include <boost/filesystem.hpp>

//Visualizer Operations
#include <vtkPointPicker.h> //josh for click interaction
#include <vtkAreaPicker.h>  //josh for click interaction
using namespace pcl;
using namespace Qt;


namespace atg_gui{

ATG_Window::ATG_Window(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    qnode(argc,argv)//ui(new Ui::ATG_Window)
{
  /*********************
  ** UI Launch Setup
  **********************/
  ui.setupUi(this);
  this->setWindowTitle ("Automated Toolpath Generation - ARTC, A*STAR");
  QLocale::setDefault(QLocale::c());
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QTimer::singleShot(0, this, SLOT(showMaximized())); //maximize window


  /*********************
  ** Logging
  **********************/
  ui.view_logging->setModel(loggingModel());
  //GUI intuitive interaction help
  ui.textEdit_help_text->append("x, X : toggle rubber band selection mode for left mouse button");
  ui.textEdit_help_text->append("p, P : switch to a point-based representation");
  ui.textEdit_help_text->append("w, W : switch to a wireframe-based representation (where available)");
  ui.textEdit_help_text->append("s, S : switch to a surface-based representation");
  ui.textEdit_help_text->append("f, F : fly to point mode");
  ui.textEdit_help_text->append("+ / - : increment/decrement overall point size");
  ui.textEdit_help_text->append("g, G : display scale grid (on/off)");
  ui.textEdit_help_text->append("r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]");
  ui.textEdit_help_text->append("ALT + s, S : turn stereo mode on/off");
  ui.textEdit_help_text->append("SHIFT + left click : select a point");
  ui.textEdit_help_text->setReadOnly(true);
  ui.textEdit_help_text->setLineWrapMode(QTextEdit::NoWrap);


  /*********************
  ** Setup Vizualizer
  **********************/
  //setup a dummy cloud for viz and fill with random points
//  ui_cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
//  ui_cloud_->points.resize(100000);
//  for (size_t i = 0; i < ui_cloud_->points.size (); ++i)
//  {
//    ui_cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//    ui_cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//    ui_cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//    ui_cloud_->points[i].r = 128;//white r=g=b=128
//    ui_cloud_->points[i].g = 128;//white r=g=b=128
//    ui_cloud_->points[i].b = 128;//white r=g=b=128
//  }

  //init g_g_cloud_opened to allow reading and storing of obj pcd
  g_cloud_opened_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>); //added for interactive GUI, must initialize ptr before using to avoid crash reading empty ptr
  g_cloud_cluster_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  g_cloud_toolpath_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  g_hover_mouse_point_= PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  g_hover_mouse_area_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  //viewer->setBackgroundColor(viewer_r,viewer_g,viewer_b);
  ui.qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->getRenderWindow()->GlobalWarningDisplayOff(); //remove warning spam of VTK
  viewer->setupInteractor (ui.qvtkWidget->GetInteractor (), ui.qvtkWidget->GetRenderWindow ());
  viewer->addCoordinateSystem(10.0,"world");
//  viewer->addPointCloud(ui_cloud_,"ui_cloud");
  ui.qvtkWidget->update ();

  //Adding interactive selection by clicking on data surfaces
  viewer->registerMouseCallback (boost::bind(&ATG_Window::mouseEventProcess,this,_1));
  viewer->registerAreaPickingCallback(boost::bind(&ATG_Window::areaPickingEventProcess,this,_1));
  viewer->registerPointPickingCallback(boost::bind(&ATG_Window::pointPickingEventOccurred,this,_1));
  viewer->registerKeyboardCallback(boost::bind(&ATG_Window::keyboardEventProcess,this,_1));

  //reset progress bar
  ui.progressBar->setValue(0);
  ui.progressBar->update();


  /*********************
  ** Hide/Disable invalid UI components
  **********************/
  if(1)
  {
    ui.tabWidget_dummy->setVisible(false);             //dummy tab
    ui.tabWidget_file->removeTab(1);                   //File Ops->Advanced (MLS)
    ui.pushButton_align_object->setVisible(false);     //robot->obj_frame->align button (not in use)

    //create finger tabs, tabwidget bound west with horizontal label names START
    QTabWidget *tabw = ui.tabWidget_Flow;
    tabw->setTabText(0, "");
    tabw->setTabText(1, "");
    tabw->setTabText(2, "");
    tabw->setTabText(3, "");
    tabw->setTabText(4, "");
    tabw->setTabText(5, "");
    QTabBar *tabbar = tabw->tabBar();

    QFont lblfont("Arial",10);
    QLabel *lbl0 = new QLabel();
    lbl0->setObjectName("lblFileOption");  //josh added for sharefolder auto detect
    lbl0->setText("\nFile\nOptions\n");
    lbl0->setFont(lblfont);
    tabbar->setTabButton(0, QTabBar::LeftSide, lbl0);
    QLabel *lbl1 = new QLabel();
    lbl1->setObjectName("lblFrame");
    lbl1->setText("\nFrame\nCLBR.\n");
    lbl1->setFont(lblfont);
    tabbar->setTabButton(1, QTabBar::LeftSide, lbl1);
    QLabel *lbl2 = new QLabel();
    lbl2->setObjectName("lblSurface");
    lbl2->setText("\nSurfaces\n");
    lbl2->setFont(lblfont);
    tabbar->setTabButton(2, QTabBar::LeftSide, lbl2);
    QLabel *lbl3 = new QLabel();
    lbl3->setObjectName("lblWorkCell");
    lbl3->setText("\nWorkcell\nSettings\n");
    lbl3->setFont(lblfont);
    tabbar->setTabButton(3, QTabBar::LeftSide, lbl3);
    QLabel *lbl4 = new QLabel();
    lbl4->setObjectName("lblToolpath");
    lbl4->setText("\nToolpath\nPlan\n");
    lbl4->setFont(lblfont);
    tabbar->setTabButton(4, QTabBar::LeftSide, lbl4);
    QLabel *lbl5 = new QLabel();
    lbl5->setObjectName("lblRobotCode");
    lbl5->setText("\nRobot\nCode\n");
    lbl5->setFont(lblfont);
    tabbar->setTabButton(5, QTabBar::LeftSide, lbl5);
    QLabel *lbl6 = new QLabel();
    lbl5->setObjectName("lblOperator");
    lbl5->setText("\nOperator\nMode\n");
    lbl5->setFont(lblfont);
    tabbar->setTabButton(6, QTabBar::LeftSide, lbl6);
    QLabel *lbl7 = new QLabel();
    lbl5->setObjectName("lblEdge_Line_PP");
    lbl5->setText("\nEdge\nDetection\n");
    lbl5->setFont(lblfont);
    tabbar->setTabButton(7, QTabBar::LeftSide, lbl7);
    //create finger tabs, tabwidget bound west with horizontal label names END
    ui.tabWidget_Flow->removeTab(5);  //for <<< and >>> button operation lblOperator
    ui.tabWidget_Flow->removeTab(4);  //for <<< and >>> button operation
    ui.tabWidget_Flow->removeTab(3);  //for <<< and >>> button operation
    ui.tabWidget_Flow->removeTab(2);  //for <<< and >>> button operation
    ui.tabWidget_Flow->removeTab(1);  //for <<< and >>> button operation
    ui.progressBar->setValue(20);
    ui.progressBar->update();
  }

  /*********************
  ** SLOTS
  **********************/
  if(1)
  {
    //=======================================
    //slots for tab control << and >> buttons
    connect (ui.pushButton_process_next, SIGNAL (clicked()), this, SLOT (pushButton_processNextPrev_clicked ()));
    connect (ui.pushButton_process_previous, SIGNAL (clicked()), this, SLOT (pushButton_processNextPrev_clicked ()));
    ui.spinBox_process_nextpre->setStyleSheet("QSpinBox{background-color: QColor(0, 0, 0, 0);}"); //qcolor alpha set to 0
    connect (ui.pushButton_quit, SIGNAL (clicked()), this, SLOT (pushButton_quit_clicked()));


    //=======================================
    //slots for file tab
    connect (ui.pushButton_close, SIGNAL (clicked()), this, SLOT (pushButton_close_clicked()));
    connect (ui.pushButton_open,  SIGNAL (clicked()), this, SLOT (pushButton_open_clicked ()));


    //=======================================
    //slots for segmentation tab + seg thread functions
    connect (ui.pushButton_segmentation,  SIGNAL (clicked()), this, SLOT (pushButton_segmentation_clicked ()));
    connect (ui.spinBox_clusters       ,  SIGNAL (valueChanged(int)), this, SLOT (cluster_no_ctrl (int)));
    connect (ui.spinBox_clusters_draw  ,  SIGNAL (valueChanged(int)), this, SLOT (cluster_no_ctrl (int)));
    connect (ui.spinBox_toolpaths      ,  SIGNAL (valueChanged(int)), this, SLOT (cluster_no_ctrl (int)));
    connect (ui.pushButton_visualize_clusters  , SIGNAL (clicked()), this, SLOT (pushButton_visualize_clusters_clicked ()));
    connect (ui.pushButton_visualize_clusters_2, SIGNAL (clicked()), this, SLOT (pushButton_visualize_clusters_clicked ()));
    connect (ui.pushButton_clear_clusters      , SIGNAL (clicked()), this, SLOT (pushButton_clear_clusters_clicked     ()));
    connect (ui.pushButton_clear_clusters_2    , SIGNAL (clicked()), this, SLOT (pushButton_clear_clusters_clicked     ()));
    connect (ui.pushButton_load_backup_clusters, SIGNAL (clicked()), this, SLOT (pushButton_load_backup_clusters_clicked ()));
    connect (ui.pushButton_backup_curr_clusters, SIGNAL (clicked()), this, SLOT (pushButton_backup_curr_clusters_clicked ()));
    //pointcloud viz functions
    connect (ui.pushButton_IM_cluster_select,SIGNAL (clicked()), this, SLOT (pushButton_IM_cluster_select_clicked()));
    connect (ui.tabWidget_Flow, SIGNAL (currentChanged(int)), this, SLOT (interactive_tab_signal(int)));
    connect (ui.tabWidget_seg , SIGNAL (currentChanged(int)), this, SLOT (interactive_tab_signal(int)));
    connect (ui.comboBox_interaction_mode, SIGNAL (currentTextChanged(QString)), this, SLOT (comboBox_interaction_mode_TextChanged(QString)));
    ui.comboBox_interaction_mode->setCurrentIndex(1);ui.comboBox_interaction_mode->setCurrentIndex(0);//reset mode
    ui.tabWidget_seg->setCurrentIndex(0);                                                             //reset seg tab
    connect (ui.pushButton_draw_append,   SIGNAL (clicked()), this, SLOT (pushButton_draw_append_clicked()));    //josh, for interactive UI
    connect (ui.pushButton_draw_subtract, SIGNAL (clicked()), this, SLOT (pushButton_draw_subtract_clicked()));  //josh, for interactive UI
    connect (ui.pushButton_draw_create,   SIGNAL (clicked()), this, SLOT (pushButton_draw_create_clicked ()));   //josh, for interactive UI

    //=======================================
    //slots for robot settings tab
    connect (ui.comboBox_tool_selection,      SIGNAL (currentTextChanged(QString)), this, SLOT (comboBox_tool_selection_clicked  (QString)));
    connect (ui.comboBox_env_selection,       SIGNAL (currentTextChanged(QString)), this, SLOT (comboBox_env_selection_clicked   (QString)));
    connect (ui.comboBox_robot_selection,     SIGNAL (currentTextChanged(QString)), this, SLOT (comboBox_robot_selection_clicked (QString)));
    //Robot&ENV tab- > syn TCP info with tool selection (supersedes mask and polish xyz,rxryrz)
    connect (ui.doubleSpinBox_robot_tcp_X      , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_tcp_Y      , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_tcp_Z      , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_tcp_R_     , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_tcp_P_     , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_tcp_Y_     , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_eeX        , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_eeY        , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_eeZ        , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_eeR_       , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_eeP_       , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_robot_eeY_       , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_nkns_spindle_tool_dia  , SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    connect (ui.doubleSpinBox_nkns_spindle_tilt_angle, SIGNAL (valueChanged (double)), this, SLOT (robot_TCP_changed(double)));
    //WorkObj tab
    // KIV, not needed ATM //connect (ui.pushButton_align_object, SIGNAL (clicked()), this, SLOT (align_pcd_to_object_frame())); //josh added
    connect (ui.pushButton_flip_surface_normal, SIGNAL (clicked()), this, SLOT (flip_surface_normal ()));
    connect (ui.pushButton_viz_svd,      SIGNAL (clicked()), this, SLOT (pushButton_viz_svd_clicked     ()));
    connect (ui.pushButton_cal_svd,      SIGNAL (clicked()), this, SLOT (pushButton_cal_svd_clicked     ()));
    connect (ui.pushButton_close_svd,    SIGNAL (clicked()), this, SLOT (pushButton_close_svd_clicked   ()));
    connect (ui.pushButton_transfer_svd, SIGNAL (clicked()), this, SLOT (pushButton_transfer_svd_clicked()));
    //polishing/nakanishi spindle tool
    connect (ui.radioButton_nkns_spindle_flat_disc_perp, SIGNAL (clicked()), this, SLOT (radioButton_nkns_spindle_clicked ()));
    connect (ui.radioButton_nkns_spindle_flat_disc_tilt, SIGNAL (clicked()), this, SLOT (radioButton_nkns_spindle_clicked ()));
    connect (ui.radioButton_nkns_spindle_flap_wheel,     SIGNAL (clicked()), this, SLOT (radioButton_nkns_spindle_clicked ()));
    connect (ui.doubleSpinBox_nkns_spindle_zone_radius,SIGNAL (valueChanged (double)), this, SLOT (doubleSpinBox_nsks_spindle_zone_radius_changed(double)));
    //masking tool
    connect (ui.doubleSpinBox_masking_blend_radius,SIGNAL (valueChanged (double)), this, SLOT     (doubleSpinBox_masking_blend_radius_changed(double)));
    //impedance tool
    connect (ui.doubleSpinBox_impedance_blend_radius,SIGNAL (valueChanged (double)), this, SLOT   (doubleSpinBox_impedance_blend_radius_changed(double)));
    //sandblasting tool
    connect (ui.doubleSpinBox_sandblasting_zone_radius,SIGNAL (valueChanged (double)), this, SLOT (doubleSpinBox_sandblasting_zone_radius_changed(double)));


    //=======================================
    //slots for toolpath tab
    connect (ui.comboBox_toolpath_selection, SIGNAL (currentTextChanged(QString)), this, SLOT (comboBox_toolpath_selection_textChanged(QString)));
    connect (ui.pushButton_plot_toolpath, SIGNAL (clicked()), this, SLOT (pushButton_plot_toolpath_clicked()));
    connect (ui.pushButton_draw_simple_toolpath, SIGNAL (clicked()), this, SLOT  (pushButton_draw_simple_toolpath_clicked  ()));
    connect (ui.pushButton_clear_simple_toolpath, SIGNAL (clicked()), this, SLOT (pushButton_clear_simple_toolpath_clicked ()));
    connect (ui.pushButton_queue,       SIGNAL (clicked()), this, SLOT (pushButton_queue_clicked       ()));
    connect (ui.pushButton_reset_queue, SIGNAL (clicked()), this, SLOT (pushButton_reset_queue_clicked ()));

    //=======================================
    //slots for robot tab
    connect (ui.pushButton_robotSetup, SIGNAL (clicked()), this, SLOT (pushButton_robotSetup_clicked ()));
    connect (ui.pushButton_robotSim,   SIGNAL (clicked()), this, SLOT (pushButton_robotSim_clicked   ()));
    connect (ui.pushButton_robotRun,   SIGNAL (clicked()), this, SLOT (pushButton_robotRun_clicked   ()));
    connect (ui.pushButton_runScript,  SIGNAL (clicked()), this, SLOT (pushButton_runScript_clicked  ()));
    connect (ui.pushButton_refreshScript,  SIGNAL (clicked()), this, SLOT (pushButton_refreshScript_clicked  ()));
    connect (ui.pushButton_saveScriptAs,   SIGNAL (clicked()), this, SLOT (pushButton_saveScriptAs_clicked  ()));

  }




  /*********************
  ** Auto Run functions
  **********************/
  //check file exist
  std::string cmd;

  cmd = "rosrun atg_gui check_req_folders.py";  //cmd = "python src/atg_gui_module/script/check_req_folders.py";
  log(Info,std::string("Checked Required Files.."));
  system(cmd.c_str());
  ui.progressBar->setValue(30);
  ui.progressBar->update();

  //check
  comboBox_toolpath_selection_textChanged(ui.comboBox_toolpath_selection->currentText());
  readQSettings();
  pushButton_processNextPrev_clicked();
  cluster_no_ctrl(0, 'd');//disable cluster ctrl, only enable after segmentation completes

  //check robot settings tab
  //comboBox_tool_selection_clicked(ui.comboBox_tool_selection->currentText());//kicks in when readQSettings() is called.


  ui.progressBar->setValue(100);
  ui.progressBar->update();
}
ATG_Window::~ATG_Window(){/*delete ui;*/}

void ATG_Window::closeEvent(QCloseEvent *event)
{
  writeQSettings(); //save all variables
  std::cout << "Saved all UI variables.\n";
  QMainWindow::closeEvent(event);
}

/******************************************
** Logging
*******************************************/
void ATG_Window::log(const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
  case(Debug) : {
    logging_model_msg << "[DEBUG] [" << currentDateTime() << "]: " << msg;
    break;
  }
  case(Info) : {
    logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
    break;
  }
  case(Warn) : {
    logging_model_msg << "[WARN] [" << currentDateTime() << "]: " << msg;
    break;
  }
  case(Error) : {
    logging_model_msg << "[ERROR] [" << currentDateTime() << "]: " << msg;
    break;
  }
  case(Fatal) : {
    logging_model_msg << "[FATAL] [" << currentDateTime() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  updateLoggingView(); // used to readjust the scrollbar
}
void ATG_Window::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string ATG_Window::currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
  return buf;
}

/******************************************
** QSettings read/write
*******************************************/
void ATG_Window::readQSettings() //read settings when value change
{
    //QSettings settings("Qt-Ros Package", "diamond_gui"); //originally, goes into folder .config/Qt-Ros Package/diamond_gui.conf in home/<user>/
    QSettings settings(QString(config_file_dir_), QSettings::IniFormat);
    QString readValue = ""; bool readBool = 0;
    QString property_name = "";

    property_name = "Segmentation";
    readValue = settings.value(property_name+"/param/Smoothness", "1.5").toString();// settings.value() returns QVariant
    ui.spinBox_Smoothness->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/Curvature", "3").toString();
    ui.spinBox_Curvature->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/MinClusteSize", "500").toString();
    ui.spinBox_min_cluster_size->setValue(readValue.toInt());
    readValue = settings.value(property_name+"/param/Neighbour", "100").toString();
    ui.spinBox_Neighbour->setValue(readValue.toInt());
    readValue = settings.value(property_name+"/param/KSearch", "20").toString();
    ui.spinBox_KSearch->setValue(readValue.toInt());

//// will add back when open toolpath is in
//    if(coupon.file_name_short=="")
//    {
//        //dont load any snap points
//    }
//    else
//    {
//        readValue = settings.value("Draw/snap_pts/"+QString::fromStdString(coupon.file_name_short), "").toString();
//        ui.textEdit_snap_data->setText(readValue);
//    }


/*    readValue = settings.value("WorkObj/Frame/x", "0").toString();   //superseded by ReadQSettings_TCP_only()
//    ui.doubleSpinBox_object_x->setValue(readValue.toDouble());       //superseded by ReadQSettings_TCP_only()
//    readValue = settings.value("WorkObj/Frame/y", "0").toString();   //superseded by ReadQSettings_TCP_only()
//    ui.doubleSpinBox_object_y->setValue(readValue.toDouble());       //superseded by ReadQSettings_TCP_only()
//    readValue = settings.value("WorkObj/Frame/z", "0").toString();   //superseded by ReadQSettings_TCP_only()
//    ui.doubleSpinBox_object_z->setValue(readValue.toDouble());       //superseded by ReadQSettings_TCP_only()
//    readValue = settings.value("WorkObj/Frame/rx", "0").toString();  //superseded by ReadQSettings_TCP_only()
//    ui.doubleSpinBox_object_Rx->setValue(readValue.toDouble());      //superseded by ReadQSettings_TCP_only()
//    readValue = settings.value("WorkObj/Frame/ry", "0").toString();  //superseded by ReadQSettings_TCP_only()
//    ui.doubleSpinBox_object_Ry->setValue(readValue.toDouble());      //superseded by ReadQSettings_TCP_only()
//    readValue = settings.value("WorkObj/Frame/rz", "0").toString();  //superseded by ReadQSettings_TCP_only()
//    ui.doubleSpinBox_object_Rz->setValue(readValue.toDouble());      //superseded by ReadQSettings_TCP_only()*/

    readValue = settings.value("WorkObj/Fixed_z/z", "5").toString();
    ui.doubleSpinBox_fixed_z->setValue(readValue.toDouble());
    readValue = settings.value("WorkObj/Fixed_z/z_lift", "30").toString();
    ui.doubleSpinBox_fixed_z_lift->setValue(readValue.toDouble());
    readBool = settings.value("WorkObj/Fixed_z/checked", "0").toBool();
    ui.checkBox_fixed_z->setChecked(readBool);

    property_name = "Toolpath";
    readValue = settings.value(property_name+"/Toolsettings/resolution", "2").toString();
    ui.spinBox_toolpath_resolution->setValue(readValue.toDouble());
    readBool = settings.value(property_name+"/Toolsettings/force_resolution", "0").toBool();
    ui.checkbox_toolpath_resolution_forced->setChecked(readBool);
    readValue = settings.value(property_name+"/Toolsettings/hole_patch", "2").toString();
    ui.spinBox_toolpath_hole_patch->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/Toolsettings/step_size", "2").toString();
    ui.spinBox_toolpath_step_size->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/Toolsettings/offset", "2").toString();
    ui.spinBox_toolpath_offset->setValue(readValue.toDouble());
    readBool = settings.value(property_name+"/Toolsettings/select_external", "0").toBool();
    ui.checkBox_Select_External->setChecked(readBool);
    readBool = settings.value(property_name+"/Toolsettings/select_internal", "0").toBool();
    ui.checkBox_Select_Internal->setChecked(readBool);
    readBool = settings.value(property_name+"/Toolsettings/normal_flip", "0").toBool();
    ui.checkBox_normal_flip->setChecked(readBool);
    readValue = settings.value(property_name+"/Toolsettings/downsample", "1").toString();
    ui.spinBox_toolpath_downsample->setValue(readValue.toInt());
    readValue = settings.value(property_name+"/Toolsettings/path_rotate", "0").toString();
    ui.spinBox_Path_Rotate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/Toolsettings/KSearch", "500").toString();
    ui.spinBox_KSearch_TP_Surface->setValue(readValue.toInt());
    readValue = settings.value(property_name+"/Toolsettings/range_min", "0").toString();
    ui.spinBox_path_section_range_min->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/Toolsettings/range_max", "100").toString();
    ui.spinBox_path_section_range_max->setValue(readValue.toDouble());
    readBool = settings.value(property_name+"/Toolsettings/reverse_toolpath", "0").toBool();
    ui.checkbox_reverse_toolpath->setChecked(readBool);

    property_name = "tab_masking_tool";
    readValue = settings.value(property_name+"/param/feedrate", "10").toString();
    ui.doubleSpinBox_masking_feedrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/flowrate", "1").toString();
    ui.doubleSpinBox_masking_flowrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_speed", "50").toString();
    ui.doubleSpinBox_masking_lift_speed->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_height", "30").toString();
    ui.doubleSpinBox_masking_lift_height->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/blend", "0").toString();
    ui.doubleSpinBox_masking_blend_radius->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/step_size", "0").toString();
    ui.doubleSpinBox_masking_step_size->setValue(readValue.toDouble());

    property_name = "tab_spindle_tool";
    readValue = settings.value(property_name+"/param/tool_dia", "25").toString();
    ui.doubleSpinBox_nkns_spindle_tool_dia->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/tilt_angle", "3").toString();
    ui.doubleSpinBox_nkns_spindle_tilt_angle->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/step_size", "1").toString();
    ui.doubleSpinBox_nkns_spindle_step_size->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/feedrate", "10").toString();
    ui.doubleSpinBox_nkns_spindle_feedrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/force", "20").toString();
    ui.doubleSpinBox_nkns_spindle_force->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/spindlespeed", "1000").toString();
    ui.doubleSpinBox_nkns_spindle_spindle_speed->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/zone", "1").toString();
    ui.doubleSpinBox_nkns_spindle_zone_radius->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_speed", "50").toString();
    ui.doubleSpinBox_nkns_spindle_lift_speed->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_height", "30").toString();
    ui.doubleSpinBox_nkns_spindle_lift_height->setValue(readValue.toDouble());

    property_name = "tab_sandblasting_tool";
    readValue = settings.value(property_name+"/param/tilt_angle_x", "0").toString();
    ui.doubleSpinBox_sandblasting_tilt_angle_x->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/tilt_angle_y", "0").toString();
    ui.doubleSpinBox_sandblasting_tilt_angle_y->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/feedrate", "10").toString();
    ui.doubleSpinBox_sandblasting_feedrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/blast_dia", "5").toString();
    ui.doubleSpinBox_sandblasting_blast_dia->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/air_flow_rate", "5").toString();
    ui.doubleSpinBox_sandblasting_air_flow_rate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/step_size", "1").toString();
    ui.doubleSpinBox_sandblasting_step_size->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/zone", "1").toString();
    ui.doubleSpinBox_sandblasting_zone_radius->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_speed", "50").toString();
    ui.doubleSpinBox_sandblasting_lift_speed->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_height", "30").toString();
    ui.doubleSpinBox_sandblasting_lift_height->setValue(readValue.toDouble());

    property_name = "tab_impedance_tool";
    readValue = settings.value(property_name+"/param/feedrate", "10").toString();
    ui.doubleSpinBox_impedance_feedrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/rpm", "1000").toString();
    ui.doubleSpinBox_impedance_rpm->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_speed", "50").toString();
    ui.doubleSpinBox_impedance_lift_speed->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_height", "30").toString();
    ui.doubleSpinBox_impedance_lift_height->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/blend", "0").toString();
    ui.doubleSpinBox_impedance_blend_radius->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/step_size", "0").toString();
    ui.doubleSpinBox_impedance_step_size->setValue(readValue.toDouble());

    property_name = "tab_ndt_tool";
    readValue = settings.value(property_name+"/param/feedrate", "10").toString();
    ui.doubleSpinBox_ndt_feedrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/flowrate", "1").toString();
    ui.doubleSpinBox_ndt_flowrate->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_speed", "50").toString();
    ui.doubleSpinBox_ndt_lift_speed->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/lift_height", "30").toString();
    ui.doubleSpinBox_ndt_lift_height->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/blend", "0").toString();
    ui.doubleSpinBox_ndt_blend_radius->setValue(readValue.toDouble());
    readValue = settings.value(property_name+"/param/wait", "0").toString();
    ui.doubleSpinBox_ndt_wait->setValue(readValue.toDouble());

    readValue = settings.value("Settings/Robot/Robot", "0").toString();
    ui.comboBox_robot_selection->setCurrentIndex(readValue.toDouble());
    readValue = settings.value("Settings/Robot/Tool", "0").toString();
    ui.comboBox_tool_selection->setCurrentIndex(readValue.toDouble());
    readValue = settings.value("Settings/Robot/Env", "0").toString();
    ui.comboBox_env_selection->setCurrentIndex(readValue.toDouble());

    readValue = settings.value("Settings/SVD/pt_cloud_coords", "").toString();
    ui.textEdit_svd1->setText(readValue);
    readValue = settings.value("Settings/SVD/robot_coords", "").toString();
    ui.textEdit_svd2->setText(readValue);

    readQSettings_TCP_only();
    readQSettings_ENV_OBJ_only();
    readQSettings_Robot_IP_only();

    restoreGeometry(settings.value("geometry").toByteArray()); //port over from original readsettings
    restoreState(settings.value("windowState").toByteArray()); //port over from original readsettings

}

void ATG_Window::readQSettings_TCP_only()//read TCP and eeTCP only
{
  QSettings settings(QString(config_file_dir_), QSettings::IniFormat);
  QString readValue = ""; bool readBool = 0;
  //check tool and robot combobox for TCP and eeTCP
  QString tool_selection = ui.comboBox_tool_selection->currentText(); //may need to remove space and special chars, cant convert as QSetting uses % encoding as format has a severe restriction on syntax of keys
  QString X="0",Y="0",Z="0",R_="0",P_="0",Y_="0",eeX="0",eeY="0",eeZ="0",eeR_="0",eeP_="0",eeY_="0";
  //temp for adding, these values are ignored once there are values being stored in atg_config.ini
  if(tool_selection=="Masking Tool (old)")
  {X="0",Y="160",Z="216.5",R_="0",P_="0",Y_="0",eeX="0",eeY="0",eeZ="0",eeR_="0",eeP_="0",eeY_="0";}
  if(tool_selection=="Masking Tool")
  {X="-51.23",Y="-50.60",Z="178.63",R_="0",P_="0",Y_="0",eeX="0",eeY="0",eeZ="0",eeR_="0",eeP_="180",eeY_="45";}
  if(tool_selection=="Nakanishi Tool")
  {X="160.74",Y="1.70",Z="98.38",R_="0",P_="0",Y_="0",eeX="0",eeY="0",eeZ="0",eeR_="0",eeP_="0",eeY_="0";}
  if(tool_selection=="Impedance Tool")
  {X="0.59",Y="-2.1",Z="344.44",R_="0",P_="0",Y_="0",eeX="0",eeY="0",eeZ="0",eeR_="0",eeP_="0",eeY_="0";}
  if(tool_selection=="Sandblasting Nozzle Tool")
  {X="123.06",Y="0",Z="345.54",R_="0",P_="45",Y_="0",eeX="16.99",eeY="0",eeZ="239.47",eeR_="0",eeP_="45",eeY_="0";}
  if(tool_selection=="NDT Tool")
  {X="0",Y="0",Z="170",R_="0",P_="0",Y_="0",eeX="0",eeY="0",eeZ="0",eeR_="0",eeP_="0",eeY_="0";}

  readValue = settings.value(tool_selection+"/tcp/X", X).toString();
  ui.doubleSpinBox_robot_tcp_X->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/tcp/Y", Y).toString();
  ui.doubleSpinBox_robot_tcp_Y->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/tcp/Z", Z).toString();
  ui.doubleSpinBox_robot_tcp_Z->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/tcp/R_", R_).toString();
  ui.doubleSpinBox_robot_tcp_R_->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/tcp/P_", P_).toString();
  ui.doubleSpinBox_robot_tcp_P_->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/tcp/Y_", Y_).toString();
  ui.doubleSpinBox_robot_tcp_Y_->setValue(readValue.toDouble());

  readValue = settings.value(tool_selection+"/ee/X", eeX).toString();
  ui.doubleSpinBox_robot_eeX->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/ee/Y", eeY).toString();
  ui.doubleSpinBox_robot_eeY->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/ee/Z", eeZ).toString();
  ui.doubleSpinBox_robot_eeZ->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/ee/R_", eeR_).toString();
  ui.doubleSpinBox_robot_eeR_->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/ee/P_", eeP_).toString();
  ui.doubleSpinBox_robot_eeP_->setValue(readValue.toDouble());
  readValue = settings.value(tool_selection+"/ee/Y_", eeY_).toString();
  ui.doubleSpinBox_robot_eeY_->setValue(readValue.toDouble());
}

void ATG_Window::readQSettings_ENV_OBJ_only()//read Environment and Obj Frame only
{
  QSettings settings(QString(config_file_dir_), QSettings::IniFormat);
  QString readValue = ""; bool readBool = 0;
  //check tool and robot combobox for TCP and eeTCP
  QString env_selection = ui.comboBox_env_selection->currentText(); //may need to remove space and special chars, cant convert as QSetting uses % encoding as format has a severe restriction on syntax of keys
  QString env_X="0",env_Y="0",env_Z="0",env_R_="0",env_P_="0",env_Y_="0",
          obj_X="0",obj_Y="0",obj_Z="0",obj_R_="0",obj_P_="0",obj_Y_="0";
  //temp for adding, these values are ignored once there are values being stored in atg_config.ini
  if(env_selection=="Masking Table (old)")
  {env_X="-130",env_Y="-220",env_Z="-25",env_R_="90",env_P_="0",env_Y_="90",
   obj_X="0",obj_Y="0",obj_Z="0",obj_R_="0",obj_P_="0",obj_Y_="0";}
  if(env_selection=="Masking Table")
  {env_X="-282.84",env_Y="14.14",env_Z="0",env_R_="0",env_P_="180",env_Y_="225",
   obj_X="-696.59",obj_Y="-603.54",obj_Z="71.52",obj_R_="-0.45",obj_P_="0",obj_Y_="-45.4";}
  if(env_selection=="Round Table R06")
  {env_X="1000",env_Y="500",env_Z="500",env_R_="0",env_P_="0",env_Y_="0",
   obj_X="0",obj_Y="0",obj_Z="0",obj_R_="0",obj_P_="0",obj_Y_="0";}

  readValue = settings.value(env_selection+"/env/X", env_X).toString();
  ui.doubleSpinBox_env_X->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/env/Y", env_Y).toString();
  ui.doubleSpinBox_env_Y->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/env/Z", env_Z).toString();
  ui.doubleSpinBox_env_Z->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/env/R_", env_R_).toString();
  ui.doubleSpinBox_env_R_->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/env/P_", env_P_).toString();
  ui.doubleSpinBox_env_P_->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/env/Y_", env_Y_).toString();
  ui.doubleSpinBox_env_Y_->setValue(readValue.toDouble());

  readValue = settings.value(env_selection+"/obj/X", obj_X).toString();
  ui.doubleSpinBox_object_X->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/obj/Y", obj_Y).toString();
  ui.doubleSpinBox_object_Y->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/obj/Z", obj_Z).toString();
  ui.doubleSpinBox_object_Z->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/obj/R_", obj_R_).toString();
  ui.doubleSpinBox_object_R_->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/obj/P_", obj_P_).toString();
  ui.doubleSpinBox_object_P_->setValue(readValue.toDouble());
  readValue = settings.value(env_selection+"/obj/Y_", obj_Y_).toString();
  ui.doubleSpinBox_object_Y_->setValue(readValue.toDouble());

}

void ATG_Window::readQSettings_Robot_IP_only()
{
  QSettings settings(QString(config_file_dir_), QSettings::IniFormat);
  QString readValue = ""; bool readBool = 0;
  //check tool and robot combobox for TCP and eeTCP
  QString robot_selection = ui.comboBox_robot_selection->currentText();
  QString default_IP = "192.168.0.100";
  QString default_Port = "30002";
  if(robot_selection=="ABB IRB 1200"
   ||robot_selection=="ABB IRB 2400"
   ||robot_selection=="ABB IRB 2600")
  {
    default_IP = "192.168.125.1";
    default_Port = "80";
  }

  readValue = settings.value(robot_selection+"/param/IP", default_IP).toString();
  ui.line_edit_robot_ip->setText(readValue);
  readValue = settings.value(robot_selection+"/param/Port", default_Port).toString();
  ui.spinBox_robot_port->setValue(readValue.toInt());
}

void ATG_Window::writeQSettings() //write settings when value change
{
    //QSettings settings("Qt-Ros Package", "diamond_gui");//originally
    QSettings settings(QString(config_file_dir_), QSettings::IniFormat);
    QString property_name = "";
//    settings.setValue("WorkObj/Frame/x",  QString::number(ui.doubleSpinBox_object_x ->value(),'f',2));
//    settings.setValue("WorkObj/Frame/y",  QString::number(ui.doubleSpinBox_object_y ->value(),'f',2));
//    settings.setValue("WorkObj/Frame/z",  QString::number(ui.doubleSpinBox_object_z ->value(),'f',2));
//    settings.setValue("WorkObj/Frame/rx", QString::number(ui.doubleSpinBox_object_Rx->value(),'f',2));
//    settings.setValue("WorkObj/Frame/ry", QString::number(ui.doubleSpinBox_object_Ry->value(),'f',2));
//    settings.setValue("WorkObj/Frame/rz", QString::number(ui.doubleSpinBox_object_Rz->value(),'f',2));

    property_name = "WorkObj";
    settings.setValue(property_name+"/Fixed_z/z",       QString::number(ui.doubleSpinBox_fixed_z     ->value(),'f',2));
    settings.setValue(property_name+"/Fixed_z/z_lift",  QString::number(ui.doubleSpinBox_fixed_z_lift->value(),'f',2));
    settings.setValue(property_name+"/Fixed_z/checked", QString::number(ui.checkBox_fixed_z      ->isChecked()));

    property_name = "Toolpath";
    settings.setValue(property_name+"/Toolsettings/resolution", QString::number(ui.spinBox_toolpath_resolution->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/force_resolution", QString::number(ui.checkbox_toolpath_resolution_forced->isChecked()));
    settings.setValue(property_name+"/Toolsettings/hole_patch",  QString::number(ui.spinBox_toolpath_hole_patch ->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/step_size",   QString::number(ui.spinBox_toolpath_step_size  ->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/offset",      QString::number(ui.spinBox_toolpath_offset     ->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/select_external", QString::number(ui.checkBox_Select_External->isChecked()));
    settings.setValue(property_name+"/Toolsettings/select_internal", QString::number(ui.checkBox_Select_Internal->isChecked()));
    settings.setValue(property_name+"/Toolsettings/normal_flip",     QString::number(ui.checkBox_normal_flip    ->isChecked()));
    settings.setValue(property_name+"/Toolsettings/downsample",   QString::number(ui.spinBox_toolpath_downsample ->value()));
    settings.setValue(property_name+"/Toolsettings/path_rotate",  QString::number(ui.spinBox_Path_Rotate ->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/KSearch",      QString::number(ui.spinBox_KSearch_TP_Surface ->value()));
    settings.setValue(property_name+"/Toolsettings/range_min",    QString::number(ui.spinBox_path_section_range_min ->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/range_max",    QString::number(ui.spinBox_path_section_range_max ->value(),'f',2));
    settings.setValue(property_name+"/Toolsettings/reverse_toolpath",     QString::number(ui.checkbox_reverse_toolpath ->isChecked()));

    property_name = "Segmentation";
    settings.setValue(property_name+"/param/KSearch",       QString::number(ui.spinBox_KSearch         ->value()));
    settings.setValue(property_name+"/param/Neighbour",     QString::number(ui.spinBox_Neighbour       ->value()));
    settings.setValue(property_name+"/param/MinClusteSize", QString::number(ui.spinBox_min_cluster_size->value()));
    settings.setValue(property_name+"/param/Curvature",     QString::number(ui.spinBox_Curvature       ->value(),'f',2));
    settings.setValue(property_name+"/param/Smoothness",    QString::number(ui.spinBox_Smoothness      ->value(),'f',2));

    property_name = "tab_masking_tool";
    settings.setValue(property_name+"/param/feedrate",   QString::number(ui.doubleSpinBox_masking_feedrate    ->value(),'f',2));
    settings.setValue(property_name+"/param/flowrate",   QString::number(ui.doubleSpinBox_masking_flowrate    ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_speed", QString::number(ui.doubleSpinBox_masking_lift_speed  ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_height",QString::number(ui.doubleSpinBox_masking_lift_height ->value(),'f',2));
    settings.setValue(property_name+"/param/blend",      QString::number(ui.doubleSpinBox_masking_blend_radius->value(),'f',2));
    settings.setValue(property_name+"/param/step_size",  QString::number(ui.doubleSpinBox_masking_step_size   ->value(),'f',2));

    property_name = "tab_spindle_tool";
    settings.setValue(property_name+"/param/tool_dia",     QString::number(ui.doubleSpinBox_nkns_spindle_tool_dia     ->value(),'f',2));
    settings.setValue(property_name+"/param/tilt_angle",   QString::number(ui.doubleSpinBox_nkns_spindle_tilt_angle   ->value(),'f',2));
    settings.setValue(property_name+"/param/step_size",    QString::number(ui.doubleSpinBox_nkns_spindle_step_size    ->value(),'f',2));
    settings.setValue(property_name+"/param/feedrate",     QString::number(ui.doubleSpinBox_nkns_spindle_feedrate     ->value(),'f',2));
    settings.setValue(property_name+"/param/force",        QString::number(ui.doubleSpinBox_nkns_spindle_force        ->value(),'f',2));
    settings.setValue(property_name+"/param/spindlespeed", QString::number(ui.doubleSpinBox_nkns_spindle_spindle_speed->value()));
    settings.setValue(property_name+"/param/zone",         QString::number(ui.doubleSpinBox_nkns_spindle_zone_radius  ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_speed",   QString::number(ui.doubleSpinBox_nkns_spindle_lift_speed   ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_height",  QString::number(ui.doubleSpinBox_nkns_spindle_lift_height  ->value(),'f',2));

    property_name = "tab_sandblasting_tool";
    settings.setValue(property_name+"/param/tilt_angle_x", QString::number(ui.doubleSpinBox_sandblasting_tilt_angle_x ->value(),'f',2));
    settings.setValue(property_name+"/param/tilt_angle_y", QString::number(ui.doubleSpinBox_sandblasting_tilt_angle_y ->value(),'f',2));
    settings.setValue(property_name+"/param/feedrate",     QString::number(ui.doubleSpinBox_sandblasting_feedrate     ->value(),'f',2));
    settings.setValue(property_name+"/param/blast_dia",    QString::number(ui.doubleSpinBox_sandblasting_blast_dia    ->value(),'f',2));
    settings.setValue(property_name+"/param/air_flow_rate",QString::number(ui.doubleSpinBox_sandblasting_air_flow_rate->value(),'f',2));
    settings.setValue(property_name+"/param/step_size",    QString::number(ui.doubleSpinBox_sandblasting_step_size    ->value(),'f',2));
    settings.setValue(property_name+"/param/zone",         QString::number(ui.doubleSpinBox_sandblasting_zone_radius  ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_speed",   QString::number(ui.doubleSpinBox_sandblasting_lift_speed   ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_height",  QString::number(ui.doubleSpinBox_sandblasting_lift_height  ->value(),'f',2));

    property_name = "tab_impedance_tool";
    settings.setValue(property_name+"/param/feedrate",   QString::number(ui.doubleSpinBox_impedance_feedrate    ->value(),'f',2));
    settings.setValue(property_name+"/param/rpm",        QString::number(ui.doubleSpinBox_impedance_rpm         ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_speed", QString::number(ui.doubleSpinBox_impedance_lift_speed  ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_height",QString::number(ui.doubleSpinBox_impedance_lift_height ->value(),'f',2));
    settings.setValue(property_name+"/param/step_size",  QString::number(ui.doubleSpinBox_impedance_step_size   ->value(),'f',2));
    settings.setValue(property_name+"/param/blend",      QString::number(ui.doubleSpinBox_impedance_blend_radius->value(),'f',2));

    property_name = "tab_ndt_tool";
    settings.setValue(property_name+"/param/feedrate",   QString::number(ui.doubleSpinBox_ndt_feedrate    ->value(),'f',2));
    settings.setValue(property_name+"/param/flowrate",   QString::number(ui.doubleSpinBox_ndt_flowrate    ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_speed", QString::number(ui.doubleSpinBox_ndt_lift_speed  ->value(),'f',2));
    settings.setValue(property_name+"/param/lift_height",QString::number(ui.doubleSpinBox_ndt_lift_height ->value(),'f',2));
    settings.setValue(property_name+"/param/blend",      QString::number(ui.doubleSpinBox_ndt_blend_radius->value(),'f',2));
    settings.setValue(property_name+"/param/wait",       QString::number(ui.doubleSpinBox_ndt_wait        ->value(),'f',2));

    settings.setValue("Settings/Robot/Robot",    QString::number(ui.comboBox_robot_selection ->currentIndex()));
    settings.setValue("Settings/Robot/Tool",     QString::number(ui.comboBox_tool_selection  ->currentIndex()));
    settings.setValue("Settings/Robot/Env",      QString::number(ui.comboBox_env_selection   ->currentIndex()));

    //check tool and robot combobox for TCP and eeTCP for -> ReadQSettings_TCP_only()
    QString tool_selection = ui.comboBox_tool_selection->currentText(); //may need to remove space and special chars, cant convert as QSetting uses % encoding as format has a severe restriction on syntax of keys
    settings.setValue(tool_selection+"/tcp/X",  QString::number(ui.doubleSpinBox_robot_tcp_X ->value(),'f',2));
    settings.setValue(tool_selection+"/tcp/Y",  QString::number(ui.doubleSpinBox_robot_tcp_Y ->value(),'f',2));
    settings.setValue(tool_selection+"/tcp/Z",  QString::number(ui.doubleSpinBox_robot_tcp_Z ->value(),'f',2));
    settings.setValue(tool_selection+"/tcp/R_", QString::number(ui.doubleSpinBox_robot_tcp_R_->value(),'f',2));
    settings.setValue(tool_selection+"/tcp/P_", QString::number(ui.doubleSpinBox_robot_tcp_P_->value(),'f',2));
    settings.setValue(tool_selection+"/tcp/Y_", QString::number(ui.doubleSpinBox_robot_tcp_Y_->value(),'f',2));
    settings.setValue(tool_selection+"/ee/X",  QString::number(ui.doubleSpinBox_robot_eeX ->value(),'f',2));
    settings.setValue(tool_selection+"/ee/Y",  QString::number(ui.doubleSpinBox_robot_eeY ->value(),'f',2));
    settings.setValue(tool_selection+"/ee/Z",  QString::number(ui.doubleSpinBox_robot_eeZ ->value(),'f',2));
    settings.setValue(tool_selection+"/ee/R_", QString::number(ui.doubleSpinBox_robot_eeR_->value(),'f',2));
    settings.setValue(tool_selection+"/ee/P_", QString::number(ui.doubleSpinBox_robot_eeP_->value(),'f',2));
    settings.setValue(tool_selection+"/ee/Y_", QString::number(ui.doubleSpinBox_robot_eeY_->value(),'f',2));

    //check tool and robot combobox for TCP and eeTCP for -> ReadQSettings_ENV_OBJ_only()
    QString env_selection = ui.comboBox_env_selection->currentText(); //may need to remove space and special chars, cant convert as QSetting uses % encoding as format has a severe restriction on syntax of keys
    settings.setValue(env_selection+"/env/X",  QString::number(ui.doubleSpinBox_env_X ->value(),'f',2));
    settings.setValue(env_selection+"/env/Y",  QString::number(ui.doubleSpinBox_env_Y ->value(),'f',2));
    settings.setValue(env_selection+"/env/Z",  QString::number(ui.doubleSpinBox_env_Z ->value(),'f',2));
    settings.setValue(env_selection+"/env/R_", QString::number(ui.doubleSpinBox_env_R_->value(),'f',2));
    settings.setValue(env_selection+"/env/P_", QString::number(ui.doubleSpinBox_env_P_->value(),'f',2));
    settings.setValue(env_selection+"/env/Y_", QString::number(ui.doubleSpinBox_env_Y_->value(),'f',2));

    settings.setValue(env_selection+"/obj/X",  QString::number(ui.doubleSpinBox_object_X ->value(),'f',2));
    settings.setValue(env_selection+"/obj/Y",  QString::number(ui.doubleSpinBox_object_Y ->value(),'f',2));
    settings.setValue(env_selection+"/obj/Z",  QString::number(ui.doubleSpinBox_object_Z ->value(),'f',2));
    settings.setValue(env_selection+"/obj/R_", QString::number(ui.doubleSpinBox_object_R_->value(),'f',2));
    settings.setValue(env_selection+"/obj/P_", QString::number(ui.doubleSpinBox_object_P_->value(),'f',2));
    settings.setValue(env_selection+"/obj/Y_", QString::number(ui.doubleSpinBox_object_Y_->value(),'f',2));

    //check robot selection combobox
    QString robot_selection = ui.comboBox_robot_selection->currentText();
    settings.setValue(robot_selection+"/param/IP",   ui.line_edit_robot_ip->text());
    settings.setValue(robot_selection+"/param/Port",  QString::number(ui.spinBox_robot_port->value()));

    //check svd textEdit
    settings.setValue("Settings/SVD/pt_cloud_coords",  ui.textEdit_svd1->toPlainText());
    settings.setValue("Settings/SVD/robot_coords",     ui.textEdit_svd2->toPlainText());

//// will add back when open file is added
//    if(coupon.file_name_short=="")
//    {
//        //dont load any snap points
//    }
//    else
//    {
//        settings.setValue("Draw/snap_pts/"+QString::fromStdString(coupon.file_name_short),    ui.textEdit_snap_data->toPlainText());
//    }

    settings.setValue("geometry", saveGeometry()); //port over from original writesetting
    settings.setValue("windowState", saveState()); //port over from original writesetting

}

/******************************************
** SLOTS for tab control << and >> buttons
*******************************************/
void ATG_Window::pushButton_processNextPrev_clicked(){
    //identifying button text which is clicked
    QString buttonText = "";
    QPushButton* pButton = qobject_cast<QPushButton*>(sender());
    if(pButton)  buttonText = pButton->text();
    if(buttonText.toStdString()==">>>") ui.spinBox_process_nextpre->stepBy(1);
    if(buttonText.toStdString()=="<<<") ui.spinBox_process_nextpre->stepBy(-1);
    ui.spinBox_process_nextpre->findChild<QLineEdit*>()->deselect();
    //std::cout <<"buttontext = "<< ui.spinBox_process_nextpre->value() << std::endl;
    QLabel *lbl = new QLabel();QFont lblfont("Arial",10);
    switch(ui.spinBox_process_nextpre->value())//+10 to ignore
    {
    case 1:
        lbl = this->findChild<QLabel *>("lblFileOption"); //check label
        if(!lbl)
        {
            ui.tabWidget_Flow->insertTab(0,ui.tab_File,"");
            lbl = new QLabel();
            lbl->setObjectName("lblFileOption");
            lbl->setText("\nFile\nOptions\n");
            lbl->setFont(lblfont);
            ui.tabWidget_Flow->tabBar()->setTabButton(0,QTabBar::LeftSide,lbl);
            ui.tabWidget_Flow->removeTab(1);
        }
        //else do Nothing
        break;
    /*case 2:
        lbl = this->findChild<QLabel *>("lblFrame"); //check label
        if(!lbl)
        {
            ui.tabWidget_Flow->insertTab(0,ui.tab_Frame,"");
            lbl = new QLabel();
            lbl->setObjectName("lblFrame");
            lbl->setText("\nFrame\nCLBR.\n");
            lbl->setFont(lblfont);
            ui.tabWidget_Flow->tabBar()->setTabButton(0,QTabBar::LeftSide,lbl);
            ui.tabWidget_Flow->removeTab(1);
        }
        //else do Nothing
        //ui.tabWidget_Flow->setCurrentIndex(1);
        break;*/
    case 2:
        lbl = this->findChild<QLabel *>("lblSurface"); //check label
        if(!lbl)
        {
            ui.tabWidget_Flow->insertTab(0,ui.tab_Seg,"");
            lbl = new QLabel();
            lbl->setObjectName("lblSurface");
            lbl->setText("\nSurface\n");
            lbl->setFont(lblfont);
            ui.tabWidget_Flow->tabBar()->setTabButton(0,QTabBar::LeftSide,lbl);
            ui.tabWidget_Flow->removeTab(1);
        }
        //else do Nothing
        //ui.tabWidget_Flow->setCurrentIndex(2);
        break;
    case 3:
        lbl = this->findChild<QLabel *>("lblWorkCell"); //check label
        if(!lbl)
        {
            ui.tabWidget_Flow->insertTab(0,ui.tab_Settings,"");
            lbl = new QLabel();
            lbl->setObjectName("lblWorkCell");
            lbl->setText("\nWorkcell\nSettings\n");
            lbl->setFont(lblfont);
            ui.tabWidget_Flow->tabBar()->setTabButton(0,QTabBar::LeftSide,lbl);
            ui.tabWidget_Flow->removeTab(1);
        }
        //else do Nothing
        //ui.tabWidget_Flow->setCurrentIndex(3);
        break;
    case 4:
        lbl = this->findChild<QLabel *>("lblToolpath"); //check label
        if(!lbl)
        {
            ui.tabWidget_Flow->insertTab(0,ui.tab_Toolpath,"");
            lbl = new QLabel();
            lbl->setObjectName("lblToolpath");
            lbl->setText("\nToolpath\nPlan\n");
            lbl->setFont(lblfont);
            ui.tabWidget_Flow->tabBar()->setTabButton(0,QTabBar::LeftSide,lbl);
            ui.tabWidget_Flow->removeTab(1);
        }
        //else do Nothing
        //ui.tabWidget_Flow->setCurrentIndex(4);
        break;
    case 5:
        lbl = this->findChild<QLabel *>("lblRobotCode"); //check label
        if(!lbl)
        {
            ui.tabWidget_Flow->insertTab(0,ui.tab_Robot,"");
            lbl = new QLabel();
            lbl->setObjectName("lblRobotCode");
            lbl->setText("\nRobot\nCode\n");
            lbl->setFont(lblfont);
            ui.tabWidget_Flow->tabBar()->setTabButton(0,QTabBar::LeftSide,lbl);
            ui.tabWidget_Flow->removeTab(1);
        }
        //else do Nothing
        //ui.tabWidget_Flow->setCurrentIndex(5);
        break;
    }
    writeQSettings();
}

void ATG_Window::pushButton_quit_clicked() {
  QMessageBox::StandardButton msgBox;
  msgBox = QMessageBox::question(this, "Quit "
                                       "the system?", "Are you sure to quit?",
                                 QMessageBox::Yes|QMessageBox::No);
  if (msgBox == QMessageBox::Yes) {
    //qDebug() << "Yes was clicked";
    close();
  } else {
    //qDebug() << "Yes was *not* clicked";
  }
}

/******************************************
** SLOTS for Files tab
*******************************************/
void ATG_Window::pushButton_close_clicked(){
  QMessageBox::StandardButton msgBox;
  msgBox = QMessageBox::question(this, "Close the project?", "Are you sure to close this project?",
                                 QMessageBox::Yes|QMessageBox::No);
  if (msgBox == QMessageBox::Yes) {
    //qDebug() << "Yes was clicked";
    viewer->removeAllPointClouds();
    viewer->removeAllCoordinateSystems();
    viewer->removeAllShapes();
    viewer->addCoordinateSystem(10.0,"world");
    cluster_no_ctrl(0, 'd');//disable cluster ctrl, only enable after segmentation completes
    ui.tab_draw->setEnabled(false);
    log(Info,std::string("All files are closed"));
    ui.progressBar->setValue(0);
    g_cloud_opened_->clear();
    ui.qvtkWidget->update();//update viewer view
    g_open_filename_="";
    g_short_filename_="";
    cluster_centroids_.clear();
  } else {
    //qDebug() << "Yes was *not* clicked";
  }
}

void ATG_Window::pushButton_open_clicked(QString assigned_inputfilename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZ>);
  //get a filename to open
  QString exDir =  ex_data_file_dir_; // "/home/data/"
  g_open_filename_ = assigned_inputfilename;
  if (assigned_inputfilename=="")//skips this if there's already an assigned name function input variable
  {
    g_open_filename_ = QFileDialog::getOpenFileName(this,
    tr("Open Point Cloud Data File"), exDir, tr("Point Cloud Data Files (*.obj *.pcd)"));
    //tr("Open Point Cloud Data File"), "/home/diamond_ws/data/", tr("Point Cloud Data Files (*.obj) *.pcd "));
  }
  if (g_open_filename_.length() == 0){
    QMessageBox::information(NULL, tr("Oops"), tr("You didn't select any files."));
  }else{
    ui.progressBar->setValue(3); ui.progressBar->update();
    std::cout << "Got filename: " << g_open_filename_.toStdString() << std::endl;
    log(Info,std::string("Loading the point cloud data......"));
    cloud_origin = load_point_cloud_data(g_open_filename_.toStdString(),1);
    Eigen::Vector4f centroid_cluster;
    pcl::compute3DCentroid(*cloud_origin,centroid_cluster);
    g_cloud_opened_centroid.x = centroid_cluster[0];
    g_cloud_opened_centroid.y = centroid_cluster[1];
    g_cloud_opened_centroid.z = centroid_cluster[2];
    std::cout <<"visualized the raw point cloud data" << "\n";
//    string centroid_pose ="Centroid position =[" + to_string(coupon.centroid[0])+","+to_string(coupon.centroid[1])+","+to_string(coupon.centroid[2])+"]";
    ui.progressBar->setValue(50); ui.progressBar->update();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->removeAllCoordinateSystems();
    viewer->addText("", 10, 20, 15, 1, 1, 1, "name", 0);     //original y was 680
    viewer->addText("", 10, 50, 15, 1, 1, 1, "clusters", 0); //original y was 620
//    viewer->addText(centroid_pose, 10, 50, 15, 1, 1, 1, "centroid", 0); //original y was 650
//    viewer->addText("", 10, 590, 15, 1, 1, 1, "Planes", 0);   //original y was 590
//    viewer->addText("", 10, 560, 15, 1, 1, 1, "Cyliners", 0); //original y was 560
//    viewer->addText("", 10, 530, 15, 1, 1, 1, "Curves", 0);   //original y was 530
    viewer->updateText(g_open_filename_.toStdString(), 10, 20, 1.0, 1.0, 1.0, "name");//original y was 680
    viewer->addPointCloud (cloud_origin, "cloud_main");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7f, 0.7f, 0.7f, "cloud_main"); //grey cloud, non processed
    viewer->addCoordinateSystem(10, g_cloud_opened_centroid.x, g_cloud_opened_centroid.y, g_cloud_opened_centroid.z, "centroid pose");
    viewer->addCoordinateSystem(10.0,"world");
    viewer->setWindowName("point cloud data visulization");
    viewer->resetCamera ();
    ui.qvtkWidget->update ();
    log(Info,std::string("Point cloud data has been successfully loaded!"));
    g_cloud_opened_ = cloud_origin; //josh added for interactive GUI
    g_kdtree_cloud_open_.setInputCloud (g_cloud_opened_); //josh added for interactive GUI
//    //align_pcd_to_object_frame();//josh added for automatically aligning frame //not required with mesh

    //==== START - get short name of file, to assist in loading QSettings for particular file
    std::string short_name = g_open_filename_.toStdString();
    // Remove directory if present.
    // Do this before extension removal incase directory has a period character.
    const size_t last_slash_idx = short_name.find_last_of("\\/");
    if (std::string::npos != last_slash_idx)
    {
        short_name.erase(0, last_slash_idx + 1);
    }

    // Remove extension if present.
    const size_t period_idx = short_name.rfind('.');
    if (std::string::npos != period_idx)
    {
        short_name.erase(period_idx);
    }
    g_short_filename_ = QString::fromStdString(short_name);
    std::cout<<"Opened file short name = " <<g_short_filename_.toStdString()<< std::endl;
    //==== End - get short name of file, to assist in loading QSettings for particular file

    cluster_no_ctrl(0, 'd');//disable cluster ctrl, only enable after segmentation completes
    ui.tab_draw->setEnabled(false);
    ui.progressBar->setValue(100); ui.progressBar->update();
    //josh added auto execute  option
    if(ui.checkBox_auto_seg->isChecked())
    {
      pushButton_segmentation_clicked();
    }
  }
  readQSettings();//josh read settings to populate variables from specific filename

}

/******************************************
** SLOTS for segmentation tab
*******************************************/
void ATG_Window::pushButton_segmentation_clicked()
{
  if (g_open_filename_.length()<1)//no opened file
  {
    log(Info, std::string("No opened file for segmenation."));
    return;
  }

  //start segmentation node from atg_surace_identification module
  //reset signal and disable relavant buttons and tabs
  read_write_fsignal("segmentation_signal",'w',"percent\n0");
  ui.tabWidget_file->setEnabled(false);
  ui.tabWidget_seg->setEnabled(false);
  ui.tabWidget_toolpath->setEnabled(false);
  cluster_centroids_.clear();
  cluster_no_ctrl(0, 'd');//disable cluster ctrl, only enable after segmentation completes
  log(Info, std::string("Point cloud data segmenation in progress......"));
  std::string cmd = "xterm -iconic -hold -e 'rosrun atg_surface_identification atg_rg_seg " + //-hold to hold, +hold to close
      g_open_filename_.toStdString()                        +" "+
      std::to_string(ui.spinBox_Smoothness->value()       )+" "+
      std::to_string(ui.spinBox_Curvature->value()        )+" "+
      std::to_string(ui.spinBox_min_cluster_size->value() )+" "+
      std::to_string(ui.spinBox_KSearch->value()          )+" "+
      std::to_string(ui.spinBox_Neighbour->value()        )+"' &";
  system(cmd.c_str());
  cmd = "sleep 1; xdotool search --class xterm set_desktop_for_window %@ 1";
  system(cmd.c_str());

  //run timer to launch and track segmentation node
  QTimer *seg_timer = new QTimer(this);
  connect (seg_timer, SIGNAL(timeout()), this, SLOT(seg_thread()));   //for segmentation timer
  seg_timer->start(200); //time specified in ms       //for segmentation timer
  g_seg_timer_ = seg_timer;

}

float fake_fsignal = 0.0;
float last_fsignal;
void ATG_Window::seg_thread()
{
      //subsequent entry, track segmentation node progress through signal
      g_seg_timer_->setInterval(200);
      //check signal
      std::vector<std::string> lines;float current_fsignal;
      lines=read_write_fsignal("segmentation_signal",'r');
//      if(lines[0]=="no msg") return;//continue to next interval
//      else if (lines[0]=="no msg")
//      {
//        current_fsignal=std::stod(line);
//      }
      //error msg
      if(lines[0] == "error msg")
      {
        g_seg_timer_->stop();
        ui.progressBar->setValue(0);
        ui.progressBar->update();
        QMessageBox::information(NULL, tr("Oops"), tr(lines[1].c_str()));
        //prompt error and enable buttons
      }
      else if (lines[0] == "percent")
      {
        current_fsignal=std::stod(lines[1]);
        //check is signal is stuck on number
        if(last_fsignal==current_fsignal && current_fsignal<60 && fake_fsignal<50) fake_fsignal+=0.2; //fake_fsignal is proxi for 10-60% where normal est and region growing is processing
        if(current_fsignal>=60) fake_fsignal=0;
        last_fsignal = current_fsignal;

        //output progress
        ui.progressBar->setValue(current_fsignal+fake_fsignal);
        ui.progressBar->update();
        if (current_fsignal<80) return; //update progress and go to next loop, not continuing to stop thread
      }
      else// no msg or invalid msg, continue to next interval
      {
        return;
      }

      //progress up till here means fsignal is >= 80 or there's an error in toolpath node, stop thread

//      if (current_fsignal>=80)//(ui.progressBar->value()>=80)//need to read files here
//      {
        //stop signal to prevent repeat of timer running seg_thread() again
        g_seg_timer_->stop();

        //read cluster folder
        viewer->removeAllPointClouds(0);
        read_pcd_in_cluster_folder();

        //reset all variables and ui
        fake_fsignal=0;
        read_write_fsignal("segmentation_signal",'d');
        ui.tabWidget_file->setEnabled(true);
        ui.tabWidget_seg->setEnabled(true);
        ui.tabWidget_toolpath->setEnabled(true);
        cluster_no_ctrl(0, 'e');//enable cluster ctrl, only enable after segmentation completes
//      }
}

std::vector<std::string> ATG_Window::read_write_fsignal(std::string signal, char action, std::string msg)
{
  std::vector<std::string> lines;
  std::string sfilename = "data/coupons/"+signal+".txt"; //<package>/data/coupons/<signal>.txt
  if (action == 'r')
  {
    std::string line;float current_fsignal;
    std::ifstream fsignal;
    fsignal.open(sfilename);
    if (fsignal.is_open()) {
        while (std::getline(fsignal, line)) {
          lines.push_back(line);
          //std::cout << "\rfsignal: " << line;// << stdendl;
          //qDebug () <<  ;
        }
        fsignal.close();
    }
    if (line.size()<1) lines.push_back("no msg");//return "no msg";
    return lines;
  }
  else if (action == 'w')
  {
    std::ofstream fsignal;
    fsignal.open(sfilename,std::ofstream::out);
    fsignal << msg;
    fsignal.close();
    return lines;
  }
  else if(action== 'd')
  {
    if(boost::filesystem::exists(sfilename))std::remove(&sfilename[0]);
  }
  return lines;
}

void ATG_Window::read_pcd_in_cluster_folder()
{
  //update progress bar from 80-100
  int cluster_no = 0;
  remove_seg_labels();
  pushButton_clear_simple_toolpath_clicked();
  //find --> "data/coupons/clusters/cluster_"+std::to_string(cluster_no)+".pcd"
  //find how many cluster available
  while(1)
  {
    std::string cl_filename = "cluster_"+std::to_string(cluster_no);
    std::string curr_custer_file = in_data_file_dir_.toStdString()+"clusters/"+cl_filename+".pcd";
    if(boost::filesystem::exists(curr_custer_file))
      cluster_no++;
    else
      break;
  }

  //load all clusters found in folder
  for(int i =-1;i<cluster_no;i++)
  {
    if(i!=-1)
    {
      std::string cl_filename = "cluster_"+std::to_string(i);
      std::string curr_custer_file = in_data_file_dir_.toStdString()+"clusters/"+cl_filename+".pcd";
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cl (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile <pcl::PointXYZ> (curr_custer_file, *cloud_cl);
      viewer->addPointCloud (cloud_cl, cl_filename);
      //random color while avoiding red and color near background color
      float r = rand () / (RAND_MAX + 1.0f)/3*2, g = rand () / (RAND_MAX + 1.0f),b = rand () / (RAND_MAX + 1.0f);
      while(1)
      {
        if(fabs(g-0)>0.15 && fabs(b-0)>0.15) break;
        g = rand () / (RAND_MAX + 1.0f);b = rand () / (RAND_MAX + 1.0f);
      }

      //if (cl==coupon.clusters_no){ r=1;g=0;b=0;}//to load rejected points
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cl_filename);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cl_filename);
      std::cout << "cluster_" + std::to_string(i) + " has " << cloud_cl->points.size() << " points." << std::endl;

      //show normals only on checkbox
      if(ui.checkBox_show_normals->isChecked())
      {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_nl(new pcl::PointCloud<pcl::Normal>);
        pcl::io::loadPCDFile <pcl::Normal> (curr_custer_file, *cloud_nl);//load normals from pcd
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_cl, cloud_nl, 5.0, 1.0, cl_filename+"_normals");
      }

      //show cluster name only on checkbox
      if (ui.checkBox_show_cluster_name->isChecked() )
      {
        //find centriod to anchor label/Text3D
        Eigen::Vector4f centroid_cluster; PointXYZ p;
        compute3DCentroid(*cloud_cl, centroid_cluster);
        p.x = centroid_cluster[0]; p.y = centroid_cluster[1]; p.z = centroid_cluster[2];
        cluster_centroids_.push_back(p);//so that no need to recompute
        viewer->addText3D("No." + std::to_string(i), p, 2, 1, 1, 0,"N"+std::to_string(i));//josh edited to add idname and small text
      }
    }
    else//(i==-1) load remaining clusters first so that it's priority is at the back
    {
      //load reject cluster and break loop
      std::string cl_filename = "cluster_rejected";//"+std::to_string(cluster_no);
      std::string curr_custer_file = in_data_file_dir_.toStdString()+"clusters/rejected_points.pcd";
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cl (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile <pcl::PointXYZ> (curr_custer_file, *cloud_cl);
      viewer->addPointCloud (cloud_cl, cl_filename);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, cl_filename);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cl_filename);
      std::cout << "rejected_points cluster has " << cloud_cl->points.size() << " points." << std::endl;

      //show normals only on checkbox
      if(ui.checkBox_show_normals->isChecked())
      {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_nl(new pcl::PointCloud<pcl::Normal>);
        pcl::io::loadPCDFile <pcl::Normal> (curr_custer_file, *cloud_nl);
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_cl, cloud_nl, 5.0, 1.0, cl_filename+"_normals");
      }
    }
    ui.progressBar->setValue(80+20*(i+1)/(cluster_no));
    ui.progressBar->update();
  }

  cluster_no_ctrl(cluster_no-1,'m');
  std::string text_clusters_no = "No. of Clusters = "+std::to_string(cluster_no);
  viewer->updateText(text_clusters_no, 10, 50, 1, 1, 1, "clusters"); //original y was 620
  ui.qvtkWidget->update ();
  log(Info, std::string("All Clusters Loaded!"));

}

void ATG_Window::remove_seg_labels()
{
  for (int i=0; i<1000; i++)
  {
    viewer->removePointCloud("cluster_"+std::to_string(i));
    viewer->removePointCloud("cluster_"+std::to_string(i)+"_normals");
    viewer->removeText3D("N"+std::to_string(i));//josh edited to remove all text3D
  }
  viewer->removePointCloud("cluster_rejected");
//  viewer->removeAllPointClouds(); //not using this as it deletes the text on screen
//  viewer->removeAllShapes();      //not using this as it deletes the text on screen
}

void ATG_Window::cluster_no_ctrl(int value, char action)
{

  if(action == 'e')//enable cluster ctrl
  {
    ui.frame_Cluster_Ctrl->setEnabled(true);
    ui.tab_draw->setEnabled(true);
  }
  else if (action == 'd')//disable cluster ctrl
  {
    ui.frame_Cluster_Ctrl->setEnabled(false);
    ui.tab_draw->setEnabled(false);
  }
  else if(action == 'm')//maximum change
  {
    ui.spinBox_IM_cluster_no->setMaximum(value);
    ui.spinBox_clusters_draw->setMaximum(value);
    ui.spinBox_clusters     ->setMaximum(value);
  }
  else //if(action == 'v')//value change
  {
    ui.spinBox_clusters_draw->setValue(value);
    ui.spinBox_clusters     ->setValue(value);
    ui.spinBox_toolpaths    ->setValue(value);
  }
}

void ATG_Window::pushButton_visualize_clusters_clicked()
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);using g_cloud_cluster_ instead
  std::string custer_no = std::to_string(ui.spinBox_clusters->value());
  std::string file_name = in_data_file_dir_.toStdString()+"clusters/cluster_"+custer_no+".pcd";
  std::string cluster_name = "clustersviz_"+custer_no;

  //std::string cl_filename = "cluster_"+std::to_string(cluster_no);
  //std::string curr_custer_file = in_data_file_dir_.toStdString()+"clusters/"+cl_filename+".pcd";

  //file_name = "data/coupons/clusters/cluster_"+to_string(clusters_no)+".pcd";

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *g_cloud_cluster_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read cluster .pcd file \n");
    exit(-1);
  }
  std::cout << "Cluster Point Cloud Data Loaded no: " << custer_no << std::endl;
  //cluster_name ="clustersviz_"+std::to_string(clusters_no);
  //log(Info, std::string("loading cluster point cloud data....."));

  //viewer->updateText(file_name, 10, 20, 1, 1, 1, "name");//original y was680//should not update text..
  viewer->addPointCloud (g_cloud_cluster_, cluster_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 1.0f, cluster_name);
  ui.qvtkWidget->update ();
  std::string log_string = "Point cloud data for cluster "+custer_no+" selected";
  log(Info,log_string);

    Eigen::Vector4f centroid;
    compute3DCentroid(*g_cloud_cluster_, centroid);
    PointXYZ p;
    p.x = centroid[0]; p.y = centroid[1]; p.z = centroid[2];
    viewer->removeText3D("N"+custer_no); //josh added
    viewer->addText3D("No."+custer_no, p, 3, 1, 1, 0,"N"+custer_no); //josh edited Bold text
}

void ATG_Window::pushButton_clear_clusters_clicked()
{
  //find all cloud names in visualizer starting with "clustersviz_"
  std::vector<int> cluster_remove;
  std::vector<pcl::PointXYZ> cluster_pt;
  for (auto iter = viewer->getCloudActorMap()->begin(); iter != viewer->getCloudActorMap()->end(); ++iter)
  {
    pcl::visualization::CloudActor &actor = iter->second;
    std::string cloud_name = iter->first.c_str ();
    //pcl::PointXYZ cloud_pt = iter->third;
    //std::cout << "Area Search - ActorName (" << iter->first.c_str () << ")" << std::endl;//actor.actor->GetProperty()->GetInformation()
    if(cloud_name.find("clustersviz_")!=std::string::npos)//found cluster viz
    {
      //std::cout << "substr " << cloud_name.substr(12,cloud_name.length()) <<std::endl;
      int clusterviz_no = stoi(cloud_name.substr(12,cloud_name.length()));
      cluster_remove.push_back(clusterviz_no);
    }
  }
  if (cluster_remove.size()>0)
  {
    for (int i=0;i<cluster_remove.size();i++)
    {
      viewer->removePointCloud ("clustersviz_"+std::to_string(cluster_remove[i]));
      viewer->removeText3D("N"+std::to_string(cluster_remove[i]));
      if (ui.checkBox_show_cluster_name->isChecked())
        viewer->addText3D("No."+std::to_string(cluster_remove[i]), cluster_centroids_[cluster_remove[i]], 2, 1, 1, 0,"N"+std::to_string(cluster_remove[i])); //josh edited Bold text
      ui.qvtkWidget->update ();
      std::cout << "Cluster Point Cloud Data Cleared no: " << cluster_remove[i] << std::endl;
      std::string log_string = "Point cloud data for cluster "+std::to_string(cluster_remove[i])+" de-selected";
      log(Info,log_string);
    }
  }
}

void ATG_Window::pushButton_load_backup_clusters_clicked ()
{
  //Step 1: Overwrite current cluster folder with backup folder
  //Step 2: Load all files in cluster folder & Show cluster no. if valid
  //Step 3: Show normal if valid
  std::string cmd = "rosrun atg_gui duplicate_files.py "+
                              ex_data_file_dir_.toStdString()+"backup/"+g_short_filename_.toStdString()+" "+  // /home/data/ + backup/ + filename
                              in_data_file_dir_.toStdString()+"clusters";                                     // data/coupons/ + clusters
  int response = system(cmd.c_str()); //load to /home/data (docker sharefolder) instead of own dir
  if (response>0)//response valid in sys.exit is int/256
  {
      std::cout << "Reponse: " <<response<<"\n";
      std::cout << "Load Backup Failed, please check folder!!\n\n";
      std::string log_string = "Fail to loaded clusters from "+ex_data_file_dir_.toStdString()+"backup/"+g_short_filename_.toStdString()+", pls check folder.";
      log(Info,log_string);
      return;
  }
  std::cout << "Removing the old cluster files....\n";
  std::cout << "Duplicating the backup cluster files....\n";
  std::string log_string = "Loaded clusters from "+ex_data_file_dir_.toStdString()+"backup/"+g_short_filename_.toStdString();
  log(Info,log_string);
  viewer->removeAllPointClouds();
  read_pcd_in_cluster_folder();
  //reset all variables and ui
  read_write_fsignal("segmentation_signal",'d');
  ui.tabWidget_file->setEnabled(true);
  ui.tabWidget_seg->setEnabled(true);
  ui.tabWidget_toolpath->setEnabled(true);
  cluster_no_ctrl(0, 'e');//enable cluster ctrl, only enable after segmentation completes
}

void ATG_Window::pushButton_backup_curr_clusters_clicked ()
{
  //Step 1: Overwrite current backup folder with cluster folder

  //python src/scripts/duplicate_files.py data/coupons/clusters /home/data/backup/<file_name>;
  std::string cmd = "rosrun atg_gui duplicate_files.py "+
                              in_data_file_dir_.toStdString()+"clusters "+                                // data/coupons/ + clusters
                              ex_data_file_dir_.toStdString()+"backup/"+g_short_filename_.toStdString();  // /home/data/ + backup/ + filename
  int response = system(cmd.c_str()); //load to /home/data (docker sharefolder) instead of own dir
  if (response>0) //response valid in sys.exit is int/256
  {
      std::cout << "Reponse: " <<response<<"\n";
      std::cout << "Save Clusters Fail!!\n\n";
      return;
  }
  std::cout << "Removing the old backup files....\n";
  std::cout << "Duplicating the cluster files into backup....\n";
  std::string log_string = "Saved clusters to "+ex_data_file_dir_.toStdString()+"backup/"+g_short_filename_.toStdString();
  log(Info,log_string);
}

void ATG_Window::pushButton_draw_append_clicked()   //josh added for interactive click
{
  draw_cluster(0,ui.spinBox_clusters_draw->value());
}
void ATG_Window::pushButton_draw_subtract_clicked() //josh added for interactive click
{
  draw_cluster(1,ui.spinBox_clusters_draw->value());
}
void ATG_Window::pushButton_draw_create_clicked()   //josh added for interactive click
{
  int max_plus_one = ui.spinBox_clusters_draw->maximum()+1;
  bool result = draw_cluster(2,max_plus_one);
  if (result)
  {
    clusters_no_min_max(0,max_plus_one);
    ui.spinBox_clusters->setValue(max_plus_one);
  }
}

void ATG_Window::clusters_no_min_max(int min, int max)
{
  ui.spinBox_clusters->     setMinimum(min);
  ui.spinBox_clusters_draw->setMinimum(min);
  ui.spinBox_toolpaths->    setMinimum(min);
  ui.spinBox_clusters->     setMaximum(max);
  ui.spinBox_clusters_draw->setMaximum(max);
  ui.spinBox_toolpaths->    setMaximum(max);
  ui.spinBox_IM_cluster_no->setMinimum(min);
  ui.spinBox_IM_cluster_no->setMaximum(max);
}

/******************************************
** SLOTS for robot settings tab
*******************************************/
//=========Setup tab===========
void ATG_Window::comboBox_tool_selection_clicked(QString value)
{
    //std::cout<< "tabWidget3 has tabs -> " << ui.tabWidget_3->count() << std::endl;
    int last = 3;//ui.tabWidget_3->count()-1;//using count will additionally remove tabs when toggling comboBox
    ui.tabWidget_robot_setup->removeTab(last); //3
    ui.tabWidget_robot_setup->removeTab(last); //4
    ui.tabWidget_robot_setup->removeTab(last); //5
    ui.tabWidget_robot_setup->removeTab(last); //6
    if(ui.comboBox_tool_selection->currentText()=="Nakanishi Tool")
    {
        ui.tabWidget_robot_setup->insertTab(last,ui.tab_spindle_tool,"Spindle Tool");
        std::cout << "Nakanishi Tool Selected" << std::endl;
    }
    else if(ui.comboBox_tool_selection->currentText()=="Masking Tool"
            || ui.comboBox_tool_selection->currentText()=="Masking Tool (old)")
    {
        ui.tabWidget_robot_setup->insertTab(last,ui.tab_masking_tool,"Masking Tool");
        std::cout << "Masking Tool Selected" << std::endl;
    }
    else if(ui.comboBox_tool_selection->currentText()=="Impedance Tool")
    {
      ui.tabWidget_robot_setup->insertTab(last,ui.tab_impedance_tool,"Impedance Tool");
      std::cout << "Impedance Tool Selected" << std::endl;
    }
    else if(ui.comboBox_tool_selection->currentText()=="Sandblasting Nozzle Tool")
    {
      ui.tabWidget_robot_setup->insertTab(last,ui.tab_sandblasting_tool,"Sandblasting Tool");
      std::cout << "Sandblasting Tool Selected" << std::endl;
    }
    else if(ui.comboBox_tool_selection->currentText()=="NDT Tool")
    {
      ui.tabWidget_robot_setup->insertTab(last,ui.tab_ndt_tool,"NDT Tool");
      std::cout << "NDT Tool Selected" << std::endl;
    }
    else
    {
        std::cout << ui.comboBox_tool_selection->currentText().toStdString() << " Selected" << std::endl;
    }
    readQSettings_TCP_only();
}

void ATG_Window::comboBox_env_selection_clicked(QString value)
{
  readQSettings_ENV_OBJ_only();
}

void ATG_Window::comboBox_robot_selection_clicked(QString value)
{
  readQSettings_Robot_IP_only();
  pushButton_refreshScript_clicked();
}

//=========Robot & Environment tab===========
Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) //local function
{
  Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

void ATG_Window::robot_TCP_changed(double value){
  //setup values for robot tcp
  double X=0,Y=0,Z=0,R_=0,P_=0,Y_=0,dia=0;
  X  = ui.doubleSpinBox_robot_tcp_X ->value();
  Y  = ui.doubleSpinBox_robot_tcp_Y ->value();
  Z  = ui.doubleSpinBox_robot_tcp_Z ->value();
  R_ = ui.doubleSpinBox_robot_tcp_R_->value()*M_PI/180;
  P_ = ui.doubleSpinBox_robot_tcp_P_->value()*M_PI/180;
  Y_ = ui.doubleSpinBox_robot_tcp_Y_->value()*M_PI/180;

  //additional calculation for different TCP modes (eg spindle for flap wheel, tilt angle)
    if(ui.comboBox_tool_selection->currentText()=="Nakanishi Tool")
    {
      //use spindle option values
      if(ui.radioButton_nkns_spindle_flat_disc_tilt->isChecked())
      {
        Z -= ui.doubleSpinBox_nkns_spindle_tool_dia->value()/2;//1200    //Y += ui.doubleSpinBox_polish_tool_dia->value()/2;//2400
        P_+= 90*M_PI/180;
        P_-= ui.doubleSpinBox_nkns_spindle_tilt_angle->value()*M_PI/180;//1200   //R_+= ui.doubleSpinBox_polish_tilt_angle->value()*M_PI/180;//2400

      }
      else if(ui.radioButton_nkns_spindle_flat_disc_perp->isChecked()) //flat_disc and flap wheel<--to be determined
      {
        P_+= 90*M_PI/180;
      }
      else if(ui.radioButton_nkns_spindle_flap_wheel->isChecked())
      {
          Z += ui.doubleSpinBox_nkns_spindle_tool_dia->value()/2;
          dia = ui.doubleSpinBox_nkns_spindle_tool_dia->value()/2;
      }
    }
    else if(ui.comboBox_tool_selection->currentText()=="Sandblasting Tool")
    {
      R_-= ui.doubleSpinBox_sandblasting_tilt_angle_x->value()*M_PI/180; //TODO: need to validate with RViz
      P_-= ui.doubleSpinBox_sandblasting_tilt_angle_y->value()*M_PI/180; //TODO: need to validate with RViz
    }
    else if(ui.comboBox_tool_selection->currentText()=="Masking Tool"
         || ui.comboBox_tool_selection->currentText()=="Masking Tool (old)"
         || ui.comboBox_tool_selection->currentText()=="Impedance Tool"
         || ui.comboBox_tool_selection->currentText()=="NDT Tool")
    {
      //use tcp values stated only
    }


  //Example for TCP_param: 'X Y Z R_ P_ Y_ dia eeX eeY eeZ eeR eeP eeY' in m and rad
  QString text1 = "";
  text1 += QString::number(X/1000  ,'f',5) + " ";
  text1 += QString::number(Y/1000  ,'f',5) + " ";
  text1 += QString::number(Z/1000  ,'f',5) + " ";
  text1 += QString::number(R_      ,'f',4) + " ";
  text1 += QString::number(P_      ,'f',4) + " ";
  text1 += QString::number(Y_      ,'f',4) + " ";
  text1 += QString::number(ui.doubleSpinBox_robot_eeX ->value()/1000    ,'f',5) + " ";
  text1 += QString::number(ui.doubleSpinBox_robot_eeY ->value()/1000    ,'f',5) + " ";
  text1 += QString::number(ui.doubleSpinBox_robot_eeZ ->value()/1000    ,'f',5) + " ";
  text1 += QString::number(ui.doubleSpinBox_robot_eeR_->value()*M_PI/180,'f',4) + " ";
  text1 += QString::number(ui.doubleSpinBox_robot_eeP_->value()*M_PI/180,'f',4) + " ";
  text1 += QString::number(ui.doubleSpinBox_robot_eeY_->value()*M_PI/180,'f',4) + " ";
  text1 += QString::number(dia/1000,'f',5) + " "; //normally 0 unless for flap wheel --- TODO: change format to back
  ui.label_robot_tcp_info_xyzrpy->setText(text1);

  //Example for UR: p[-0.11324,0.0,0.11216,0.0,-0.757,0.0]
  text1 = "p[";
  text1 += QString::number(X/1000,'f',5) + ","; //x in [m]
  text1 += QString::number(Y/1000,'f',5) + ","; //y in [m]
  text1 += QString::number(Z/1000,'f',5) + ","; //z in [m]
  text1 += QString::number(R_    ,'f',5) + ","; //r in [rad]
  text1 += QString::number(P_    ,'f',5) + ","; //p in [rad]
  text1 += QString::number(Y_    ,'f',5) + "]"; //y in [rad]
  ui.label_robot_tcp_info_xyzrpy_UR->setText(text1); //for UR

  //Eample for ABB: [170.939,0.478131,98.3167],[0.707717,-0.00180394,0.706491,0.00180086]
  //calculate quaternions from rpy
  Eigen::Affine3d tmp_rot = create_rotation_matrix(R_,P_,Y_);
  Eigen::Matrix3d rot = tmp_rot.rotation().matrix();  //convert into matrix
  //Eigen::Matrix3d rot_mat = rot;                      //convert into 3x3 matrix
  Eigen::Quaterniond quat(rot);                       //convert into quat
  text1 = "[";
  text1 += QString::number(X       ,'f',2) + ",";  //x in [mm]
  text1 += QString::number(Y       ,'f',2) + ",";  //x in [mm]
  text1 += QString::number(Z       ,'f',2) + "],[";//x in [mm]
  text1 += QString::number(quat.w(),'f',8) + ",";  //w in quaternion
  text1 += QString::number(quat.x(),'f',8) + ",";  //x in quaternion
  text1 += QString::number(quat.y(),'f',8) + ",";  //y in quaternion
  text1 += QString::number(quat.z(),'f',8) + "]";  //z in quaternion
  ui.label_robot_tcp_info_xyzquat_ABB->setText(text1); //for UR
}

//=========Work Object tab===========
void ATG_Window::flip_surface_normal(){
  std::string cmd;
  //Josh doing point cloud conversion using meshlabserver function to flip PLY normals
  std::string obj_filename_mini = ex_data_file_dir_.toStdString()+"cache/pclmini.ply";// "/home/data/cache/pclmini.ply"
  std::string stl_filename_mini = ex_data_file_dir_.toStdString()+"cache/pclmini.ply";// "/home/data/cache/pclmini.ply"
  std::string mlx_filename_mini = "src/atg_gui_module/script/flip_mesh_normal.mlx"; //TODO: change structure to source for atg_gui package file

  cmd="xterm -hold -e 'meshlabserver -i " + obj_filename_mini+" -o "+
                            stl_filename_mini+" -s "+
                            mlx_filename_mini+" -om vc vf vn ff' &";
  system(cmd.c_str());
  cmd = "sleep 1; xdotool search --class xterm set_desktop_for_window %@ 1";
  system(cmd.c_str());
  std::cout<<"Flipped Mesh data normal for Rviz visualization!"<< std::endl;
  log(Info,std::string("Flipped Mesh data normal for RViz visualization!"));

}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_svd1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_svd2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_svd3 (new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f svd_mat = Eigen::Matrix4f::Identity ();
void ATG_Window::pushButton_viz_svd_clicked     ()
{
  //viz ICP from UI
  QString txtdata1 = ui.textEdit_svd1->toPlainText();
  QString txtdata2 = ui.textEdit_svd2->toPlainText();
  if (txtdata1.length()+txtdata2.length()<1)
  {
    //add some text
    ui.textEdit_svd1->setText("0;0;20;\n120;0;20;\n0;200;20;\n130;300;50;\n130;0;50;\n0;300;50;");
    ui.textEdit_svd2->setText("0.5;0.6;30.5;\n0.2;-120.5;30;\n200.3;0.5;30.1;\n300.4;-130.2;60.2;\n0.4;-130.23;60.4;\n300.1;0.5;60.3;");
    /* in
    0;0;20;\n120;0;20;\n0;200;20;\n130;300;50;\n130;0;50;\n0;300;50;

    tar
    0.5;0.6;30.5;\n0.2;-120.5;30;\n200.3;0.5;30.1;\n300.4;-130.2;60.2;\n0.4;-130.23;60.4;\n300.1;0.5;60.3;
    */
  }
  else
  {
    //read for valid text to point cloud
    cloud_svd1->clear();
    cloud_svd2->clear();
    pcl::PointXYZ point;
    QStringList strList1 = txtdata1.split(QRegExp("[\n]"),QString::SkipEmptyParts);
    QStringList strList2 = txtdata2.split(QRegExp("[\n]"),QString::SkipEmptyParts);

    for(int i = 0;i<strList1.length();i++)
    {
      QStringList strListPt = strList1[i].split(QRegExp("[;]"),QString::SkipEmptyParts);
      if(strListPt.length()==3)
      {
        point.x = strListPt[0].toFloat();
        point.y = strListPt[1].toFloat();
        point.z = strListPt[2].toFloat();
        cloud_svd1->push_back(point);
      }
    }
    if (cloud_svd1->points.size()>0)
    {
      viewer->removePointCloud ("cloud_svd_input");
      viewer->addPointCloud (cloud_svd1, "cloud_svd_input");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "cloud_svd_input");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_svd_input");
    }

    for(int i = 0;i<strList2.length();i++)
    {
      QStringList strListPt = strList2[i].split(QRegExp("[;]"),QString::SkipEmptyParts);
      if(strListPt.length()==3)
      {
        point.x = strListPt[0].toFloat();
        point.y = strListPt[1].toFloat();
        point.z = strListPt[2].toFloat();
        cloud_svd2->push_back(point);
      }
    }
    if (cloud_svd2->points.size()>0)
    {
      viewer->removePointCloud ("cloud_svd_target");
      viewer->addPointCloud (cloud_svd2, "cloud_svd_target");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "cloud_svd_target");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_svd_target");
    }
    ui.qvtkWidget->update();
  }
}
void ATG_Window::pushButton_cal_svd_clicked     ()
{
  pushButton_viz_svd_clicked();
  //do calculate and viz ICP from UI
  if((cloud_svd1->points.size()>0)&&(cloud_svd2->points.size()>0))
  {
    cloud_svd3->clear();
    // Set the input source and target
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    //=====1. calculate transformation between input and target, input X transform = output
    cloud_in = cloud_svd1;
    cloud_tgt = cloud_svd2;
    pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ,pcl::PointXYZ> TESVD;
    pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation2;
    TESVD.estimateRigidTransformation (*cloud_in,*cloud_tgt,transformation2);
    pcl::transformPointCloud (*cloud_in, *cloud_out, transformation2);

    std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
    std::cout << transformation2.matrix() << std::endl;

    //=====2. visualisation
    if (cloud_out->points.size()>0)//(cloud_icp3->points.size()>0)
    {
//      viewer->removePointCloud ("cloud_icp3");
//      viewer->addPointCloud (cloud_icp3, "cloud_icp3");
//      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "cloud_icp3");
//      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp3");
//      pcl::transformPointCloud (*cloud_icp1, *cloud_icp3, Ti);
      viewer->removePointCloud ("cloud_svd_output");
      viewer->addPointCloud (cloud_out, "cloud_svd_output");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "cloud_svd_output");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_svd_output");
      svd_mat = transformation2;
    }
    ui.qvtkWidget->update();

    //outputs
    if(1)
    {
      //=====3. find error rate
      QString error_rate = "Output-Error-Rate: ";
      for(int i = 0; i < cloud_tgt->points.size(); i++)
      {
        double dist = pcl::geometry::squaredDistance(cloud_tgt->points[i],cloud_out->points[i]);
        double dist_ori = pcl::geometry::squaredDistance(cloud_tgt->points[i],cloud_in->points[i]);
        std::cout << "Point[" << i << "] sqdist = " << dist << "\tvs " << dist_ori << std::endl;
        error_rate += QString::number(dist,'f',2) + " ";
      }
      ui.label_svd_error_rates->setText(error_rate);

      //=====4. convert transform into quaternion to calculate for RPY
      Eigen::Matrix4f svd_mat = Eigen::Matrix4f::Identity ();
      Eigen::Quaternionf ret_quat;
      //===== 3. convert rotation in matrix to quaternion for return
      if (cloud_out->points.size()>0)
      {
        svd_mat = transformation2;
        Eigen::Quaternionf q(svd_mat.topLeftCorner<3,3>());
        ret_quat = q;
      }


      //===== 5. convert to rotation vector for return
      //convert quaternion to rpy (UR using zyx)
      //Method 1: using eularAngles zyx
      std::cout<<"Method 1: eular zyx \n";
      Eigen::Vector3f euler_t1 = ret_quat.toRotationMatrix().eulerAngles(2,1,0);             //works but eular angles might flip by 2x 180 around 2 axis which yield same result
      Eigen::AngleAxisf Y1(euler_t1(0), Eigen::Vector3f::UnitZ());
      Eigen::AngleAxisf P1(euler_t1(1), Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf R1(euler_t1(2), Eigen::Vector3f::UnitX());
      Eigen::Quaternionf quat_f1(Y1*P1*R1);//for checking only
      std::cout <<"rot_vec rx,ry,rz: "<<euler_t1[2]<<","<<euler_t1[1]<<","<<euler_t1[0]<<","<< std::endl;           //works but eular angles might flip by 2x 180 around 2 axis which yield same result
      std::cout <<"rot_vec rx,ry,rz: "<<euler_t1[2]*180/M_PI<<","<<euler_t1[1]*180/M_PI<<","<<euler_t1[0]*180/M_PI<<","<< std::endl;  //works but eular angles might flip by 2x 180 around 2 axis which yield same result
      std::cout <<"ret_quat x,y,z,w: " << ret_quat.x() << ", " << ret_quat.y() << ", " << ret_quat.z() << ", " << ret_quat.w() << std::endl;
      std::cout <<"quat_f1  x,y,z,w: " << quat_f1.x() << ", " << quat_f1.y() << ", " << quat_f1.z() << ", " << quat_f1.w() << std::endl<< std::endl;
      //return var from Method 1 euler zyx
      //ret_quat=quat_f1;
  //    ret_rot_vec(0)=euler_t1[2];//R
  //    ret_rot_vec(1)=euler_t1[1];//P
  //    ret_rot_vec(2)=euler_t1[0];//Y
  //    if(1)//convert to deg
  //    {
  //        ret_rot_vec= ret_rot_vec*180/M_PI;
  //    }

      //Output into string
      QString XYZ_ = "XYZ: "+QString::number(svd_mat(0,3),'f',2)+" "+QString::number(svd_mat(1,3),'f',2)+" "+QString::number(svd_mat(2,3),'f',2)+" ";
      QString RPY_ = "RPY: "+QString::number(euler_t1[2]*180/M_PI,'f',2) + " "+QString::number(euler_t1[1]*180/M_PI,'f',2) + " "+QString::number(euler_t1[0]*180/M_PI,'f',2) + " ";
      ui.label_svd_XYZ->setText(XYZ_);
      ui.label_svd_RPY->setText(RPY_);
    }

  }
}
void ATG_Window::pushButton_close_svd_clicked   ()
{
  //close ICP cloud from UI
  viewer->removePointCloud ("cloud_svd_target");
  viewer->removePointCloud ("cloud_svd_input");
  viewer->removePointCloud ("cloud_svd_output");
  ui.qvtkWidget->update();
}
void ATG_Window::pushButton_transfer_svd_clicked()
{
  std::string str_xyz = ui.label_svd_XYZ->text().toStdString();
  std::string str_rpy = ui.label_svd_RPY->text().toStdString();
  std::string str_err = ui.label_svd_error_rates->text().toStdString();
  std::stringstream lineStream(str_xyz+str_rpy+str_err);
  std::string cell;
  Eigen::VectorXd xyzrpy(6);
  int i = 0;bool invalid_errors = 0;
  try
  {
    while (std::getline(lineStream, cell, ' '))
    {
      //if (i==0)//only text
      if (i==1) //check X
      {
        xyzrpy(0) = std::stod(cell);
        std::cout << "X = " << cell;//<< std::endl;//josh debug
      }
      if (i==2)//check Y
      {
        xyzrpy(1) = std::stod(cell);
        std::cout << " Y = " << cell;//<< std::endl;//josh debug
      }
      if (i==3)//check Z
      {
        xyzrpy(2) = std::stod(cell);
        std::cout << " Z = " << cell;//<< std::endl;//josh debug
      }
      //if (i==4)//only text
      if (i==5)//check R_
      {
        xyzrpy(3) = std::stod(cell);
        std::cout << " R = " << cell;//<< std::endl;//josh debug
      }
      if (i==6)//check P_
      {
        xyzrpy(4) = std::stod(cell);
        std::cout << " P = " << cell;//<< std::endl;//josh debug
      }
      if (i==7)//check Y_
      {
        xyzrpy(5) = std::stod(cell);
        std::cout << " Y = " << cell << std::endl;//<< std::endl;//josh debug
      }
      //if (i==8)//only text
      if (i==9)//check valid Pt1 error rate
      {
        double er = std::stod(cell);
        std::cout << " Pt1_error = " << er;//<< std::endl;//josh debug
        if (er>100) invalid_errors = 1;
      }
      if (i==10)//check valid Pt2 error rate
      {
        double er = std::stod(cell);
        std::cout << " Pt2_error = " << er;//<< std::endl;//josh debug
        if (er>100) invalid_errors = 1;
      }
      if (i==11)//check valid Pt3 error rate
      {
        double er = std::stod(cell);
        std::cout << " Pt3_error = " << er << std::endl;//<< std::endl;//josh debug
        if (er>100) invalid_errors = 1;
      }
      i++;
    }
  }
  catch(...)
  {
    //do nothing
  }


  if(i>11 && invalid_errors==0)
  {
    std::cout << "Valid Transformation Matrix, sending to obj frame." << std::endl;//<< std::endl;//josh debug
    ui.doubleSpinBox_object_X->setValue(xyzrpy(0));
    ui.doubleSpinBox_object_Y->setValue(xyzrpy(1));
    ui.doubleSpinBox_object_Z->setValue(xyzrpy(2));
    ui.doubleSpinBox_object_R_->setValue(xyzrpy(3));
    ui.doubleSpinBox_object_P_->setValue(xyzrpy(4));
    ui.doubleSpinBox_object_Y_->setValue(xyzrpy(5));
    log(Info,std::string("Transfered Transformation Matrix to Object Frame!"));
  }
  else
  {
    std::cout << "Invalid Transformation Matrix, not sending to obj frame." << std::endl;//<< std::endl;//josh debu
    log(Info,std::string("Invalid Transformation Matrix, error rate too high!"));
  }
}

//=========Nakanishi Spindle tab===========
void ATG_Window::radioButton_nkns_spindle_clicked ()
{
    if(ui.radioButton_nkns_spindle_flat_disc_perp->isChecked())
    {
        ui.doubleSpinBox_nkns_spindle_tilt_angle->setEnabled(false);
    }
    if(ui.radioButton_nkns_spindle_flat_disc_tilt->isChecked())
    {
        ui.doubleSpinBox_nkns_spindle_tilt_angle->setEnabled(true);
    }
    if(ui.radioButton_nkns_spindle_flap_wheel->isChecked())
    {
        ui.doubleSpinBox_nkns_spindle_tilt_angle->setEnabled(false);
    }
    //polishTCPChanged(0); //update tcp as well (superseded)
    robot_TCP_changed(0); //update tcp as well
}
double previous_zone_nkns = 0;
void ATG_Window::doubleSpinBox_nsks_spindle_zone_radius_changed(double value)
{
  if (ui.comboBox_robot_selection->currentText()=="ABB IRB 1200" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2400" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2600" )
  {
    //use zone values from ABB
    if (previous_zone_nkns<value)//value going up
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>1 && value<=5) value = 5;
      else if (value>5 && value<=10) value = 10;
      else if (value>10 && value<=15) value = 15;
      else if (value>15 && value<=20) value = 20;
      else if (value>20 && value<=30) value = 30;
      else if (value>30 && value<=40) value = 40;
      else if (value>40 && value<=50) value = 50;
      else if (value>50 && value<=60) value = 60;
      else if (value>60 && value<=80) value = 80;
      else if (value>80 && value<=100) value = 100;
      else if (value>100 && value<=150) value = 150;
      else if (value>150 && value<=200) value = 200;
      ui.doubleSpinBox_nkns_spindle_zone_radius->setValue(value);
    }
    else // value going down
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>=1   && value<5) value = 1;
      else if (value>=5   && value<10) value = 5;
      else if (value>=10  && value<15) value = 10;
      else if (value>=15  && value<20) value = 15;
      else if (value>=20  && value<30) value = 20;
      else if (value>=30  && value<40) value = 30;
      else if (value>=40  && value<50) value = 40;
      else if (value>=50  && value<60) value = 50;
      else if (value>=60  && value<80) value = 60;
      else if (value>=80  && value<100) value = 80;
      else if (value>=100 && value<150) value = 100;
      else if (value>=150 && value<200) value = 150;
      ui.doubleSpinBox_nkns_spindle_zone_radius->setValue(value);
    }
  }
  else //not ABB in comboBox_robot_selection
  {
    if (value<0.5) ui.doubleSpinBox_nkns_spindle_zone_radius->setValue(0);
    else if (value>=0.5 && value<=1) ui.doubleSpinBox_nkns_spindle_zone_radius->setValue(1);
  }
  previous_zone_nkns = value;
}

//=========Masking Tool tab===========
double previous_zone_mask = 0;
void ATG_Window::doubleSpinBox_masking_blend_radius_changed(double value)
{
  if (ui.comboBox_robot_selection->currentText()=="ABB IRB 1200" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2400" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2600" )
  {
    //use zone values from ABB
    if (previous_zone_mask<value)//value going up
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>1 && value<=5) value = 5;
      else if (value>5 && value<=10) value = 10;
      else if (value>10 && value<=15) value = 15;
      else if (value>15 && value<=20) value = 20;
      else if (value>20 && value<=30) value = 30;
      else if (value>30 && value<=40) value = 40;
      else if (value>40 && value<=50) value = 50;
      else if (value>50 && value<=60) value = 60;
      else if (value>60 && value<=80) value = 80;
      else if (value>80 && value<=100) value = 100;
      else if (value>100 && value<=150) value = 150;
      else if (value>150 && value<=200) value = 200;
      ui.doubleSpinBox_masking_blend_radius->setValue(value);
    }
    else // value going down
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>=1   && value<5) value = 1;
      else if (value>=5   && value<10) value = 5;
      else if (value>=10  && value<15) value = 10;
      else if (value>=15  && value<20) value = 15;
      else if (value>=20  && value<30) value = 20;
      else if (value>=30  && value<40) value = 30;
      else if (value>=40  && value<50) value = 40;
      else if (value>=50  && value<60) value = 50;
      else if (value>=60  && value<80) value = 60;
      else if (value>=80  && value<100) value = 80;
      else if (value>=100 && value<150) value = 100;
      else if (value>=150 && value<200) value = 150;
      ui.doubleSpinBox_masking_blend_radius->setValue(value);
    }
  }
  else //not ABB in comboBox_robot_selection
  {
    if (value<0.5) ui.doubleSpinBox_masking_blend_radius->setValue(0);
    else if (value>=0.5 && value<=1) ui.doubleSpinBox_masking_blend_radius->setValue(1);
  }
  previous_zone_mask=value;
}

//=========Impedance Tool tab===========
double previous_zone_imp = 0;
void ATG_Window::doubleSpinBox_impedance_blend_radius_changed(double value)
{
  if (ui.comboBox_robot_selection->currentText()=="ABB IRB 1200" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2400" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2600" )
  {
    //use zone values from ABB
    if (previous_zone_imp<value)//value going up
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>1 && value<=5) value = 5;
      else if (value>5 && value<=10) value = 10;
      else if (value>10 && value<=15) value = 15;
      else if (value>15 && value<=20) value = 20;
      else if (value>20 && value<=30) value = 30;
      else if (value>30 && value<=40) value = 40;
      else if (value>40 && value<=50) value = 50;
      else if (value>50 && value<=60) value = 60;
      else if (value>60 && value<=80) value = 80;
      else if (value>80 && value<=100) value = 100;
      else if (value>100 && value<=150) value = 150;
      else if (value>150 && value<=200) value = 200;
      ui.doubleSpinBox_impedance_blend_radius->setValue(value);
    }
    else // value going down
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>=1   && value<5) value = 1;
      else if (value>=5   && value<10) value = 5;
      else if (value>=10  && value<15) value = 10;
      else if (value>=15  && value<20) value = 15;
      else if (value>=20  && value<30) value = 20;
      else if (value>=30  && value<40) value = 30;
      else if (value>=40  && value<50) value = 40;
      else if (value>=50  && value<60) value = 50;
      else if (value>=60  && value<80) value = 60;
      else if (value>=80  && value<100) value = 80;
      else if (value>=100 && value<150) value = 100;
      else if (value>=150 && value<200) value = 150;
      ui.doubleSpinBox_impedance_blend_radius->setValue(value);
    }
  }
  else //not ABB in comboBox_robot_selection
  {
    if (value<0.5) ui.doubleSpinBox_impedance_blend_radius->setValue(0);
    else if (value>=0.5 && value<=1) ui.doubleSpinBox_impedance_blend_radius->setValue(1);
  }
  previous_zone_imp=value;
}


//=========Sandblasting Tool tab===========
double previous_zone_sdbl = 0;
void ATG_Window::doubleSpinBox_sandblasting_zone_radius_changed(double value)
{
  if (ui.comboBox_robot_selection->currentText()=="ABB IRB 1200" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2400" ||
      ui.comboBox_robot_selection->currentText()=="ABB IRB 2600" )
  {
    //use zone values from ABB
    if (previous_zone_sdbl<value)//value going up
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>1 && value<=5) value = 5;
      else if (value>5 && value<=10) value = 10;
      else if (value>10 && value<=15) value = 15;
      else if (value>15 && value<=20) value = 20;
      else if (value>20 && value<=30) value = 30;
      else if (value>30 && value<=40) value = 40;
      else if (value>40 && value<=50) value = 50;
      else if (value>50 && value<=60) value = 60;
      else if (value>60 && value<=80) value = 80;
      else if (value>80 && value<=100) value = 100;
      else if (value>100 && value<=150) value = 150;
      else if (value>150 && value<=200) value = 200;
      ui.doubleSpinBox_sandblasting_zone_radius->setValue(value);
    }
    else // value going down
    {
      if (value==0) 1;
      else if (value==1) 1;
      else if (value>=1   && value<5) value = 1;
      else if (value>=5   && value<10) value = 5;
      else if (value>=10  && value<15) value = 10;
      else if (value>=15  && value<20) value = 15;
      else if (value>=20  && value<30) value = 20;
      else if (value>=30  && value<40) value = 30;
      else if (value>=40  && value<50) value = 40;
      else if (value>=50  && value<60) value = 50;
      else if (value>=60  && value<80) value = 60;
      else if (value>=80  && value<100) value = 80;
      else if (value>=100 && value<150) value = 100;
      else if (value>=150 && value<200) value = 150;
      ui.doubleSpinBox_sandblasting_zone_radius->setValue(value);
    }
  }
  else //not ABB in comboBox_robot_selection
  {
    if (value<0.5) ui.doubleSpinBox_sandblasting_zone_radius->setValue(0);
    else if (value>=0.5 && value<=1) ui.doubleSpinBox_sandblasting_zone_radius->setValue(1);
  }
  previous_zone_sdbl=value;
}


/******************************************
** SLOTS for toolpath tab
*******************************************/
void ATG_Window::comboBox_toolpath_selection_textChanged(QString Text_comboBox)
{
    //default set all not visible
    ui.label_toolpath_resolution  ->setVisible(false);
    ui.spinBox_toolpath_resolution->setVisible(false);
    ui.checkbox_toolpath_resolution_forced->setVisible(false);
    ui.label_toolpath_hole_patch          ->setVisible(false);
    ui.spinBox_toolpath_hole_patch        ->setVisible(false);
    ui.label_toolpath_step_size   ->setVisible(false);
    ui.spinBox_toolpath_step_size ->setVisible(false);
    ui.label_toolpath_offset   ->setVisible(false);
    ui.spinBox_toolpath_offset ->setVisible(false);
    ui.label_toolpath_downsample  ->setVisible(false);
    ui.spinBox_toolpath_downsample->setVisible(false);
    ui.label_Path_Rotate  ->setVisible(false);
    ui.spinBox_Path_Rotate->setVisible(false);
    ui.label_KSearch_TP_Surface  ->setVisible(false);
    ui.spinBox_KSearch_TP_Surface->setVisible(false);
    ui.label_path_section_range      ->setVisible(false);
    ui.spinBox_path_section_range_min->setVisible(false);
    ui.spinBox_path_section_range_max->setVisible(false);
    ui.checkbox_reverse_toolpath     ->setVisible(false);

    if(Text_comboBox == "Boundary")
    {
      ui.label_toolpath_resolution  ->setVisible(true);
      ui.spinBox_toolpath_resolution->setVisible(true);
      ui.checkbox_toolpath_resolution_forced->setVisible(true);
      ui.label_toolpath_hole_patch          ->setVisible(true);
      ui.spinBox_toolpath_hole_patch        ->setVisible(true);
      ui.label_toolpath_offset   ->setVisible(true);
      ui.spinBox_toolpath_offset ->setVisible(true);
      ui.label_toolpath_downsample  ->setVisible(true);
      ui.spinBox_toolpath_downsample->setVisible(true);
      ui.label_Path_Rotate  ->setVisible(true);
      ui.spinBox_Path_Rotate->setVisible(true);
      ui.label_KSearch_TP_Surface  ->setVisible(true);
      ui.spinBox_KSearch_TP_Surface->setVisible(true);
      ui.label_path_section_range      ->setVisible(true);
      ui.spinBox_path_section_range_min->setVisible(true);
      ui.spinBox_path_section_range_max->setVisible(true);
      ui.checkbox_reverse_toolpath     ->setVisible(true);
    }
    else if(Text_comboBox == "Zig Zag")
    {
      ui.label_toolpath_resolution  ->setVisible(true);
      ui.spinBox_toolpath_resolution->setVisible(true);
      ui.checkbox_toolpath_resolution_forced->setVisible(true);
      ui.label_toolpath_hole_patch          ->setVisible(true);
      ui.spinBox_toolpath_hole_patch        ->setVisible(true);
      ui.label_toolpath_step_size   ->setVisible(true);
      ui.spinBox_toolpath_step_size ->setVisible(true);
      ui.label_toolpath_offset   ->setVisible(true);
      ui.spinBox_toolpath_offset ->setVisible(true);
      ui.label_toolpath_downsample  ->setVisible(true);
      ui.spinBox_toolpath_downsample->setVisible(true);
      ui.label_Path_Rotate  ->setVisible(true);
      ui.spinBox_Path_Rotate->setVisible(true);
      ui.label_KSearch_TP_Surface  ->setVisible(true);
      ui.spinBox_KSearch_TP_Surface->setVisible(true);
      ui.checkbox_reverse_toolpath     ->setVisible(true);
    }
    else
    {
      ui.label_toolpath_resolution  ->setVisible(true);
      ui.spinBox_toolpath_resolution->setVisible(true);
      ui.label_Path_Rotate  ->setVisible(true);
      ui.spinBox_Path_Rotate->setVisible(true);
      ui.label_KSearch_TP_Surface  ->setVisible(true);
      ui.spinBox_KSearch_TP_Surface->setVisible(true);
    }
}

void ATG_Window::pushButton_plot_toolpath_clicked()
{
  //start segmentation node from atg_surace_identification module
  //reset signal and disable relavant buttons and tabs
  read_write_fsignal("toolpath_signal",'w',"0");
  ui.tabWidget_toolpath->setEnabled(false);  //tmp disable for troubleshooting
  //WriteQSettings();//save settings to QSettings
  int     External_on       = ui.checkBox_Select_External->isChecked();
  int     Internal_on       = ui.checkBox_Select_Internal->isChecked();
  float   Resolution        = ui.spinBox_toolpath_resolution->value();
  float   Step_Size         = ui.spinBox_toolpath_step_size->value();
  float   Offset            = ui.spinBox_toolpath_offset->value();
  float   Path_Rotate       = ui.spinBox_Path_Rotate->value();
  int     Downsample        = ui.spinBox_toolpath_downsample->value();
  int     ksearch_tp        = ui.spinBox_KSearch_TP_Surface->value();
  int     Normal_Flip       = ui.checkBox_normal_flip->isChecked();
  float   section_range_min = ui.spinBox_path_section_range_min->value();
  float   section_range_max = ui.spinBox_path_section_range_max->value();
  int     reverse_toolpath  = ui.checkbox_reverse_toolpath->isChecked();
  int     Forced_Resolution = ui.checkbox_toolpath_resolution_forced->isChecked();
  float   Hole_Patch_Size   = ui.spinBox_toolpath_hole_patch ->value();
  float   Lift_Height       = 0;
  //Eigen::Vector4f centroid = coupon.centroid;
  if(ui.comboBox_tool_selection->currentText()=="Nakanishi Tool")
  {
    Lift_Height = ui.doubleSpinBox_nkns_spindle_lift_height->value();
  }
  else if(ui.comboBox_tool_selection->currentText()=="Masking Tool"
          || ui.comboBox_tool_selection->currentText()=="Masking Tool (old)")
  {
    Lift_Height = ui.doubleSpinBox_masking_lift_height->value();
  }
  else if(ui.comboBox_tool_selection->currentText()=="Impedance Tool")
  {
    Lift_Height = ui.doubleSpinBox_impedance_lift_height->value();
  }
  else if(ui.comboBox_tool_selection->currentText()=="Sandblasting Nozzle Tool")
  {
    Lift_Height = ui.doubleSpinBox_sandblasting_lift_height->value();
  }
  else if(ui.comboBox_tool_selection->currentText()=="NDT Tool")
  {
    Lift_Height = ui.doubleSpinBox_ndt_lift_height->value();
  }
  else
  {
    //invalid Lift_Height
  }

  std::string custer_no = std::to_string(ui.spinBox_clusters->value());
  std::string file_name = in_data_file_dir_.toStdString()+"clusters/cluster_"+custer_no+".pcd";
  std::string cmd = file_name + " ";
  cmd += std::to_string(External_on      )+" ";
  cmd += std::to_string(Internal_on      )+" ";
  cmd += std::to_string(Resolution       )+" ";
  cmd += std::to_string(Step_Size        )+" ";
  cmd += std::to_string(Offset           )+" ";
  cmd += std::to_string(Path_Rotate      )+" ";
  cmd += std::to_string(Downsample       )+" ";
  cmd += std::to_string(ksearch_tp       )+" ";
  cmd += std::to_string(Normal_Flip      )+" ";
  cmd += std::to_string(section_range_min)+" ";
  cmd += std::to_string(section_range_max)+" ";
  cmd += std::to_string(reverse_toolpath )+" ";
  cmd += std::to_string(Forced_Resolution)+" ";
  cmd += std::to_string(Hole_Patch_Size  )+" ";
  cmd += std::to_string(Lift_Height      );

  ui.progressBar->setValue(5);
  std::string toolpath_text = ui.comboBox_toolpath_selection->currentText().toStdString();
  log(Info,std::string("Generating "+toolpath_text));
  if (ui.comboBox_toolpath_selection->currentText()=="Boundary")
    cmd = "xterm -iconic -hold -e 'rosrun atg_toolpath_generation atg_toolpath_contour " + cmd +"' &";//-hold to hold, +hold to close
  else if (ui.comboBox_toolpath_selection->currentText()=="Zig Zag")
    cmd = "xterm -iconic -hold -e 'rosrun atg_toolpath_generation atg_toolpath_zigzag " + cmd +"' &";//-hold to hold, +hold to close
  else if (ui.comboBox_toolpath_selection->currentText()=="Point")
    cmd = "xterm -iconic -hold -e 'rosrun atg_toolpath_generation Point " + cmd +"' &";//-hold to hold, +hold to close

  else
  {
    std::cout << "Invalid toolpath selection\n";
    return;
  }
  system(cmd.c_str());
  cmd = "sleep 1; xdotool search --class xterm set_desktop_for_window %@ 1";
  system(cmd.c_str());

  //run timer to launch and track segmentation node
  QTimer *tpg_timer = new QTimer(this);
  connect (tpg_timer, SIGNAL(timeout()), this, SLOT(toolpath_thread()));   //for segmentation timer
  tpg_timer->start(200); //time specified in ms       //for segmentation timer
  g_tpg_timer_ = tpg_timer;
}

void ATG_Window::toolpath_thread()
{
  //subsequent entry, track segmentation node progress through signal
  g_tpg_timer_->setInterval(200);
  //check signal
  std::vector<std::string> lines;float current_fsignal;
  lines=read_write_fsignal("toolpath_signal",'r');
//  if(lines[0]=="no msg") return;//continue to next interval
//  else
//  {
    //error msg
    if(lines[0] == "error msg")
    {
      g_tpg_timer_->stop();
      ui.progressBar->setValue(0);
      ui.progressBar->update();
      QMessageBox::information(NULL, tr("Oops"), tr(lines[1].c_str()));
      //prompt error and enable buttons
    }
    else if(lines[0] == "percent") //valid float signal
    {
      current_fsignal=std::stod(lines[1]);
      //check is signal is stuck on number
      if(last_fsignal==current_fsignal && current_fsignal<50 && fake_fsignal<50) fake_fsignal+=0.2; //fake_fsignal is proxi for 00-50% where toolpath py is processing
      if(current_fsignal>=50) fake_fsignal=0;
      last_fsignal = current_fsignal;

      //output progress
      ui.progressBar->setValue(current_fsignal+fake_fsignal);
      ui.progressBar->update();
      if (!(current_fsignal>=100))
        return; //update progress and go to next loop, not continuing to stop thread
    }
    else// no msg or invalid msg, continue to next interval
    {
      return;
    }

//  }
  //progress up till here means fsignal is >= 100 or there's an error in toolpath node, stop thread
//  if (current_fsignal>=100)
//  {
    //stop signal to prevent repeat of timer running seg_thread() again
    g_tpg_timer_->stop();

    //reset all variables and ui
    fake_fsignal=0;
    read_write_fsignal("toolpath_signal",'d');
    ui.tabWidget_toolpath->setEnabled(true);
//  }
}

void ATG_Window::pushButton_draw_simple_toolpath_clicked(){
  //show the resulsts: toolpath + point cloud data of the target featrues + normals of the toolpath
  pushButton_clear_simple_toolpath_clicked();
  //check if cloud is loaded, return if not no cluster/toolpath available
  pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_sel_norm (new pcl::PointCloud<pcl::Normal>);

  std::string filename = "data/coupons/tool_path_back_projected_w_lift.pcd";
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *g_cloud_toolpath_) == -1)
  {
    std::cout << "Failed to read cluster file from: " << filename << std::endl;
    exit(1);
  }
  if ( pcl::io::loadPCDFile <pcl::Normal> (filename, *cloud_cluster_sel_norm) == -1)
  {
    std::cout << "Failed to read cluster file from: " << filename << std::endl;
    exit(1);
  }


  if (g_cloud_toolpath_->size()<1 || g_cloud_cluster_->size()<1)
  {
    std::cout<<"cluster_target_surface_size="<<g_cloud_cluster_->size()<<std::endl;
    std::cout<<"cluster_toolpath_size="<<g_cloud_toolpath_->size()<<std::endl;
    std::cout<<"cluster_toolpath_normal_size="<<cloud_cluster_sel_norm->size()<<std::endl;
    std::cout<<"No Available Toolpath or Targeted Surface for Visualizer!"<<std::endl;
    return;
  }

  //proceed to show, toolpath and surface available
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_surface_color_handler (g_cloud_cluster_, 0, 0, 200);
  viewer->addPointCloud<pcl::PointXYZ>(g_cloud_cluster_, target_surface_color_handler, "target_"+std::to_string(ui.spinBox_toolpaths->value()));
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_"+std::to_string(ui.spinBox_toolpaths->value()));

   // We add the point cloud to the viewer and pass the color handler
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tool_path_color_handler (g_cloud_toolpath_, 255, 0, 0);
  viewer->addPointCloud (g_cloud_toolpath_, tool_path_color_handler, "tool_path_"+std::to_string(ui.spinBox_toolpaths->value()));
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "tool_path_"+std::to_string(ui.spinBox_toolpaths->value()));

  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(g_cloud_toolpath_, cloud_cluster_sel_norm, 1, 2, "normals_"+std::to_string(ui.spinBox_toolpaths->value()));
  ui.qvtkWidget->update();

  //show arrows as lines and cones
  if (ui.checkBox_simple_toolpath->isChecked()){
    PointCloud<PointXYZ>::Ptr toolpath (g_cloud_toolpath_);

    //toolpath =coupon.load_toolpath(toolpath_no);
    std::cout <<"load the toolpath sucessfully, waypoint sizes =" <<toolpath->points.size() << "\n";
    log(Info, std::string("Drawing toolpath for the seleted feature no:"+std::to_string(ui.spinBox_toolpaths->value())));

    pcl::ModelCoefficients coeffs;

    pcl::PointXYZ p1, p2;
    std::string arrow_name;
    arrow_name = "tool_path_"+std::to_string(ui.spinBox_toolpaths->value())+"_";
//   int count = 0;
//   for(int i=0; i<10000;i++)
//   {
//     viewer->removePointCloud (arrow_name+std::to_string(count));
//   }

    int increament = int((toolpath->points.size())/100);
    //if ((toolpath->points.size())<50) increament = 1;
    if (increament<1) increament = 1;
    p1.x =0; p1.y =0; p1.z =0;
    ui.progressBar->setValue(10);
    float avg_pt2pt_dist = 0;
    for (int i=1; i<toolpath->points.size();i++)
    {
      p1.x = toolpath->points[i-1].x;
      p1.y = toolpath->points[i-1].y;
      p1.z = toolpath->points[i-1].z;
      p2.x = toolpath->points[i].x; //p2.x = toolpath->points[i*2].x;  //p2.x = toolpath->points[i*10].x;
      p2.y = toolpath->points[i].y; //p2.y = toolpath->points[i*2].y;  //p2.y = toolpath->points[i*10].y;
      p2.z = toolpath->points[i].z; //p2.z = toolpath->points[i*2].z;  //p2.z = toolpath->points[i*10].z;
      avg_pt2pt_dist += sqrt(pow(fabs(p1.x-p2.x),2)+pow(fabs(p1.y-p2.y),2)+pow(fabs(p1.z-p2.z),2));
    }
    avg_pt2pt_dist = avg_pt2pt_dist/(toolpath->points.size()-2);

    for (int i=1; i < toolpath->points.size(); i++)//for (int i=increament; i < toolpath->points.size(); i=i+increament)
    {
      //cout <<"draw toolpath way points:" << i <<"\n";
      p1.x = toolpath->points[i-1].x;
      p1.y = toolpath->points[i-1].y;
      p1.z = toolpath->points[i-1].z;
      p2.x = toolpath->points[i].x; //p2.x = toolpath->points[i*2].x;  //p2.x = toolpath->points[i*10].x;
      p2.y = toolpath->points[i].y; //p2.y = toolpath->points[i*2].y;  //p2.y = toolpath->points[i*10].y;
      p2.z = toolpath->points[i].z; //p2.z = toolpath->points[i*2].z;  //p2.z = toolpath->points[i*10].z;
      float pt2pt_dist = sqrt(pow(fabs(p1.x-p2.x),2)+pow(fabs(p1.y-p2.y),2)+pow(fabs(p1.z-p2.z),2));
      if (i>0 && i%increament==0 || i>0 && pt2pt_dist>avg_pt2pt_dist*3)
      {
        //viewer->addArrow<pcl::PointXYZ> (p1, p2, 1, 1, 1,arrow_name);
        viewer->addLine<pcl::PointXYZ>(p1, p2, 1, 1, 0, arrow_name+"l_"+std::to_string(i));
        coeffs.values.clear();
        coeffs.values.push_back(p2.x);
        coeffs.values.push_back(p2.y);
        coeffs.values.push_back(p2.z);
        float scale_to_size = 1/sqrt(pow(abs(p1.x-p2.x),2)+pow(abs(p1.y-p2.y),2)+pow(abs(p1.z-p2.z),2))*3;//size 3mm
        coeffs.values.push_back((p1.x-p2.x)*scale_to_size); //coeffs.values.push_back(0.0);
        coeffs.values.push_back((p1.y-p2.y)*scale_to_size); //coeffs.values.push_back(1.0);
        coeffs.values.push_back((p1.z-p2.z)*scale_to_size); //coeffs.values.push_back(0.0);
        coeffs.values.push_back(10); //coeffs.values.push_back(5.0);
        viewer->addCone (coeffs, arrow_name+std::to_string(i));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0, arrow_name+std::to_string(i));
        if (i<=increament*2) viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, arrow_name+std::to_string(i));//start arrow as green
        if (i>=toolpath->points.size()-increament*2) viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, arrow_name+std::to_string(i));//start arrow as red

        ui.qvtkWidget->update();
        //boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        ui.progressBar->setValue(10+90*(i/toolpath->points.size()));
      }
    }
    ui.progressBar->setValue(100);
    log(Info, std::string("Toolpath is showing for the seleted feature no:"+std::to_string(ui.spinBox_toolpaths->value())));
  }
}

void ATG_Window::pushButton_clear_simple_toolpath_clicked(){
  viewer->removePointCloud("target_"+   std::to_string(ui.spinBox_toolpaths->value()));
  viewer->removePointCloud("tool_path_"+std::to_string(ui.spinBox_toolpaths->value()));
  viewer->removePointCloud("normals_"+  std::to_string(ui.spinBox_toolpaths->value()));
  std::string arrow_name;
  arrow_name = "tool_path_"+std::to_string(ui.spinBox_toolpaths->value())+"_";
  for(int i=0; i<10000;i++)
  {
    viewer->removeShape (arrow_name+std::to_string(i));
    viewer->removeShape (arrow_name+"l_"+std::to_string(i));
  }
  ui.qvtkWidget->update();
}

int queue_number = 0;
void ATG_Window::pushButton_reset_queue_clicked()
{
  queue_number=0;
}

void ATG_Window::pushButton_queue_clicked()
{
  //Gathering all input variables from UI
  //combine csv for multiple features selection
  std::string cmd="";
  std::string tcp_info="";
  std::string process="";
  std::string blend_zone="1";
  std::string feedrate="20";
  std::string lift_spd="20";
  std::string opt1="0",opt2="0",opt3="0",opt4="0";

  //===================
  //robot dependent var
  if(ui.comboBox_robot_selection->currentText()=="ABB IRB 2400"
   ||ui.comboBox_robot_selection->currentText()=="ABB IRB 2600"
   ||ui.comboBox_robot_selection->currentText()=="ABB IRB 1200")
  {
    //blend_zone = to_string(ui.doubleSpinBox_zone_radius ->value());
    tcp_info = ui.label_robot_tcp_info_xyzquat_ABB->text().toStdString();
    //cmd = cmd+ui.label_robot_tcp_info_xyzquat_ABB->text().toStdString()+" ";//sys.argv[19]
  }
  else if(ui.comboBox_robot_selection->currentText()=="UR10"
       || ui.comboBox_robot_selection->currentText()=="UR10e"
       || ui.comboBox_robot_selection->currentText()=="UR5e")
  {
    //blend_zone = to_string(ui.doubleSpinBox_blend_radius ->value());
    tcp_info = ui.label_robot_tcp_info_xyzrpy_UR->text().toStdString();
    //cmd = cmd+ui.label_robot_tcp_info_xyzrpy_UR->text().toStdString()+" ";//sys.argv[19]
  }

  //==================
  //tool dependent var
  if(ui.comboBox_tool_selection->currentText()=="Nakanishi Tool")
  {
    feedrate  = std::to_string(ui.doubleSpinBox_nkns_spindle_feedrate     ->value());
    blend_zone= std::to_string(ui.doubleSpinBox_nkns_spindle_zone_radius  ->value());
    lift_spd  = std::to_string(ui.doubleSpinBox_nkns_spindle_lift_speed   ->value());
    opt1      = std::to_string(ui.doubleSpinBox_nkns_spindle_force        ->value());
    opt2      = std::to_string(ui.doubleSpinBox_nkns_spindle_spindle_speed->value());
    process   = "Polishing";
    cmd = "#Line1:Comments Line2:filename;process;tcp_info Line3:feedrate;blend/zone radius;lift_spd;force;RPM;0;0; Line4:obj pose from base frame; Subsequent points are actual points on the coupon";
  }
  else if(ui.comboBox_tool_selection->currentText()=="Masking Tool"
       || ui.comboBox_tool_selection->currentText()=="Masking Tool (old)")
  {
    feedrate  = std::to_string(ui.doubleSpinBox_masking_feedrate    ->value());
    blend_zone= std::to_string(ui.doubleSpinBox_masking_blend_radius->value());
    lift_spd  = std::to_string(ui.doubleSpinBox_masking_lift_speed  ->value());
    opt1      = std::to_string(ui.doubleSpinBox_masking_flowrate    ->value());
    process   = "Masking";
    cmd = "#Line1:Comments Line2:filename;process;tcp_info Line3:feedrate;blend/zone radius;lift_spd;flowrate;0;0;0; Line4:obj pose from base frame; Subsequent points are actual points on the coupon";
  }
  else if(ui.comboBox_tool_selection->currentText()=="Impedance Tool")
  {
    feedrate  = std::to_string(ui.doubleSpinBox_impedance_feedrate    ->value());
    blend_zone= std::to_string(ui.doubleSpinBox_impedance_blend_radius->value());
    lift_spd  = std::to_string(ui.doubleSpinBox_impedance_lift_speed  ->value());
    opt1      = std::to_string(ui.doubleSpinBox_impedance_rpm         ->value());
    process   = "Impedance";
    cmd = "#Line1:Comments Line2:filename;process;tcp_info Line3:feedrate;blend/zone radius;lift_spd;RPM;0;0;0; Line4:obj pose from base frame; Subsequent points are actual points on the coupon";
  }
  else if(ui.comboBox_tool_selection->currentText()=="Sandblasting Nozzle Tool")
  {
    feedrate  = std::to_string(ui.doubleSpinBox_sandblasting_feedrate     ->value());
    blend_zone= std::to_string(ui.doubleSpinBox_sandblasting_zone_radius  ->value());
    lift_spd  = std::to_string(ui.doubleSpinBox_sandblasting_lift_speed   ->value());
    opt1      = std::to_string(ui.doubleSpinBox_sandblasting_blast_dia    ->value());
    opt2      = std::to_string(ui.doubleSpinBox_sandblasting_air_flow_rate->value());
    process   = "Sandblasting";
    cmd = "#Line1:Comments Line2:filename;process;tcp_info Line3:feedrate;blend/zone radius;lift_spd;blast_dia;air_flow_rate;0;0; Line4:obj pose from base frame; Subsequent points are actual points on the coupon";
  }
  else if(ui.comboBox_tool_selection->currentText()=="NDT Tool")
  {
    feedrate  = std::to_string(ui.doubleSpinBox_ndt_feedrate    ->value());
    blend_zone= std::to_string(ui.doubleSpinBox_ndt_blend_radius->value());
    lift_spd  = std::to_string(ui.doubleSpinBox_ndt_lift_speed  ->value());
    opt1      = std::to_string(ui.doubleSpinBox_ndt_flowrate    ->value());
    process   = "NDT";
    cmd = "#Line1:Comments Line2:filename;process;tcp_info Line3:feedrate;blend/zone radius;lift_spd;flowrate;0;0;0; Line4:obj pose from base frame; Subsequent points are actual points on the coupon";
  }
  else
  {
    //do nothing
  }

  std::string objf_X = std::to_string(ui.doubleSpinBox_object_X->value())       ;
  std::string objf_Y = std::to_string(ui.doubleSpinBox_object_Y->value())       ;
  std::string objf_Z = std::to_string(ui.doubleSpinBox_object_Z->value())       ;
  std::string objf_R_ = std::to_string(ui.doubleSpinBox_object_R_->value())     ;
  std::string objf_P_ = std::to_string(ui.doubleSpinBox_object_P_->value())     ;
  std::string objf_Y_ = std::to_string(ui.doubleSpinBox_object_Y_->value())     ;
//  std::string objfs_X = std::to_string(ui.doubleSpinBox_object_X_shift->value());// use double instead
//  std::string objfs_Y = std::to_string(ui.doubleSpinBox_object_Y_shift->value());// use double instead
//  std::string objfs_Z = std::to_string(ui.doubleSpinBox_object_Z_shift->value());// use double instead

/*  cmd += std::to_string(ui.checkBox_frame_check->isChecked())+" ";//sys.argv[10]
  cmd += std::to_string(ui.doubleSpinBox_feedrate->value())  +" ";  //sys.argv[11]
  cmd += blend_zone  +" ";                                     //sys.argv[12]
  cmd += std::to_string(ui.doubleSpinBox_flowrate->value())  +" "; //sys.argv[13]
  cmd += std::to_string(ui.doubleSpinBox_jumpspeed->value()) +" ";//sys.argv[14]
  if(ui.checkBox_fixed_z->isChecked())
  {
      cmd = cmd+to_string(ui.doubleSpinBox_fixed_z->value())+" "+       //sys.argv[15]
                to_string(ui.doubleSpinBox_fixed_z_lift->value()) +" "; //sys.argv[16]
  }
  else
  {
      cmd = cmd+"0 0 ";
  }
  cmd = cmd+to_string(ui.doubleSpinBox_polish_spindle_speed->value())+" "+   //sys.argv[17]
            to_string(ui.doubleSpinBox_polish_force->value()) +       " ";   //sys.argv[18]
  cmd = cmd+coupon.file_name_short+"_cluster"+to_string(ui.spinBox_clusters->value())+" ";//sys.argv[19]
  cmd += tcp_info  +" ";  //sys.argv[20] */

  //system(cmd.c_str());


  //write too robot_toolpath.csv
  /* Format as below
   #Line1:Comments Line2:filename, Line3:obj pose from base frame; Line4:feedrate,blend/zone radius,flowrate,jumpspeed,fixed_z,fixed_z_lift  ,Subsequent points are actual points on the coupon
   ScanObject033_polish_1_cluster4 p[-0.05232,-0.05155,0.27800,0.00000,0.00000,0.00000]
   -406.0,-853.0,500.0,-39.06,172.64,38.33
   20.0,2.0,0.8,50.0,0.0,0.0,5000.0,3.0
   126.039,193.364,50.7158,176.507,-11.2457,-145.586,0
   114.338,189.779,-8.0229,176.507,-11.2457,-145.586,1
   ...
   */
  std::string filename1 = ex_data_file_dir_.toStdString()+"cache/toolpath/robot_toolpath.csv";// /home/data/cache/toolpath/robot_toolpath.csv
  std::string filename2 = in_data_file_dir_.toStdString()+"tool_path_back_projected_w_lift_n_trigger.txt"; // <package>/data/coupons/tool_path_back_projected_w_lift_n_trigger.txt
  std::ofstream outfile;
  std::ifstream indata;
  outfile.open(filename1);
  //outfile << "#Line1:Comments Line2:filename;process;tcp_info; Line3:feedrate;blend/zone radius;lift_spd;opt1;opt2;opt3;opt4; Line4:obj pose from base frame;  ,Subsequent points are actual points on the coupon" << std::endl;
  outfile << cmd << std::endl;
  outfile << g_short_filename_.toStdString()<<"_cluster"<<std::to_string(ui.spinBox_clusters->value())<< ";" << process << ";" << tcp_info << std::endl;
  outfile << feedrate << ";" << blend_zone << ";" << lift_spd << ";" << opt1 << ";" << opt2 << ";" << opt3 << ";" << opt4 << ";" << std::endl;
  outfile << objf_X << ";" << objf_Y << ";" << objf_Z << ";" << objf_R_ << ";" << objf_P_ << ";" << objf_Y_ << ";" << std::endl;
  indata.open(filename2);
  int lineNum = 0;
  std::string line,cell;
  while (std::getline(indata,line))
  {
    int cellNum = 0;
    lineNum++; std::stringstream lineStream(line);//input-> 0;1;2; 3; 4; 5;6;7;8; 9;10;11;12;13
    while(std::getline(lineStream,cell,';'))      //input-> x;y;z;nx;ny;nz;r;p;y;q1;q2;q3;q4;sw
    {
           if(cellNum==0) outfile << std::to_string(strtod(cell.c_str(),NULL)+ui.doubleSpinBox_object_X_shift->value()) << ";";
      else if(cellNum==1) outfile << std::to_string(strtod(cell.c_str(),NULL)+ui.doubleSpinBox_object_Y_shift->value()) << ";";
      else if(cellNum==2) outfile << std::to_string(strtod(cell.c_str(),NULL)+ui.doubleSpinBox_object_Z_shift->value()) << ";";
      else if(cellNum==6
            ||cellNum==7
            ||cellNum==8) outfile << std::to_string(strtod(cell.c_str(),NULL)) << ";";
      else if(cellNum==13)outfile << std::to_string(atoi(cell.c_str())) << ";";
      cellNum++;
    }
    outfile << std::endl;
  }
  outfile.close();
  indata.close();
  std::cout << "Generated Toolpath for cluster " << std::to_string(ui.spinBox_clusters->value()) << std::endl;

  //append to robot_toolpath_queue.csv, if first on queue copy all, if second and later, append from line 4
  queue_number=queue_number+1;
//  std::ifstream indata;
//  std::ofstream outfile;
//  std::string line;
  lineNum = 0;
  filename1 = ex_data_file_dir_.toStdString()+"cache/toolpath/robot_toolpath.csv";// /home/data/cache/toolpath/robot_toolpath.csv
  filename2 = ex_data_file_dir_.toStdString()+"cache/toolpath/robot_toolpath_queued.csv";//"src/masking/config/robot_masking_queue.csv";
  indata.open(filename1);
  //Open the file
  if (queue_number==1)
  {
    outfile.open(filename2);
    while ( std::getline(indata, line, '\n') )
    {
      outfile << line << std::endl;
    }
    outfile.close();
    std::cout << "Appended Toolpath to new queue for cluster " << std::to_string(ui.spinBox_clusters->value()) << std::endl;
    log(Info,std::string("Generated toolpath for cluster and created new toolpath queue!"));
  }
  if (queue_number>1)
  {
    outfile.open(filename2,std::ios_base::app);
    while ( std::getline(indata, line, '\n') )
    {
      lineNum++;
      if (lineNum <= 4){continue;}
      if (lineNum > 4 )
      {
         outfile << line << std::endl;
      }
    }
    outfile.close();
    std::cout << "Appended Toolpath to queue for cluster " << std::to_string(ui.spinBox_clusters->value()) << std::endl;
    log(Info,std::string("Generated toolpath for cluster and appended to toolpath queue!"));
  }

}


/******************************************
** SLOTS for robot tab
*******************************************/
void ATG_Window::pushButton_robotSetup_clicked()
{
  std::string cmd, text_before, text_after, filename="";
  cmd = "killall xterm";
  system(cmd.c_str());
  writeQSettings();

  //check tool_model package, return if not valid
  std::string package_name = ros::package::getPath("tool_model");
  if(package_name == "")
  {
    std::cout << "tool_model package not found!" << std::endl;
    return;
  }

  std::string env_xacro_dir = "/urdf/blank_env.xacro";//dummy default value
       if(ui.comboBox_env_selection->currentText().toStdString()=="Blank Environment")  {/*env_xacro_dir="/urdf/blank_env.xacro"*/;    }
  else if(ui.comboBox_env_selection->currentText().toStdString()=="Masking Table (old)"){env_xacro_dir="/urdf/masking_table_old.xacro";}
  else if(ui.comboBox_env_selection->currentText().toStdString()=="Masking Table")      {env_xacro_dir="/urdf/masking_table.xacro";    }
  else if(ui.comboBox_env_selection->currentText().toStdString()=="Round Table R06")    {env_xacro_dir="/urdf/round_table(r06).xacro"; }

  std::string robot_sh = package_name+"/sh/ur10e_robot_setup.sh";//dummy default value
       if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10")         {robot_sh=package_name+"/sh/ur10_robot_setup.sh";     }
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10e")        {/*robot_sh=package_name+"/sh/ur10e_robot_setup.sh"*/;}
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="UR5e")         {robot_sh=package_name+"/sh/ur5e_robot_setup.sh";}
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2400") {robot_sh=package_name+"/sh/irb2400_robot_setup.sh";  }
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2600") {robot_sh=package_name+"/sh/irb2600_robot_setup.sh";  }
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 1200") {robot_sh=package_name+"/sh/irb1200_robot_setup.sh";  }

  std::string tool_xacro_dir = "/urdf/EcoPen450_Pen_Tool.xacro";//dummy default value
       if(ui.comboBox_tool_selection->currentText().toStdString()=="Masking Tool (old)")       {tool_xacro_dir="/urdf/EcoPen450_Pen_Tool_old.xacro";}
  else if(ui.comboBox_tool_selection->currentText().toStdString()=="Masking Tool")             {/*tool_xacro_dir="/urdf/EcoPen450_Pen_Tool.xacro"*/;}
  else if(ui.comboBox_tool_selection->currentText().toStdString()=="Impedance Tool")           {tool_xacro_dir="/urdf/EE3DOF.xacro";                }
  else if(ui.comboBox_tool_selection->currentText().toStdString()=="Sandblasting Nozzle Tool") {tool_xacro_dir="/urdf/Sandblasting_Nozzle.xacro";   }
  else if(ui.comboBox_tool_selection->currentText().toStdString()=="Nakanishi Tool")           {tool_xacro_dir="/urdf/ATI_w_Spindle.xacro";         }
  else if(ui.comboBox_tool_selection->currentText().toStdString()=="NDT Tool")                 {tool_xacro_dir="/urdf/NDT_Tool.xacro";              }

  //generate object frame yaml
  if(1)
  {
    std::string filename_output = package_name + "/config/obj_frame.yaml";
    std::ofstream outfile; outfile.open(filename_output);
    outfile << "##added by ATG for object frame\n";
    outfile << "coupon_file: "+ex_data_file_dir_.toStdString()+"cache/pclmini.ply\n";
    outfile << "X: "+std::to_string(ui.doubleSpinBox_object_X->value()/1000)+"\n";
    outfile << "Y: "+std::to_string(ui.doubleSpinBox_object_Y->value()/1000)+"\n";
    outfile << "Z: "+std::to_string(ui.doubleSpinBox_object_Z->value()/1000)+"\n";
    outfile << "R_: "+std::to_string(ui.doubleSpinBox_object_R_->value()*M_PI/180)+"\n";//output as RPY rad
    outfile << "P_: "+std::to_string(ui.doubleSpinBox_object_P_->value()*M_PI/180)+"\n";//output as RPY rad
    outfile << "Y_: "+std::to_string(ui.doubleSpinBox_object_Y_->value()*M_PI/180)+"\n";//output as RPY rad
    outfile.close();
  }

  //generate environment params yaml
  if(1)
  {
    std::string filename_output = package_name + "/config/env_params.yaml";
    std::ofstream outfile; outfile.open(filename_output);
    outfile << "##added by ATG for env params\n";
    outfile << "package: \"tool_model\" \n";
    outfile << "xacro_dir: "+env_xacro_dir+"\n";
    outfile << "X: "+std::to_string(ui.doubleSpinBox_env_X->value()/1000)+"\n";
    outfile << "Y: "+std::to_string(ui.doubleSpinBox_env_Y->value()/1000)+"\n";
    outfile << "Z: "+std::to_string(ui.doubleSpinBox_env_Z->value()/1000)+"\n";
    outfile << "R_: "+std::to_string(ui.doubleSpinBox_env_R_->value()*M_PI/180)+"\n";//output as RPY rad
    outfile << "P_: "+std::to_string(ui.doubleSpinBox_env_P_->value()*M_PI/180)+"\n";//output as RPY rad
    outfile << "Y_: "+std::to_string(ui.doubleSpinBox_env_Y_->value()*M_PI/180)+"\n";//output as RPY rad
  }

  //generate tcp and robot yaml
  if(1)
  {
    std::string filename_output = package_name + "/config/tcp_params.yaml";
    std::ofstream outfile; outfile.open(filename_output);
    outfile << "#added by ATG for tcp change in tilt and diameter\n";
    outfile << "package: \"tool_model\" \n";
    outfile << "xacro_dir: "+tool_xacro_dir+"\n";
    std::stringstream lineStream(ui.label_robot_tcp_info_xyzrpy->text().toStdString());
    std::string cell;
    Eigen::VectorXd xyzrpy(6);
    int i = 0;bool invalid_errors = 0;
    try
    {
      while (std::getline(lineStream, cell, ' '))
      {
        //if (i==0)//only text
        if (i==0) {outfile << "X: "   + cell + "\n";}
        if (i==1) {outfile << "Y: "   + cell + "\n";}
        if (i==2) {outfile << "Z: "   + cell + "\n";}
        if (i==3) {outfile << "R_: "  + cell + "\n";}
        if (i==4) {outfile << "P_: "  + cell + "\n";}
        if (i==5) {outfile << "Y_: "  + cell + "\n";}
        if (i==6) {outfile << "eeX: " + cell + "\n";}
        if (i==7) {outfile << "eeY: " + cell + "\n";}
        if (i==8) {outfile << "eeZ: " + cell + "\n";}
        if (i==9) {outfile << "eeR_: "+ cell + "\n";}
        if (i==10){outfile << "eeP_: "+ cell + "\n";}
        if (i==11){outfile << "eeY_: "+ cell + "\n";}
        if (i==12){outfile << "dia: " + cell + "\n";}
        i++;
      }
    }
    catch(...)
    {
      //do nothing
    }
  }

  //launch robot shell script -> launch file
  sleep(3);
  text_before = "Launching "+ ui.comboBox_robot_selection->currentText().toStdString() +" Robot Setup node...";
  cmd = "xterm -hold -e '"+robot_sh+"' &"; //+hold closes window
  log(Info,text_before);
  //boost::this_thread::sleep(boost::posix_time::microseconds(10000));
  system(cmd.c_str());
  cmd = "sleep 1; xdotool search --class xterm set_desktop_for_window %@ 1";
  system(cmd.c_str());
}

void ATG_Window::pushButton_robotSim_clicked()
{
  //check tool_model package, return if not valid
  std::string package_name = ros::package::getPath("tool_model");
  if(package_name == "")
  {
    std::cout << "tool_model package not found!" << std::endl;
    return;
  }

  std::string cmd= "xterm -hold -e '"+package_name+"/sh/ur10e_sim.sh' & ";
       if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10")        {cmd = "xterm -hold -e '"+package_name+"/sh/ur10_robot_sim.sh' & ";} //+hold closes window
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10e")       {/*cmd = "xterm -hold -e '"+package_name+"/sh/ur10e_sim.sh' & "*/;     } //+hold closes window
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="UR5e")        {cmd = "xterm -hold -e '"+package_name+"/sh/ur5e_sim.sh' & ";     } //+hold closes window
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2400"){cmd = "xterm -hold -e '"+package_name+"/sh/irb2400_sim.sh' & ";   } //+hold closes window
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2600"){cmd = "xterm -hold -e '"+package_name+"/sh/irb2600_sim.sh' & ";   } //+hold closes window
  else if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 1200"){cmd = "xterm -hold -e '"+package_name+"/sh/irb1200_sim.sh' & ";   } //+hold closes window

  std::string log_text;
  log_text = "Launching " + ui.comboBox_robot_selection->currentText().toStdString() + " Simulation node...";
  log(Info,log_text);
  //boost::this_thread::sleep(boost::posix_time::microseconds(10000));
  system(cmd.c_str());
  cmd = "sleep 1; xdotool search --class xterm set_desktop_for_window %@ 1; xdotool search --class RViz windowactivate;";
  system(cmd.c_str());
}

void ATG_Window::pushButton_robotRun_clicked()
{
  if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="UR10e"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="UR5e")
    RunScriptFunction("URScript.txt");
  if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2400"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2600"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 1200")
    RunScriptFunction("abb_script.mod");

}

void ATG_Window::pushButton_runScript_clicked()
{
  RunScriptFunction(ui.comboBox_robotscript->currentText().toStdString());
}

void ATG_Window::RunScriptFunction(std::string scriptname)
{
  //check tool_model package, return if not valid
  std::string package_name = ros::package::getPath("atg_robot_motion");
  if(package_name == "")
  {
    std::cout << "atg_robot_motion package not found!" << std::endl;
    return;
  }
  std::string cmd, text_before, text_after;
  //align_pcd_to_object_frame(); //not required on run script
  if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="UR10e"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="UR5e")
    cmd = "xterm -hold -e '"+package_name+"/sh/urscript_send.sh' "+ui.line_edit_robot_ip->text().toStdString()+" "+ui.spinBox_robot_port->text().toStdString()+" "+scriptname+" &  "; //+hold closes window
  if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2400"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2600"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 1200")
    cmd = "xterm -hold -e '"+package_name+"/sh/abb_rapid_send.sh' "+ui.line_edit_robot_ip->text().toStdString()+" "+scriptname+" &  "; //+hold closes window
  log(Info,text_before);
  system(cmd.c_str());
  log(Info,std::string("All codes are sucessfully injected!"));
}

void ATG_Window::pushButton_saveScriptAs_clicked()
{
  QString str_exDir = "/home/data/cache/urscript";
  QString current_script = "/URScript.txt";
  QString filename = "";
  QString ext  = ".txt";
  QString ext_ = "*.txt";
  if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="UR10e"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="UR5e")
  {
    str_exDir = "/home/data/cache/urscript";
    ext_ = "*.txt";
    ext  = ".txt";
    current_script = "/URScript.txt";
  }
  if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2400"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2600"
   ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 1200")
  {
    str_exDir = "/home/data/cache/RAPID";
    ext_ = "*.mod";
    ext  = ".mod";
    current_script = "/abb_script.mod";
  }

  QString sfileName = QFileDialog::getSaveFileName(this,
                      tr("Save Current Script As"), str_exDir,
                      ext_,&filename,QFileDialog::DontUseNativeDialog);
  if (sfileName.length() == 0)
  {
    QMessageBox::information(NULL, tr("Warning"), tr("Invalid file name."));
  }
  else
  {
    //the returned fileName will not be appended with the extension automatically
    //will need to check if extension has been appended when using
    QString file_extention = filename;
    file_extention.remove(0,1); //removing * from *.txt
    int finder = sfileName.indexOf(file_extention,0,Qt::CaseInsensitive);
    //std::cout << "Found " << extension.toStdString() << " @ " << finder << "/" << sfileName.length() << std::endl;
    if(finder<sfileName.length()-4)
    {
      //filename keyed is without extension,
      //add extension, check for special char and do a last check to see if file exist
      sfileName += ext;//".txt";
      QFileInfo fi(sfileName);
      //std::cout << "Finding special char from " << fi.fileName().toStdString() << std::endl;
      if (fi.fileName().contains(QRegExp("[^A-Za-z0-9_()\\[\\]\\.\\-]")))// doublebackslash special char = special char
      {
        QMessageBox::warning(NULL, tr("Warning"), tr("Invalid file name.\n"));
        return;
      }
      if (boost::filesystem::exists(sfileName.toStdString()))
      {
        QMessageBox::StandardButton reply = QMessageBox::warning(this,"Save Current Script As",
                                            fi.fileName()+" already exists.\nDo you want to replace it?",
                                            QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::No)return;
        //if (reply == QMessageBox::Yes){}//continue to save file
      }
    }
    std::cout << "Saving script into " << sfileName.toStdString() << std::endl;
    try
    {
      boost::filesystem::copy(str_exDir.toStdString()+current_script.toStdString(),sfileName.toStdString());
      log(Info, std::string("Saved script to "+sfileName.toStdString()));
    }
    catch(boost::filesystem::filesystem_error const & e)
    {
      std::cerr << e.what() << '\n';
      return;
    }
  }
  pushButton_refreshScript_clicked();
}

void ATG_Window::pushButton_refreshScript_clicked()
{
    ui.comboBox_robotscript->clear();
    try
    {
      std::string str_exDir = "/home/data/cache/urscript";
      std::string ext_ = ".txt";
      if(ui.comboBox_robot_selection->currentText().toStdString()=="UR10"
       ||ui.comboBox_robot_selection->currentText().toStdString()=="UR10e"
       ||ui.comboBox_robot_selection->currentText().toStdString()=="UR5e")
      {str_exDir = "/home/data/cache/urscript";ext_ = ".txt";}
      if(ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2400"
       ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 2600"
       ||ui.comboBox_robot_selection->currentText().toStdString()=="ABB IRB 1200")
      {str_exDir = "/home/data/cache/RAPID";ext_ = ".mod";}
      const boost::filesystem::path root = str_exDir;
      const std::string ext = ext_;
      std::vector<boost::filesystem::path> ret;
      if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;
      boost::filesystem::recursive_directory_iterator it(root);
      boost::filesystem::recursive_directory_iterator endit;
      //std::cout<< "File system read.." <<std::endl;
      while(it != endit)
      {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
        {
          ret.push_back(it->path().filename());
          //std::cout << "Found " << it->path().filename() << std::endl;
          ui.comboBox_robotscript->addItem(QString::fromStdWString(it->path().filename().wstring()));
        }
        //std::cout << "Found Not " << it->path().filename() << std::endl;
        ++it;
      }
      std::cout<< "Find Script Refresh Completed!" <<std::endl;
    }
    catch (std::system_error & e)
    {
      std::cerr << "Find Script Exception :: " << e.what();
    }
}

/******************************************
** point cloud functions
*******************************************/
pcl::PointCloud<pcl::PointXYZ>::Ptr  ATG_Window::load_point_cloud_data(std::string file_name,bool make_mini)
{
  std::string src_filename, dst_filename;
  std::string tmp_pcd_filename = "data/coupons/tmp.pcd";   //josh added for sharefile write error, pcd cannot write w filelock
  std::string cmd, output_dir;
  Converter Conv_file_format;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZ>);


  std::cout <<"---------------------------------------------------------------------------\n";
  std::cout <<"-------------------------Load Point Cloud From File------------------------\n";
  std::cout <<"---------------------------------------------------------------------------\n";

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

  //prepare pcd for RViz
  if(make_mini)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mini (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_mini = point_cloud_manipulate(cloud_origin, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001);

    std::string obj_filename_mini = ex_data_file_dir_.toStdString()+"cache/pclmini.obj";
    std::string pcd_filename_mini = ex_data_file_dir_.toStdString()+"cache/pclmini.pcd";
    io::savePCDFile(pcd_filename_mini, *cloud_mini);
    Conv_file_format.pcd2obj(pcd_filename_mini, obj_filename_mini);

    std::string cmd;
    //Josh doing point cloud conversion using meshlabserver function into downsampled PLY
    std::string stl_filename_mini = ex_data_file_dir_.toStdString()+"cache/pclmini.ply";// "/home/data/cache/pclmini.ply"
    std::string mlx_filename_mini = "src/atg_gui_module/script/poisson.mlx";//TODO: change structure to source for atg_gui package file

    cmd="xterm +hold -e 'meshlabserver -i " + obj_filename_mini+" -o "+ //+hold to close, -hold to hold
                              stl_filename_mini+" -s "+
                              mlx_filename_mini+" -om vc vn fq' &";
    system(cmd.c_str());
    cmd = "sleep 1; xdotool search --class xterm set_desktop_for_window %@ 1";
    system(cmd.c_str());

    std::cout<<"Point cloud and mesh data for Rviz created!"<< std::endl;
  }
  return (cloud_origin);
}

pcl::PointCloud<PointXYZ>::Ptr ATG_Window::point_cloud_manipulate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float x, float y, float z, float rx, float ry, float rz, float scale)
{
  std::cout << "toolpath_gen point_cloud_manipulate - received the signal value x,y,z,Rx,Ry,Rz,scale=" <<x <<","<<y <<","<<z <<","<<rx <<","<<ry <<","<<rz <<","<<scale <<"\n"; //josh added
  //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  source_cloud = cloud;     // Load PCD file

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << x, y, z;

  // The same rotation matrix as before; theta radians around Z axis
  Eigen::Matrix3f m2;
  float angle1 = 0; if (rx !=0) angle1 = rx*3.1415/180;
  float angle2 = 0; if (ry !=0) angle2 = ry*3.1415/180;
  float angle3 = 0; if (rz !=0) angle3 = rz*3.1415/180;
  m2 = Eigen::AngleAxisf (angle1, Eigen::Vector3f::UnitZ())
      *Eigen::AngleAxisf (angle2, Eigen::Vector3f::UnitX())
      *Eigen::AngleAxisf (angle3, Eigen::Vector3f::UnitY());
  transform_2.rotate (m2);

  transform_2.scale(scale);

  std::cout << "\nMethod #2: using an Affine3f\n";
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  return (transformed_cloud);

}

/******************************************
** mouse click events
*******************************************/
void ATG_Window::mouseEventProcess(const visualization::MouseEvent &event)
{
  if(1)//hover mode
  {
    //std::cout <<"hover @ x=" << event.getX() << "  y=" << event.getY() << std::endl;
    //custom point pointpickingevent, bypass pointPickingEvent to look for clicked point in renderwindow
    float x = 0, y = 0, z = 0;
    vtkRenderWindowInteractor* iren = viewer->getRenderWindow()->GetInteractor();
    vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast (iren->GetPicker ());
    if (!point_picker)
    {
      //std::cout << "Point picker not available, not selecting any points!\n" << std::endl;
      return;
    }
    iren->StartPickCallback ();
    vtkRenderer *ren = iren->FindPokedRenderer (event.getX(), event.getY());
    point_picker->SetTolerance(0.001);//original tolerance set was 0.05, set lower stabilises null detection 0.002 good
    point_picker->Pick (event.getX(), event.getY(), 0.0, ren);
    int ppindex = static_cast<int> (point_picker->GetPointId ());
    //find clicked point from clusters, if point found, select cluster and click visualize, else click clear visualise
    if (point_picker->GetDataSet () != NULL && ppindex!=0)
    {
      double p[3];
      point_picker->GetDataSet ()->GetPoint (ppindex, p);
      x = float (p[0]); y = float (p[1]); z = float (p[2]);
      //std::cout << "Mouse Hover @ Point coordinate ( " << x << ", " << y << ", " << z << ") index = " << ppindex << std::endl;
      //highlight point
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clicked_point (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ point;
      point.x = x;
      point.y = y;
      point.z = z;
      //highlght around radius from cloud_open
      float radius = ui.doubleSpinBox_selection_radius->value(); //mm scale
      if (radius==0) radius=0.01;
      if (radius>0 && g_cloud_opened_->empty()==0)
      {
        //cloud_opened set when openclicked
        //kdtree_cloud_open.setInputCloud (cloud_opened); set when openclicked, if declared here = performance very slow
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( g_kdtree_cloud_open_.radiusSearch (point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
          g_hover_mouse_area_->clear();
          for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
          {
            point.x = g_cloud_opened_->points[ pointIdxRadiusSearch[i] ].x;
            point.y = g_cloud_opened_->points[ pointIdxRadiusSearch[i] ].y;
            point.z = g_cloud_opened_->points[ pointIdxRadiusSearch[i] ].z;
            cloud_clicked_point->push_back(point);
            g_hover_mouse_area_->push_back(point);
          }
        }
      }
      if(!cloud_clicked_point->empty())
      {

        viewer->removePointCloud ("cloud_hover_point");
        viewer->addPointCloud (cloud_clicked_point, "cloud_hover_point");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 1.0f, "cloud_hover_point");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud_hover_point");
        ui.qvtkWidget->update ();
        g_hover_mouse_point_->clear(); //reset first, only 1 single point required
        g_hover_mouse_point_->push_back(cloud_clicked_point->points[0]);//return verified point that exist in cloud_open(and not other mesh/lines)
        viewer->removeShape("text_hover_point_x");
        viewer->removeShape("text_hover_point_y");
        viewer->removeShape("text_hover_point_z");
        std::string s = "                         ";//25 spaces
        viewer->addText(s+    "X:" + std::to_string(g_hover_mouse_point_->points[0].x),0,0,11,1,1,1,"text_hover_point_x");
        viewer->addText(s+s+  "Y:" + std::to_string(g_hover_mouse_point_->points[0].y),0,0,11,1,1,1,"text_hover_point_y");
        viewer->addText(s+s+s+"Z:" + std::to_string(g_hover_mouse_point_->points[0].z),0,0,11,1,1,1,"text_hover_point_z");

      }
      //shift click operation in cluster mode to be placed here, identify cluster required and cannot be done in line_draw_mode
      //Double Click, find in mapper and highlight in visualizer
      if (event.getButton()==pcl::visualization::MouseEvent::LeftButton) //&& event.getType()  ==pcl::visualization::MouseEvent::MouseDblClick)
      {
          //find point from mapper in CloudActorMap
          std::string cluster_found = "";
          bool pos_found = 0;
          if(point_picker)
          {
            // Get the VTK class used to render the cloud for comparison.
            vtkPolyData* points = reinterpret_cast<vtkPolyData*> (point_picker->GetDataSet());
            // Search the visualisers CloudActor instances to find the one using 'points'.
            for (auto iter = viewer->getCloudActorMap()->begin(); iter != viewer->getCloudActorMap()->end(); ++iter)
            {
              pcl::visualization::CloudActor &actor = iter->second;
              //std::cout << "Point Search - ActorName (" << iter->first.c_str () << ")" << std::endl;//actor.actor->GetProperty()->GetInformation()
              // The vktPolyData for the cloud is wrapped up in the actor's vtkMapper...
              if (vtkMapper *mapper = actor.actor->GetMapper())
              {
                // We've hit pay dirt when the mapper input matches 'points'.
                if (mapper->GetInput() == points)
                {
                  cluster_found = iter->first.c_str ();
                  if(cluster_found.find("cluster_")!=std::string::npos) pos_found = 1;
                  //pos_found = cluster_found.find("cluster_");
                  std::cout << "Clicked point in - " << iter->first.c_str () << "  find-" << pos_found << std::endl;
                  break;
                }
              }
            }
          }

          //Double Click, find in mapper and highlight in visualizer
          if(event.getType()  ==pcl::visualization::MouseEvent::MouseDblClick && g_keyboardkey_ != "Shift_L-keydown")
          {
            //highlight cluster if valid cluster
            if (cluster_found!="" && pos_found!=0 && cluster_found!="cluster_rejected")//double clicked on cluster_x, segmentation done, viz cluster
            {
              cluster_found.replace(0,cluster_found.find("_")+1,"");
              ui.pushButton_clear_clusters->click();
              ui.spinBox_clusters->setValue(stoi(cluster_found));
              ui.pushButton_visualize_clusters->click();
            }
            else if(cluster_found=="cloud_main")//double clicked on cloud1, segmentation not done, build new cluster
            {
              std::cout << "Time for new clusters!" << std::endl;
              pushButton_segmentation_clicked();//temporary do full custering, should implement partial clustering
            }
          }

          //L-click on cluster mode
          if(ui.comboBox_interaction_mode->currentText().toStdString()=="Cluster" && g_keyboardkey_ == "Shift_L-keydown"
          && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
          {
            //highlight cluster in spinbox if valid cluster
            if (cluster_found!="" && pos_found!=0 && cluster_found!="cluster_rejected")//double clicked on cluster_x, segmentation done, viz cluster
            {
              cluster_found.replace(0,cluster_found.find("_")+1,"");
              ui.spinBox_IM_cluster_no->setValue(std::stoi(cluster_found));
              pushButton_IM_cluster_select_clicked();//need to add back
            }
          }
      }
      //Single Click events, coupled with interactive mode
      if (event.getButton()==pcl::visualization::MouseEvent::LeftButton
       && event.getType()==pcl::visualization::MouseEvent::MouseButtonPress)
      {
        //interactive click changed to be used on shift click pointPickingEventOccurred() instead
        //if(cloud_clicked_point->empty()==0)
        //  line_draw_mode(ui.comboBox_interaction_mode->currentText().toStdString(),cloud_clicked_point->points[0]);
      }

    }
    else
    {
      //hover on empty region
      viewer->removePointCloud ("cloud_hover_point");
      g_hover_mouse_point_->clear();//clear global var so that it doesnt return points not in cloud_open
      g_hover_mouse_area_->clear();
      //ui.button_clear_clusters->click();
      //std::cout << "Hovering @ Empty Region." << std::endl;
      if (event.getButton()==pcl::visualization::MouseEvent::LeftButton
       && event.getType()==pcl::visualization::MouseEvent::MouseDblClick)
      {
        ui.pushButton_clear_clusters->click();
      }
    }
  }
}

void ATG_Window::areaPickingEventProcess(const visualization::AreaPickingEvent &event)
{
  //handled within mouseEvent
}

void ATG_Window::pointPickingEventOccurred(const visualization::PointPickingEvent &event)
{
  if (!g_hover_mouse_point_->empty())
  {
    if(ui.comboBox_interaction_mode->currentText().toStdString()!="Cluster")
    {line_draw_mode(ui.comboBox_interaction_mode->currentText().toStdString(),g_hover_mouse_point_->points[0]);}
    std::cout << "Point coordinate ( " << g_hover_mouse_point_->points[0].x << ", "
                                       << g_hover_mouse_point_->points[0].y << ", "
                                       << g_hover_mouse_point_->points[0].z << ")" << std::endl;
  }
}

void ATG_Window::keyboardEventProcess(const visualization::KeyboardEvent &event)
{
  //string key = event.getKeySym();
  g_keyboardkey_ = event.getKeySym();
  //std::cout<<key;
  if(event.keyDown())
  {
    //std::cout<<"-keydown"<<std::endl;
    g_keyboardkey_+="-keydown";
    std::cout<<g_keyboardkey_<<std::endl;
  }
  else if(event.keyUp())
  {
    //std::cout<<"-keyup"<<std::endl;
    if (g_keyboardkey_=="Escape")
    {
      ui.comboBox_interaction_mode->setCurrentText("None");
    }
    if (g_keyboardkey_=="exclam")
    {
      ui.comboBox_interaction_mode->setCurrentText("Poly");
    }
    if (g_keyboardkey_=="at") ui.comboBox_interaction_mode->setCurrentText("Circle");
    if (g_keyboardkey_=="numbersign") ui.comboBox_interaction_mode->setCurrentText("Cluster");
    g_keyboardkey_+="-keyup";
    std::cout<<g_keyboardkey_<<std::endl;
  }
}

void ATG_Window::comboBox_interaction_mode_TextChanged(QString text_comboBox)
{
  if (text_comboBox == "None")
  {
    ui.widget_interactive_modes ->hide();
    if(ui.doubleSpinBox_selection_radius->value()!=0.01)
      ui.doubleSpinBox_selection_radius ->setValue(0.01);
    line_draw_mode(text_comboBox.toStdString());
  }
  if (text_comboBox == "Poly")
  {
    ui.widget_interactive_modes ->show();
    ui.widget_IM_radius_select  ->show();
    ui.widget_IM_cluster_select ->hide();
    ui.label_interaction_inst   ->setText("SHIFT+LClick @ points, Active for RED points only.");
    ui.label_interaction_inst   ->setStyleSheet("QLabel { color : red; }");
    ui.radioButton_interaction_invert-> setChecked(true);
    ui.tabWidget_Flow ->setCurrentIndex(2); //set main tab
    ui.tabWidget_seg  ->setCurrentIndex(2); //set sub tab
    if(ui.doubleSpinBox_selection_radius->value()!=0.01)
      ui.doubleSpinBox_selection_radius ->setValue(0.01);
  }
  if (text_comboBox == "Circle")
  {
    ui.widget_interactive_modes ->show();
    ui.widget_IM_radius_select  ->show();
    ui.widget_IM_cluster_select ->hide();
    ui.label_interaction_inst   ->setText("SHIFT+LClick @ points, Active for BLUE points only.");
    ui.label_interaction_inst   ->setStyleSheet("QLabel { color : blue; }");
    ui.radioButton_interaction_union-> setChecked(true);
    ui.tabWidget_Flow ->setCurrentIndex(2); //set main tab
    ui.tabWidget_seg  ->setCurrentIndex(2); //set sub tab
    if(ui.doubleSpinBox_selection_radius->value()==0.00)
      ui.doubleSpinBox_selection_radius ->setValue(0.01);
  }
  if (text_comboBox == "Cluster")
  {
    ui.widget_interactive_modes ->show();
    ui.widget_IM_radius_select  ->hide();
    ui.widget_IM_cluster_select ->show();
    ui.label_interaction_inst   ->setText("SHIFT+LClick @ points, Active for DarkGreen points only.");
    ui.label_interaction_inst   ->setStyleSheet("QLabel { color : darkGreen; }");
    ui.radioButton_interaction_invert-> setChecked(true);
    ui.tabWidget_Flow ->setCurrentIndex(2); //set main tab
    ui.tabWidget_seg  ->setCurrentIndex(2); //set sub tab
    if(ui.doubleSpinBox_selection_radius->value()!=0.01)
      ui.doubleSpinBox_selection_radius ->setValue(0.01);
  }
}

void ATG_Window::interactive_tab_signal(int tab_no)
{
  if (ui.tabWidget_Flow->currentIndex()!=2 || ui.tabWidget_seg->currentIndex()!=2)
  {//out of draw tab, set selection to none
    line_draw_mode("None");
  }
}

void ATG_Window::pushButton_IM_cluster_select_clicked()
{
    line_draw_mode(ui.comboBox_interaction_mode->currentText().toStdString(),g_hover_mouse_point_->points[0]);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle_area (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_poly_area (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_poly_points (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_area (new pcl::PointCloud<pcl::PointXYZ>);
void ATG_Window::line_draw_mode(std::string event_name, pcl::PointXYZ point)
{
  //dependent on g_cloud_opened_ and g_kdtree_cloud_open_
  //dependent on comboBox_interactive_mode
  //dependent on MouseEventProcess to create pointclouds hover_mouse_point, hover_mouse_area
  //outputs pointcloud into global var cloud_circle_area, cloud_poly_area, cloud_poly_points
  if(event_name == "Poly")
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clicked_point (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_clicked_point->push_back(point);
    //highlght around radius from cloud_open
    float radius = ui.doubleSpinBox_selection_radius->value(); //mm scale
    if (radius>0 && g_cloud_opened_->empty()==0)
    {
      //========START append hover_points to cloud_poly_points
      int interaction_mode = 0;
      if(ui.radioButton_interaction_union->    isChecked()) interaction_mode = 0;
      if(ui.radioButton_interaction_subtract-> isChecked()) interaction_mode = 1;
      if(ui.radioButton_interaction_intersect->isChecked()) interaction_mode = 2;
      if(ui.radioButton_interaction_invert->   isChecked()) interaction_mode = 3;
      //cloud_poly_points = cloud_union(cloud_poly_points,cloud_clicked_point,3);//adds only single point selected
      cloud_poly_points = cloud_union(cloud_poly_points,g_hover_mouse_area_,interaction_mode);//use area instead of point to handle round edges select
      //make ring instead of slid circle for interactive_mode union and invert, slightly faster processing for polygonmesh and crophull
      if(interaction_mode==0 || interaction_mode==3)
      {
        if(radius>=1)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inner_ring (new pcl::PointCloud<pcl::PointXYZ>);
          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          //append to cloud_line_area
          if ( g_kdtree_cloud_open_.radiusSearch (point,radius-0.2, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
          {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            {
              point.x = g_cloud_opened_->points[ pointIdxRadiusSearch[i] ].x;
              point.y = g_cloud_opened_->points[ pointIdxRadiusSearch[i] ].y;
              point.z = g_cloud_opened_->points[ pointIdxRadiusSearch[i] ].z;
              cloud_inner_ring->push_back(point);
            }
          }
          cloud_poly_points = cloud_union(cloud_poly_points,cloud_inner_ring,1);//subtract inner circle to make ring
        }
      }
      //======== END  append hover_points to cloud_poly_points

      //========START create polygonmesh and find points within polygonmesh using CropHull
      //reset view
      viewer->removePointCloud ("cloud_poly_area");
      viewer->removeShape      ("polygonmesh_line");
      viewer->removePointCloud ("polygonmesh_area");
      if(cloud_poly_points->points.size()>1) //process only if 2 or more points
      {
        if(cloud_poly_points->points.size()<=1)//show polygon lines when area is less than 3 points, cant craete hull with these points
        {
          viewer->addPolygon<pcl::PointXYZ>(cloud_poly_points,0.0f,1.0f,0.0f,"polygonmesh_line");
        }
        //constructing convex hull with polygon
        else// if(cloud_line_points->points.size()>3)//process only with 4 or more points to create hull
        {
          std::clock_t start;
          double duration;
          //create envelope points of cloud_line_points for creating mesh, find point normals and create 2 more points in +-z offsets
          start = std::clock();
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_envelope (new pcl::PointCloud<pcl::PointXYZ>);
          for(int i=0;i<cloud_poly_points->points.size();i++)
          {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            //append to cloud_line_area
            if ( g_kdtree_cloud_open_.radiusSearch (cloud_poly_points->points[i],30, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
              float curvature,offset_scale = 1;//1mm offset of envelope from selected points
              Eigen::Vector4f plane_param;
              pcl::computePointNormal(*g_cloud_opened_,pointIdxRadiusSearch,plane_param,curvature);
              point.x = cloud_poly_points->points[i].x + plane_param[0]*offset_scale;
              point.y = cloud_poly_points->points[i].y + plane_param[1]*offset_scale;
              point.z = cloud_poly_points->points[i].z + plane_param[2]*offset_scale;
              cloud_envelope->push_back(point);
              point.x = cloud_poly_points->points[i].x - plane_param[0]*offset_scale;
              point.y = cloud_poly_points->points[i].y - plane_param[1]*offset_scale;
              point.z = cloud_poly_points->points[i].z - plane_param[2]*offset_scale;
              cloud_envelope->push_back(point);
            }
          }
          duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
          std::cout << "creating envelope took " << duration << "s."<< std::endl;
          // Create Tris from Box Vertices
          std::vector<pcl::Vertices> indices;
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
          /*pcl::ConcaveHull<pcl::PointXYZ> cHull;//concave hull library segment fault, may be viable if more updated pcl version used
          cHull.setDimension(2);
          std::cout << "Passed setDimension" << std::endl;
          cHull.setAlpha(1);
          std::cout << "Passed setAlpha" << std::endl;
          cHull.setInputCloud(cloud_line_points);
          std::cout << "Passed setInputCloud" << std::endl;
          cHull.setKeepInformation(true);
          std::cout << "Passed setKeepInformation" << std::endl;
          cHull.reconstruct(*cloud_hull, indices);
          std::cout << "Passed reconstruct" << std::endl;*/
          pcl::ConvexHull<pcl::PointXYZ> cHull;
          cHull.setDimension(3);
          cHull.setInputCloud(cloud_envelope);
          cHull.reconstruct(*cloud_hull,indices);
          // Create Crop Hull
          pcl::CropHull<pcl::PointXYZ> cropHull;
          //cropHull.setHullCloud(cloud_line_points);
          cropHull.setHullCloud(cloud_hull);
          cropHull.setHullIndices(indices);
          cropHull.setDim(3);
          cropHull.setInputCloud(g_cloud_opened_);
          // Extract Indices to be Removed
          std::vector<int> resultIndices;
          start = std::clock();
          cropHull.filter(resultIndices);
          duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
          std::cout << "cropHull filter took " << duration << "s."<< std::endl;
          // Copy Indices to another Data Structure for pcl::ExtractIndices
          pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
          pointIndices->indices.insert(pointIndices->indices.end(), resultIndices.begin(), resultIndices.end());
          // Extract the Points inside the CropHull
          pcl::ExtractIndices<pcl::PointXYZ> extract(true);
          extract.setInputCloud(g_cloud_opened_);
          extract.setIndices(pointIndices);
          extract.setNegative(false);
          cloud_poly_area->clear();
          extract.filter(*cloud_poly_area);
          if(cloud_poly_area->points.size()>1)
          {
            //creates the polymesh of hull
            pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2(*cloud_hull,*cloud_blob);
            pcl::PolygonMesh mesh;
            mesh.polygons = indices;
            mesh.cloud = *cloud_blob;
            //cloud_poly_area pulled from CropHull, displayed in cyan
            viewer->addPointCloud (cloud_poly_area,"cloud_poly_area");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "cloud_poly_area");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud_poly_area");
            //lines of polygonmesh in cloud_hull, displayed in green
            viewer->addPolylineFromPolygonMesh(mesh,"polygonmesh_line");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "polygonmesh_line");
            //mesh of polygonmesh in cloud_hull, displayed in purple translucent
            viewer->addPolygonMesh(mesh,"polygonmesh_area");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "polygonmesh_area");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "polygonmesh_area");
          }
        }
      }
      //selected points to be displayed in red
      viewer->removePointCloud ("cloud_poly_points");
      viewer->addPointCloud (cloud_poly_points, "cloud_poly_points");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "cloud_poly_points");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "cloud_poly_points");
      ui.qvtkWidget->update ();
      //======== END  create polygonmesh and find points within polygonmesh using CropHull
    }
  }

  if(event_name == "Circle")
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clicked_point (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_clicked_point->push_back(point);
    //highlght around radius from cloud_open
    float radius = ui.doubleSpinBox_selection_radius->value(); //mm scale
    if (radius>0 && g_cloud_opened_->empty()==0)
    {
      int interaction_mode = 0;
      if(ui.radioButton_interaction_union->    isChecked()) interaction_mode = 0;
      if(ui.radioButton_interaction_subtract-> isChecked()) interaction_mode = 1;
      if(ui.radioButton_interaction_intersect->isChecked()) interaction_mode = 2;
      if(ui.radioButton_interaction_invert->   isChecked()) interaction_mode = 3;
      //cloud_circle_area = cloud_union(cloud_circle_area,cloud_clicked_point,interaction_mode);
      cloud_circle_area = cloud_union(cloud_circle_area,g_hover_mouse_area_,interaction_mode); //use
      //Visualize cloud_circle_area in blue
      viewer->removePointCloud ("cloud_circle_area");
      viewer->addPointCloud (cloud_circle_area, "cloud_circle_area");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "cloud_circle_area");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud_circle_area");
      ui.qvtkWidget->update ();
    }
  }

  if(event_name == "Cluster")
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      std::string file_name;
      std::string cluster_name;
      file_name = "data/coupons/clusters/cluster_"+std::to_string(ui.spinBox_IM_cluster_no->value())+".pcd";
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud_cluster) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read cluster .pcd file \n");
        exit(-1);
      }

      std::cout << "adding cluster no. " << ui.spinBox_IM_cluster_no->value() << " to cloud_cluster_area" << std::endl;
      int interaction_mode = 0;
      if(ui.radioButton_interaction_union->    isChecked()) interaction_mode = 0;
      if(ui.radioButton_interaction_subtract-> isChecked()) interaction_mode = 1;
      if(ui.radioButton_interaction_intersect->isChecked()) interaction_mode = 2;
      if(ui.radioButton_interaction_invert->   isChecked()) interaction_mode = 3;
      cloud_cluster_area = cloud_union(cloud_cluster_area,cloud_cluster,interaction_mode); //use

      //Visualize cloud_circle_area in darkGreen
      viewer->removePointCloud ("cloud_cluster_area");
      viewer->addPointCloud (cloud_cluster_area, "cloud_cluster_area");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.5f, 0.0f, "cloud_cluster_area");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud_cluster_area");
  }
  if(event_name == "None")
  {
    viewer->removePointCloud ("cloud_circle_area");
    viewer->removePointCloud ("cloud_poly_points");
    viewer->removeShape      ("polygonmesh_line");
    viewer->removePointCloud ("polygonmesh_area");
    viewer->removePointCloud ("cloud_poly_area");
    viewer->removePointCloud ("cloud_cluster_area");
    cloud_circle_area->clear();
    cloud_poly_area->clear();
    cloud_poly_points->clear();
    cloud_cluster_area->clear();
    ui.qvtkWidget->update ();
  }
}

bool ATG_Window::draw_cluster(int mode,int draw_cluster_no)//mode 0=append, 1=subtract, 2=makeNEW
{
  std::cout << "Calculating clusters information." << std::endl;
  if (cloud_poly_area->points.empty() && cloud_circle_area->points.empty() && cloud_cluster_area->points.empty())
  {
    log(Info, std::string("No seleted points drawn, no creation/modification made to clusters."));
    std::cout << "No seleted points drawn, no creation/modification made to clusters." <<std::endl;
    return 0;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  std::string cluster_name;
  std::string src_filename = "data/coupons/clusters/cluster_"+std::to_string(draw_cluster_no)+".pcd";
  if(mode==0 || mode==1)//mod to existing cluster
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (src_filename, *cloud_cluster) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read cluster .pcd file \n");
      //exit(-1);
    }
    cloud_cluster = cloud_union(cloud_cluster,cloud_poly_area,mode); //mode 0 append, 1 subtract
    cloud_cluster = cloud_union(cloud_cluster,cloud_circle_area,mode);
    cloud_cluster = cloud_union(cloud_cluster,cloud_cluster_area,mode);
    if (cloud_cluster->size()<5) //error cluster subtract till no more left, ignore process
    {
        std::cout << "Error: cluster_" << draw_cluster_no << " will become empty with subtract, process ignored." << std::endl;
        log(Info, std::string("Error subtracting points from cluster....."));
        return(0);
    }
  }
  else if(mode==2)//create new cluster
  {
    cloud_cluster = cloud_union(cloud_cluster,cloud_poly_area,0); //mode 0 append, 1 subtract
    cloud_cluster = cloud_union(cloud_cluster,cloud_circle_area,0);
    cloud_cluster = cloud_union(cloud_cluster,cloud_cluster_area,0);
    //add new centroid
    //draw_cluster_no is also equals to int sizeplus1 = coupon.clusters_centriod.size()+1;
    cluster_centroids_.resize(draw_cluster_no+1);
    Eigen::Vector4f temp_centroid;
    compute3DCentroid(*cloud_cluster, temp_centroid);
    cluster_centroids_[draw_cluster_no].x = temp_centroid[0];
    cluster_centroids_[draw_cluster_no].y = temp_centroid[1];
    cluster_centroids_[draw_cluster_no].z = temp_centroid[2];
  }
  recalculate_cluster(cloud_cluster,draw_cluster_no,ui.spinBox_KSearch->value(),g_cloud_opened_centroid);
  std::cout << "Loading: cluster_ " << draw_cluster_no << std::endl;
  cluster_name ="clustersviz_"+std::to_string(draw_cluster_no);
  log(Info, std::string("loading cluster point cloud data....."));

  viewer->removePointCloud(cluster_name);
  viewer->addPointCloud (cloud_cluster, cluster_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 1.0f, cluster_name);
  ui.qvtkWidget->update ();
  log(Info,std::string("Point cloud data segemenation results are shown"));

  viewer->removeText3D("N"+std::to_string(draw_cluster_no));
  viewer->addText3D("No."+std::to_string(draw_cluster_no), cluster_centroids_[draw_cluster_no], 3, 1, 1, 0,"N"+std::to_string(draw_cluster_no)); //josh edited Bold text
  return(1);
}

void ATG_Window::recalculate_cluster(PointCloud<PointXYZ>::Ptr cloud_input, int cluster_no_r, int k_Search, pcl::PointXYZ centroid)
{
  //josh, this method might be wrong as the normals calculated are with no regards with the full point cloud.
  //compute normal and region growing to get clusters
  std::string src_filename = "data/coupons/clusters/cluster_"+std::to_string(cluster_no_r)+".pcd";
  //emit progressUpdated(30);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator; //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//original normalestimator function
  normal_estimator.setNumberOfThreads (12);//new for OMP function, speeds up calculation
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_input);
  normal_estimator.setKSearch (k_Search);
  normal_estimator.setViewPoint (centroid.x, centroid.y, centroid.z);
  normal_estimator.compute (*normals);
  //emit progressUpdated(50);

  //emit progressUpdated(80);
  pcl::PointCloud<pcl::PointNormal>::Ptr cluster(new pcl::PointCloud<pcl::PointNormal>);
  for (int j = 0; j < normals->size(); j++)
  {
    pcl::PointNormal p;
    p.x = cloud_input->points[j].x;
    p.y = cloud_input->points[j].y;
    p.z = cloud_input->points[j].z;
    p.normal_x = normals->points[j].normal_x;
    p.normal_y = normals->points[j].normal_y;
    p.normal_z = normals->points[j].normal_z;
    p.curvature = normals->points[j].curvature;
    cluster->points.push_back(p);
  }
  std::cout << "cluster " + std::to_string(cluster_no_r) + " has " << cluster->points.size() << " points." << std::endl;
  cluster->width = cluster->points.size();
  cluster->height = 1;
  if(!cluster->empty())
  {
    pcl::io::savePCDFile(src_filename, *cluster);
  }

  //emit progressUpdated(100);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ATG_Window::cloud_union(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_ori, int mode)
{
  PointCloud<pcl::PointXYZ>::Ptr cloud_new  (new pcl::PointCloud<pcl::PointXYZ>),
                                 cloud_base (new pcl::PointCloud<pcl::PointXYZ>);//to retain cloud_new_ori's data, if not modifications here will clear up all data
  pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud_new_ori, *cloud_new);
  pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud_base_ori, *cloud_base);
  //for( it= cloud->begin(); it!= cloud->end(); it++)
  //mode = 0 (union), 1 (subtract), 2 (intersect), 3 (invert)
  if(cloud_new->points.empty()==0)//if cloud_append not empty
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_overlap (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new_overlap  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud_clicked;
    pcl::PointIndices::Ptr inliers_new(new pcl::PointIndices());
    pcl::PointIndices::Ptr inliers_base(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    if(cloud_base->points.empty()==0)//if cloud_base not empty, check for duplicate and record in inliers
    {
      kdtree_cloud_clicked.setInputCloud(cloud_base);
      for(int i = 0; i<cloud_new->size();i++)
      {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree_cloud_clicked.nearestKSearch (cloud_new->points[i], 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
          if(pointRadiusSquaredDistance[0] == 0)
          {
            inliers_new->indices.push_back(i);
            inliers_base->indices.push_back(pointIdxRadiusSearch[0]);
          }
        }
      }
      std::cout << "cloud_new size = " << cloud_new->size() << std::endl;
      std::cout << "inliers size = " << inliers_new->indices.size() << std::endl;
      extract.setInputCloud(cloud_new);       //   -----------         -----------         -----------        -----------        -----------
      extract.setIndices(inliers_new);        //  |           |       |///////////|       |///////////|      |           |      |///////////|
      extract.setNegative(false);             //  | cloud_base|       |///////////|       |///////////|      |           |      |///////////|
      extract.filter(*cloud_new_overlap);     //  |     ------|----   |/////------|----   |/////------|----  |     ------|----  |/////------|----
      extract.setNegative(true);              //  |    |over  |    |  |////|//////|////|  |////|      |    | |    |//////|    | |////|      |////|
      extract.filter(*cloud_new);             //  |    |  lap |    |  |////|//////|////|  |////|      |    | |    |//////|    | |////|      |////|
      extract.setInputCloud(cloud_base);      //   ----|------     |   ----|------/////|   ----|------     |  ----|------     |  ----|------/////|
      extract.setIndices(inliers_base);       //       | cloud_new |       |///////////|       |           |      |           |      |///////////|
      extract.setNegative(false);             //       |           |       |///////////|       |           |      |           |      |///////////|
      extract.filter(*cloud_base_overlap);    //        -----------         -----------         -----------        -----------        -----------
      extract.setNegative(true);              //                             union              subtract         intersect            invert
      extract.filter(*cloud_base);            //
    }
    //std::cout << "cloud_new after filter size = " << cloud_new->size() << std::endl;
    if(mode==0) {*cloud_base += *cloud_base_overlap; *cloud_base += *cloud_new;}
    if(mode==1) {/*cloud_base as is filtered*/}
    if(mode==2) {*cloud_base = *cloud_base_overlap;}
    if(mode==3) {*cloud_base += *cloud_new;}
    //std::cout << "cloud_base size = " << cloud_base->size() << std::endl;
  }
  return (cloud_base);
}


}//close namespace atg_gui
