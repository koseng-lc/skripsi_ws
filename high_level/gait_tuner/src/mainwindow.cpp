#include "gait_tuner/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : spin_thread_(boost::thread{boost::bind(&MainWindow::spinThread, this)})
    , QMainWindow(parent)
    , ui(new Ui::MainWindow){

    ui->setupUi(this);

    for(const auto id:JointData::ALL_ID){
        joint_angle_data_.name.push_back(JointData::ID2NAME[id]);
        joint_angle_data_.position.push_back(.0);
    }

    std::stringstream style_path;
    style_path << ros::package::getPath("utils") << "/gui/dark_style_lintang.qss";

    QFile style_file(tr(style_path.str().c_str()));
    style_file.open(QFile::ReadOnly);
    QString style_sheet(style_file.readAll());

    this->setStyleSheet(style_sheet);
    this->setWindowTitle(tr("Walk Tuner"));

    settingWidgets();
    settingActions();

    loadGaitEngineConfig();

}

MainWindow::~MainWindow(){
//    ros::shutdown();
    nh_.shutdown();
    boost::mutex::scoped_lock lock(spin_mtx_);
    spin_cv_.wait(lock);
    spin_thread_.join();
    delete ui;
}

void MainWindow::spinThread(){
    nh_.setCallbackQueue(&callback_queue_);

    viz_states_sub_ = nh_.subscribe("/viz/states", 1, &MainWindow::vizStatesCb, this);
    g_eng_data_sub_ = nh_.subscribe("/walking/gait_engine/data", 1, &MainWindow::gaitEngineDataCb, this);

    active_motion_pub_ = nh_.advertise<high_level_msgs::ActiveMotion >("/motion/active_motion", 1);
    walk_cmd_pub_ = nh_.advertise<high_level_msgs::WalkCmd >("/walking/cmd", 1);
    walk_input_pub_ = nh_.advertise<high_level_msgs::WalkInput >("/walking/input", 1);
    joint_angle_pub_ = nh_.advertise<sensor_msgs::JointState >("/joint_control/data", 1);

    ros::WallDuration cb_delay(0.008);

    while(nh_.ok())
        callback_queue_.callAvailable(cb_delay);

    spin_cv_.notify_one();
}

void MainWindow::vizStatesCb(const high_level_msgs::VizStatesConstPtr &_msg){

    viz_states_ = *_msg;

    emit updateViz();
}

void MainWindow::gaitEngineDataCb(const high_level_msgs::GaitEngineDataConstPtr &_msg){

    // dereferencing
    g_eng_data_ = *_msg;

    emit updatePlot();
}

void MainWindow::settingWidgets(){

    ROS_INFO("Setting up widget...");

    main_layout_ = new QGridLayout;
    main_widget_ = new QWidget;

    joint_driver_cb_ = new QComboBox;
    joint_driver_cb_->addItem(tr("RIGHT_HAND"));
    joint_driver_cb_->addItem(tr("LEFT_HAND"));
    joint_driver_cb_->addItem(tr("RIGHT_LEG"));
    joint_driver_cb_->addItem(tr("LEFT_LEG"));
    joint_driver_cb_->setMaximumSize(joint_driver_cb_->minimumSizeHint());

    for(const auto id:JointData::ALL_ID){
        joint_driver_label_[id] = new QLabel;
        std::stringstream ss;
        ss << JointData::ID2NAME[id].c_str() << " : %1";
        joint_driver_label_[id]->setText(tr(ss.str().c_str()).arg(QString::number(0.0,'f', 2)));
        joint_driver_sld_[id] = new QSlider;
        joint_driver_sld_[id]->setValue(0);
        joint_driver_sld_[id]->setMinimum(0);
        joint_driver_sld_[id]->setMaximum(360);
        joint_driver_sld_[id]->setOrientation(Qt::Horizontal);
    }

    r_hand_gl_ = new QGridLayout;

    r_hand_gl_->addWidget(joint_driver_label_[JointData::R_SHO_PITCH],0,0,1,1);
    r_hand_gl_->addWidget(joint_driver_sld_[JointData::R_SHO_PITCH]  ,1,0,1,2);
    r_hand_gl_->addWidget(joint_driver_label_[JointData::R_SHO_ROLL] ,2,0,1,1);
    r_hand_gl_->addWidget(joint_driver_sld_[JointData::R_SHO_ROLL]   ,3,0,1,2);
    r_hand_gl_->addWidget(joint_driver_label_[JointData::R_ELBOW]    ,4,0,1,1);
    r_hand_gl_->addWidget(joint_driver_sld_[JointData::R_ELBOW]      ,5,0,1,2);
    r_hand_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),6,2);

    r_hand_wid_ = new QWidget;
    r_hand_wid_->setLayout(r_hand_gl_);

    //=================================

    l_hand_gl_ = new QGridLayout;

    l_hand_gl_->addWidget(joint_driver_label_[JointData::L_SHO_PITCH],0,0,1,1);
    l_hand_gl_->addWidget(joint_driver_sld_[JointData::L_SHO_PITCH]  ,1,0,1,2);
    l_hand_gl_->addWidget(joint_driver_label_[JointData::L_SHO_ROLL] ,2,0,1,1);
    l_hand_gl_->addWidget(joint_driver_sld_[JointData::L_SHO_ROLL]   ,3,0,1,2);
    l_hand_gl_->addWidget(joint_driver_label_[JointData::L_ELBOW]    ,4,0,1,1);
    l_hand_gl_->addWidget(joint_driver_sld_[JointData::L_ELBOW]      ,5,0,1,2);
    l_hand_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),6,2);

    l_hand_wid_ = new QWidget;
    l_hand_wid_->setLayout(l_hand_gl_);

    //=================================

    r_leg_gl_ = new QGridLayout;

    r_leg_gl_->addWidget(joint_driver_label_[JointData::R_HIP_YAW],  0,0,1,1);
    r_leg_gl_->addWidget(joint_driver_sld_[JointData::R_HIP_YAW],    1,0,1,2);
    r_leg_gl_->addWidget(joint_driver_label_[JointData::R_HIP_ROLL], 2,0,1,1);
    r_leg_gl_->addWidget(joint_driver_sld_[JointData::R_HIP_ROLL],   3,0,1,2);
    r_leg_gl_->addWidget(joint_driver_label_[JointData::R_HIP_PITCH],4,0,1,1);
    r_leg_gl_->addWidget(joint_driver_sld_[JointData::R_HIP_PITCH],  5,0,1,2);
    r_leg_gl_->addWidget(joint_driver_label_[JointData::R_KNEE],     6,0,1,1);
    r_leg_gl_->addWidget(joint_driver_sld_[JointData::R_KNEE],       7,0,1,2);
    r_leg_gl_->addWidget(joint_driver_label_[JointData::R_ANK_PITCH],8,0,1,1);
    r_leg_gl_->addWidget(joint_driver_sld_[JointData::R_ANK_PITCH],  9,0,1,2);
    r_leg_gl_->addWidget(joint_driver_label_[JointData::R_ANK_ROLL], 10,0,1,1);
    r_leg_gl_->addWidget(joint_driver_sld_[JointData::R_ANK_ROLL],   11,0,1,2);
    r_leg_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),12,2);

    r_leg_wid_ = new QWidget;
    r_leg_wid_->setLayout(r_leg_gl_);

    //=================================

    l_leg_gl_ = new QGridLayout;

    l_leg_gl_->addWidget(joint_driver_label_[JointData::L_HIP_YAW],  0,0,1,1);
    l_leg_gl_->addWidget(joint_driver_sld_[JointData::L_HIP_YAW],    1,0,1,2);
    l_leg_gl_->addWidget(joint_driver_label_[JointData::L_HIP_ROLL], 2,0,1,1);
    l_leg_gl_->addWidget(joint_driver_sld_[JointData::L_HIP_ROLL],   3,0,1,2);
    l_leg_gl_->addWidget(joint_driver_label_[JointData::L_HIP_PITCH],4,0,1,1);
    l_leg_gl_->addWidget(joint_driver_sld_[JointData::L_HIP_PITCH],  5,0,1,2);
    l_leg_gl_->addWidget(joint_driver_label_[JointData::L_KNEE],     6,0,1,1);
    l_leg_gl_->addWidget(joint_driver_sld_[JointData::L_KNEE],       7,0,1,2);
    l_leg_gl_->addWidget(joint_driver_label_[JointData::L_ANK_PITCH],8,0,1,1);
    l_leg_gl_->addWidget(joint_driver_sld_[JointData::L_ANK_PITCH],  9,0,1,2);
    l_leg_gl_->addWidget(joint_driver_label_[JointData::L_ANK_ROLL], 10,0,1,1);
    l_leg_gl_->addWidget(joint_driver_sld_[JointData::L_ANK_ROLL],   11,0,1,2);
    l_leg_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),12,2);

    l_leg_wid_ = new QWidget;
    l_leg_wid_->setLayout(l_leg_gl_);

    //======

    joint_driver_sl_ = new QStackedLayout;
    joint_driver_sl_->addWidget(r_hand_wid_);
    joint_driver_sl_->addWidget(l_hand_wid_);
    joint_driver_sl_->addWidget(r_leg_wid_);
    joint_driver_sl_->addWidget(l_leg_wid_);

    joint_driver_gb_ = new QGroupBox;
    joint_driver_gb_->setTitle(tr("RIGHT_HAND"));
    joint_driver_gb_->setLayout(joint_driver_sl_);
//    joint_driver_gb_->setMaximumSize(joint_driver_sl_->sizeHint());

    joint_driver_gl_ = new QGridLayout;
    joint_driver_gl_->addWidget(joint_driver_cb_,0,0,1,1);
    joint_driver_gl_->addWidget(joint_driver_gb_,1,0,1,2);
    joint_driver_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),2,2);

    joint_driver_wid_ = new QWidget;
    joint_driver_wid_->setLayout(joint_driver_gl_);

    //-------------------------------------------------------------------------------------------------

    gait_vx_label_ = new QLabel;
    gait_vx_label_->setText(tr("Vx : "));
    gait_vx_dsb_ = new QDoubleSpinBox;
    gait_vx_dsb_->setMaximum(1.0);
    gait_vx_dsb_->setMinimum(.0);
    gait_vx_dsb_->setSingleStep(0.01);
    gait_vx_dsb_->setValue(.0);

    gait_vy_label_ = new QLabel;
    gait_vy_label_->setText(tr("Vy : "));
    gait_vy_dsb_ = new QDoubleSpinBox;
    gait_vy_dsb_->setMaximum(1.0);
    gait_vy_dsb_->setMinimum(.0);
    gait_vy_dsb_->setSingleStep(0.01);
    gait_vy_dsb_->setValue(.0);

    gait_vphi_label_ = new QLabel;
    gait_vphi_label_->setText(tr("Vphi : "));
    gait_vphi_dsb_ = new QDoubleSpinBox;
    gait_vphi_dsb_->setMaximum(1.0);
    gait_vphi_dsb_->setMinimum(.0);
    gait_vphi_dsb_->setSingleStep(0.01);
    gait_vphi_dsb_->setValue(.0);

    gait_input_gl_ = new QGridLayout;
    gait_input_gl_->addWidget(gait_vx_label_,   0,0,1,1);
    gait_input_gl_->addWidget(gait_vx_dsb_,     0,1,1,1);
    gait_input_gl_->addWidget(gait_vy_label_,   1,0,1,1);
    gait_input_gl_->addWidget(gait_vy_dsb_,     1,1,1,1);
    gait_input_gl_->addWidget(gait_vphi_label_, 2,0,1,1);
    gait_input_gl_->addWidget(gait_vphi_dsb_,   2,1,1,1);
    gait_input_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),3,2);

    gait_input_gb_ = new QGroupBox;
    gait_input_gb_->setTitle(tr("Input"));
    gait_input_gb_->setLayout(gait_input_gl_);
    gait_input_gb_->setMaximumHeight(gait_input_gl_->sizeHint().height());

    gait_start_pb_ = new QPushButton;
    gait_start_pb_->setText(tr("Start"));

    gait_stop_pb_ = new QPushButton;
    gait_stop_pb_->setText(tr("Stop"));

    gait_set_pb_ = new QPushButton;
    gait_set_pb_->setText(tr("Set"));

    gait_params_pb_ = new QPushButton;
    gait_params_pb_->setText(tr("Params"));

    gait_gl_ = new QGridLayout;
    gait_gl_->addWidget(gait_input_gb_,0,0,1,2);
    gait_gl_->addWidget(gait_start_pb_,1,0,1,1);
    gait_gl_->addWidget(gait_set_pb_,1,1,1,1);
    gait_gl_->addWidget(gait_stop_pb_,2,0,1,1);
    gait_gl_->addWidget(gait_params_pb_,2,1,1,1);
    gait_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored, QSizePolicy::Expanding),3,2);

    gait_widget_ = new QWidget;
    gait_widget_->setLayout(gait_gl_);

    //-------------------------------------------------------------------------------------------------

    walk_params_dialog_ = new QDialog(this);
    walk_params_dialog_->setFixedSize(640,480);
    walk_params_dialog_->setWindowTitle(tr("Walk Params"));
    walk_params_dialog_->hide();

    for(int i = 0; i < 29; i++){
        walk_config_label_[i] = new QLabel;
        walk_config_dsb_[i] = new QDoubleSpinBox;
        walk_config_dsb_[i]->setMinimum(-10.0);
        walk_config_dsb_[i]->setMaximum(10.0);
        walk_config_dsb_[i]->setSingleStep(.0001);
        walk_config_dsb_[i]->setDecimals(4);
    }

    //--- Halt
    walk_config_label_[C1]->setText(tr("C1 : "));
    walk_config_label_[C2]->setText(tr("C2 : "));
    walk_config_label_[C3]->setText(tr("C3 : "));
    walk_config_label_[C4]->setText(tr("C4 : "));
    walk_config_label_[C5]->setText(tr("C5 : "));
    wp_halt_gl_ = new QGridLayout;
    wp_halt_gl_->addWidget(walk_config_label_[C1],0,0,1,1);
    wp_halt_gl_->addWidget(walk_config_dsb_[C1],0,1,1,1);
    wp_halt_gl_->addWidget(walk_config_label_[C2],1,0,1,1);
    wp_halt_gl_->addWidget(walk_config_dsb_[C2],1,1,1,1);
    wp_halt_gl_->addWidget(walk_config_label_[C3],2,0,1,1);
    wp_halt_gl_->addWidget(walk_config_dsb_[C3],2,1,1,1);
    wp_halt_gl_->addWidget(walk_config_label_[C4],3,0,1,1);
    wp_halt_gl_->addWidget(walk_config_dsb_[C4],3,1,1,1);
    wp_halt_gl_->addWidget(walk_config_label_[C5],4,0,1,1);
    wp_halt_gl_->addWidget(walk_config_dsb_[C5],4,1,1,1);
    wp_halt_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),5,2);
    wp_halt_gb_ = new QGroupBox;
    wp_halt_gb_->setTitle(tr("Halt"));
    wp_halt_gb_->setLayout(wp_halt_gl_);

    //--- Leg Lifting
    walk_config_label_[C6]->setText(tr("C6 : "));
    walk_config_label_[C7]->setText(tr("C7 : "));
    walk_config_label_[C8]->setText(tr("C8 : "));
    walk_config_label_[C9]->setText(tr("C9 : "));
    wp_leg_lifting_gl_ = new QGridLayout;
    wp_leg_lifting_gl_->addWidget(walk_config_label_[C6],0,0,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_dsb_[C6],0,1,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_label_[C7],1,0,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_dsb_[C7],1,1,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_label_[C8],2,0,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_dsb_[C8],2,1,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_label_[C9],3,0,1,1);
    wp_leg_lifting_gl_->addWidget(walk_config_dsb_[C9],3,1,1,1);
    wp_leg_lifting_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),4,2);
    wp_leg_lifting_gb_ = new QGroupBox;
    wp_leg_lifting_gb_->setTitle(tr("Leg Lifting"));
    wp_leg_lifting_gb_->setLayout(wp_leg_lifting_gl_);

    //--- Leg Swing
    walk_config_label_[C_TAU_0]->setText(tr("C_TAU_0 : "));
    walk_config_label_[C_TAU_1]->setText(tr("C_TAU_1 : "));
    walk_config_label_[C10]->setText(tr("C10 : "));
    walk_config_label_[C11]->setText(tr("C11 : "));
    walk_config_label_[C12]->setText(tr("C12 : "));
    walk_config_label_[C13]->setText(tr("C13 : "));
    walk_config_label_[C14]->setText(tr("C14 : "));
    walk_config_label_[C15]->setText(tr("C15 : "));
    walk_config_label_[C16]->setText(tr("C16 : "));
    wp_leg_swing_gl_ = new QGridLayout;
    wp_leg_swing_gl_->addWidget(walk_config_label_[C_TAU_0],0,0,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C_TAU_0],0,1,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C_TAU_1],1,0,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C_TAU_1],1,1,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C10],2,0,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C10],2,1,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C11],3,0,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C11],3,1,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C12],4,0,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C12],4,1,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C13],0,2,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C13],0,3,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C14],1,2,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C14],1,3,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C15],2,2,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C15],2,3,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_label_[C16],3,2,1,1);
    wp_leg_swing_gl_->addWidget(walk_config_dsb_[C16],3,3,1,1);
    wp_leg_swing_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),5,4);
    wp_leg_swing_gb_ = new QGroupBox;
    wp_leg_swing_gb_->setTitle(tr("Leg Swing"));
    wp_leg_swing_gb_->setLayout(wp_leg_swing_gl_);

    //--- Lateral Hip Swing
    walk_config_label_[C17]->setText(tr("C17 : "));
    wp_lat_hip_swing_gl_ = new QGridLayout;
    wp_lat_hip_swing_gl_->addWidget(walk_config_label_[C17],0,0,1,1);
    wp_lat_hip_swing_gl_->addWidget(walk_config_dsb_[C17],0,1,1,1);
    wp_lat_hip_swing_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),1,2);
    wp_lat_hip_swing_gb_ = new QGroupBox;
    wp_lat_hip_swing_gb_->setTitle(tr("Lateral Hip Swing"));
    wp_lat_hip_swing_gb_->setLayout(wp_lat_hip_swing_gl_);

    //--- Leaning
    walk_config_label_[C18]->setText(tr("C18 : "));
    walk_config_label_[C19]->setText(tr("C19 : "));
    walk_config_label_[C20]->setText(tr("C20 : "));
    wp_leaning_gl_ = new QGridLayout;
    wp_leaning_gl_->addWidget(walk_config_label_[C18],0,0,1,1);
    wp_leaning_gl_->addWidget(walk_config_dsb_[C18],0,1,1,1);
    wp_leaning_gl_->addWidget(walk_config_label_[C19],1,0,1,1);
    wp_leaning_gl_->addWidget(walk_config_dsb_[C19],1,1,1,1);
    wp_leaning_gl_->addWidget(walk_config_label_[C20],2,0,1,1);
    wp_leaning_gl_->addWidget(walk_config_dsb_[C20],2,1,1,1);
    wp_leaning_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),3,2);
    wp_leaning_gb_ = new QGroupBox;
    wp_leaning_gb_->setTitle(tr("Leaning"));
    wp_leaning_gb_->setLayout(wp_leaning_gl_);

    //--- Control Interface config
    walk_config_label_[C21]->setText(tr("C21 : "));
    walk_config_label_[C22]->setText(tr("C22 : "));
    walk_config_label_[C23]->setText(tr("C23 : "));
    walk_config_label_[C24]->setText(tr("C24 : "));
    walk_config_label_[C25]->setText(tr("C25 : "));
    walk_config_label_[C26]->setText(tr("C26 : "));
    walk_config_label_[C27]->setText(tr("C27 : "));
    wp_ctrl_interface_gl_ = new QGridLayout;
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C21],0,0,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C21],0,1,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C22],1,0,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C22],1,1,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C23],2,0,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C23],2,1,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C24],3,0,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C24],3,1,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C25],4,0,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C25],4,1,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C26],0,2,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C26],0,3,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_label_[C27],1,2,1,1);
    wp_ctrl_interface_gl_->addWidget(walk_config_dsb_[C27],1,3,1,1);
    wp_ctrl_interface_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),5,4);
    wp_ctrl_interface_gb_ = new QGroupBox;
    wp_ctrl_interface_gb_->setTitle(tr("Control Interface"));
    wp_ctrl_interface_gb_->setLayout(wp_ctrl_interface_gl_);

    //--- Walk Params Legend
    wp_legend_pb_ = new QPushButton;
    wp_legend_pb_->setText(tr("?"));

    wp_legend_le_ = new QTextEdit;
    wp_legend_le_->setReadOnly(true);
    wp_legend_le_->setStyleSheet(tr("QTextEdit{"
                                    "   background:black;"
                                    "   color:rgb(255,0,175);"
                                    "   font:italic \"Consolas\" "
                                    "}"));
    wp_legend_le_->setText("C1 : Halt Position Leg Extension\n"
                           "C2 : Halt Position Leg Roll Angle\n"
                           "C3 : Halt Position Leg Pitch Angle\n"
                           "C4 : Halt Position Foot Roll Angle\n"
                           "C5 : Halt Position Foot Pitch Angle\n"
                           "C6 : Constant Ground Push\n"
                           "C7 : Proportional Ground Push\n"
                           "C8 : Constant Step Height\n"
                           "C9 : Proportional Step Height\n"
                           "C_TAU_0 : Swing Start Timing\n"
                           "C_TAU_1 : Swing Stop Timing\n"
                           "C10 : Sagittal Swing Amplitude Fwd\n"
                           "C11 : Sagittal Swing Amplitude Bwd\n"
                           "C12 : Lateral Swing Amplitude\n"
                           "C13 : Lateral Swing Amplitude Offset\n"
                           "C14 : Turning Lateral Swing Amplitude Offset\n"
                           "C15 : Rotational Swing Amplitude\n"
                           "C16 : Rotational Swing Amplitude Offset\n"
                           "C17 : Lateral Hip Swing Amplitude\n"
                           "C18 : Forward Lean\n"
                           "C19 : Backward Lean\n"
                           "C20 : Forward and Turning Lean\n"
                           "C21 : Gait Velocity Limiting Norm p\n"
                           "C22 : Sagittal Acceleration\n"
                           "C23 : Lateral Acceleration\n"
                           "C24 : Rotational Acceleration\n"
                           "C25 : Constant Step Frequency\n"
                           "C26 : Sagittal Proportional Step Frequency\n"
                           "C27 : Lateral Proportional Step Frequency");

    wp_legend_gl_ = new QGridLayout;
    wp_legend_gl_->addWidget(wp_legend_le_,0,0,1,1);

    wp_legend_dialog_ = new QDialog(this);    
    wp_legend_dialog_->setFixedSize(400,240);
    wp_legend_dialog_->setWindowTitle(tr("Walk Params Legend"));
    wp_legend_dialog_->setLayout(wp_legend_gl_);
    wp_legend_dialog_->hide();

    wp_gl_ = new QGridLayout;
    wp_gl_->addWidget(wp_halt_gb_,0,0,2,1);
    wp_gl_->addWidget(wp_leg_lifting_gb_,0,1,1,1);
    wp_gl_->addWidget(wp_legend_pb_,1,1,1,1);
    wp_gl_->addWidget(wp_leg_swing_gb_,0,2,2,1);
    wp_gl_->addWidget(wp_lat_hip_swing_gb_,2,0,1,2);
    wp_gl_->addWidget(wp_leaning_gb_,3,0,1,2);
    wp_gl_->addWidget(wp_ctrl_interface_gb_,2,2,2,1);
    wp_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),4,3);

    wp_widget_ = new QWidget;
    wp_widget_->setLayout(wp_gl_);

    //-------------------------------------------------------------------------------------------------

    mp_cb_ = new QComboBox;
    mp_cb_->addItem(tr("HALT"));
    mp_cb_->addItem(tr("LEG LIFTING"));
    mp_cb_->addItem(tr("LEG SWING"));
    mp_cb_->addItem(tr("LATERAL HIP SWING"));
    mp_cb_->addItem(tr("LEANING"));

    mp_legend_label_ = new QLabel;
    mp_legend_label_->setText(tr("Red : Left\tBlue : Right"));

    std::map<int, std::string > idx2signal_name{
        {P_HALT_LEG_EXT, "HALT LEG EXT."},
        {P_HALT_LEG_ROLL, "HALT LEG ROLL"},
        {P_HALT_LEG_PITCH, "HALT LEG PITCH"},
        {P_HALT_FOOT_ROLL, "HALT FOOT ROLL"},
        {P_HALT_FOOT_PITCH, "HALT FOOT PITCH"},
        {P_LIFT_LEG_EXT, "LEG LIFTING"},
        {P_SWING_LEG_PITCH, "SWING LEG PITCH"},
        {P_SWING_LEG_ROLL, "SWING LEG ROLL"},
        {P_SWING_LEG_YAW, "SWING LEG YAW"},
        {P_SWING_HIP_LATERAL, "LATERAL HIP SWING"},
        {P_LEAN_LEG_PITCH, "LEAN LEG PITCH"},
        {P_LEAN_LEG_ROLL, "LEAN LEG ROLL"}
    };

    for(int i(0); i < 12; i++){
        mp_signals_[i] = new SignalPlotter(0.5);
        mp_signals_[i]->setTitle(tr(idx2signal_name[i].c_str()));
        mp_signals_[i]->setMinimumSize(562,200);
        mp_signals_[i]->setMaximumSize(562,200);
        mp_signals_cview_[i] = new QChartView;
//        mp_signals_cview_[i]->setParent(this);
//        mp_signals_cview_[i]->setMinimumSize(320,240);
        mp_signals_cview_[i]->setChart(mp_signals_[i]);
        mp_signals_cview_[i]->setRenderHint(QPainter::Antialiasing);
//        mp_signals_cview_[i]->setFixedSize(mp_signals_cview_[i]->minimumSizeHint());
    }

    mp_type_gl_[HALT] = new QGridLayout;
    mp_type_gl_[HALT]->addWidget(mp_signals_cview_[P_HALT_LEG_EXT],0,0,1,1);
    mp_type_gl_[HALT]->addWidget(mp_signals_cview_[P_HALT_LEG_ROLL],1,0,1,1);
    mp_type_gl_[HALT]->addWidget(mp_signals_cview_[P_HALT_LEG_PITCH],2,0,1,1);
    mp_type_gl_[HALT]->addWidget(mp_signals_cview_[P_HALT_FOOT_ROLL],3,0,1,1);
    mp_type_gl_[HALT]->addWidget(mp_signals_cview_[P_HALT_FOOT_PITCH],4,0,1,1);
    mp_type_gl_[HALT]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),5,1);
//    mp_type_gl_[HALT]->setSizeConstraint(QLayout::SetMinAndMaxSize);
    mp_type_widget_[HALT] = new QWidget;
    mp_type_widget_[HALT]->setLayout(mp_type_gl_[HALT]);

    mp_type_gl_[LEG_LIFTING] = new QGridLayout;
    mp_type_gl_[LEG_LIFTING]->addWidget(mp_signals_cview_[P_LIFT_LEG_EXT],0,0,1,1);
    mp_type_gl_[LEG_LIFTING]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),1,1);
//    mp_type_gl_[LEG_LIFTING]->setSizeConstraint(QLayout::SetMinAndMaxSize);
    mp_type_widget_[LEG_LIFTING] = new QWidget;
    mp_type_widget_[LEG_LIFTING]->setLayout(mp_type_gl_[LEG_LIFTING]);

    mp_type_gl_[LEG_SWING] = new QGridLayout;
    mp_type_gl_[LEG_SWING]->addWidget(mp_signals_cview_[P_SWING_LEG_PITCH],0,0,1,1);
    mp_type_gl_[LEG_SWING]->addWidget(mp_signals_cview_[P_SWING_LEG_ROLL],1,0,1,1);
    mp_type_gl_[LEG_SWING]->addWidget(mp_signals_cview_[P_SWING_LEG_YAW],2,0,1,1);
    mp_type_gl_[LEG_SWING]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),3,1);
//    mp_type_gl_[LEG_SWING]->setSizeConstraint(QLayout::SetMinAndMaxSize);
    mp_type_widget_[LEG_SWING] = new QWidget;
    mp_type_widget_[LEG_SWING]->setLayout(mp_type_gl_[LEG_SWING]);

    mp_type_gl_[LATERAL_HIP_SWING] = new QGridLayout;
    mp_type_gl_[LATERAL_HIP_SWING]->addWidget(mp_signals_cview_[P_SWING_HIP_LATERAL],0,0,1,1);
    mp_type_gl_[LATERAL_HIP_SWING]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),1,1);
//    mp_type_gl_[LATERAL_HIP_SWING]->setSizeConstraint(QLayout::SetMinAndMaxSize);
    mp_type_widget_[LATERAL_HIP_SWING] = new QWidget;
    mp_type_widget_[LATERAL_HIP_SWING]->setLayout(mp_type_gl_[LATERAL_HIP_SWING]);

    mp_type_gl_[LEANING] = new QGridLayout;
    mp_type_gl_[LEANING]->addWidget(mp_signals_cview_[P_LEAN_LEG_PITCH],0,0,1,1);
    mp_type_gl_[LEANING]->addWidget(mp_signals_cview_[P_LEAN_LEG_ROLL],1,0,1,1);
    mp_type_gl_[LEANING]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),2,1);
//    mp_type_gl_[LEANING]->setSizeConstraint(QLayout::SetMinAndMaxSize);
    mp_type_widget_[LEANING] = new QWidget;
    mp_type_widget_[LEANING]->setLayout(mp_type_gl_[LEANING]);

    mp_type_sw_ = new QStackedWidget;
    mp_type_sw_->addWidget(mp_type_widget_[HALT]);
    mp_type_sw_->addWidget(mp_type_widget_[LEG_LIFTING]);
    mp_type_sw_->addWidget(mp_type_widget_[LEG_SWING]);
    mp_type_sw_->addWidget(mp_type_widget_[LATERAL_HIP_SWING]);
    mp_type_sw_->addWidget(mp_type_widget_[LEANING]);

    mp_sa_ = new QScrollArea;    
    mp_sa_->setBackgroundRole(QPalette::Dark);
    mp_sa_->setWidget(mp_type_sw_);
    mp_sa_->setWidgetResizable(true);
    mp_sa_->setMaximumHeight(mp_sa_->sizeHint().height());
//    mp_sa_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    mp_gl_ = new QGridLayout;
    mp_gl_->addWidget(mp_cb_,0,0,1,1);
    mp_gl_->addWidget(mp_legend_label_,0,1,1,1);
    mp_gl_->addWidget(mp_sa_,1,0,1,2);
    mp_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),2,2);

    mp_widget_ = new QWidget;
    mp_widget_->setLayout(mp_gl_);

    //-----

    il_cb_ = new QComboBox;
    il_cb_->addItem(tr("LEG EXT."));
    il_cb_->addItem(tr("LEG"));
    il_cb_->addItem(tr("FOOT"));

    il_legend_label_ = new QLabel;
    il_legend_label_->setText(tr("Red : Left\tBlue : Right"));

    std::map<int, std::string > idx2patt_name{
        {LEG_EXT, "LEG EXTENSION"},
        {YAW_LEG, "YAW LEG"},
        {ROLL_LEG, "ROLL LEG"},
        {PITCH_LEG, "PITCH LEG"},
        {PITCH_FOOT, "PITCH FOOT"},
        {ROLL_FOOT, "ROLL FOOT"}
    };

    for(int i(0); i < 6; i++){
        il_patt_[i] = new SignalPlotter(0.5);
        il_patt_[i]->setTitle(tr(idx2patt_name[i].c_str()));
        il_patt_[i]->setMinimumSize(562,200);
        il_patt_[i]->setMaximumSize(562,200);
        il_patt_cview_[i] = new QChartView;
        il_patt_cview_[i]->setChart(il_patt_[i]);
        il_patt_cview_[i]->setRenderHint(QPainter::Antialiasing);
    }

    il_patt_gl_[LEG_EXT] = new QGridLayout;
    il_patt_gl_[LEG_EXT]->addWidget(il_patt_cview_[LEG_EXT],0,0,1,1);
    il_patt_gl_[LEG_EXT]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),1,1);
    il_patt_widget_[LEG_EXT] = new QWidget;
    il_patt_widget_[LEG_EXT]->setLayout(il_patt_gl_[LEG_EXT]);

    il_patt_gl_[LEG] = new QGridLayout;
    il_patt_gl_[LEG]->addWidget(il_patt_cview_[YAW_LEG],0,0,1,1);
    il_patt_gl_[LEG]->addWidget(il_patt_cview_[ROLL_LEG],1,0,1,1);
    il_patt_gl_[LEG]->addWidget(il_patt_cview_[PITCH_LEG],2,0,1,1);
    il_patt_gl_[LEG]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),3,1);
    il_patt_widget_[LEG] = new QWidget;
    il_patt_widget_[LEG]->setLayout(il_patt_gl_[LEG]);

    il_patt_gl_[FOOT] = new QGridLayout;
    il_patt_gl_[FOOT]->addWidget(il_patt_cview_[PITCH_FOOT],0,0,1,1);
    il_patt_gl_[FOOT]->addWidget(il_patt_cview_[ROLL_FOOT],1,0,1,1);
    il_patt_gl_[FOOT]->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),2,1);
    il_patt_widget_[FOOT] = new QWidget;
    il_patt_widget_[FOOT]->setLayout(il_patt_gl_[FOOT]);

    il_patt_sw_ = new QStackedWidget;
    il_patt_sw_->addWidget(il_patt_widget_[LEG_EXT]);
    il_patt_sw_->addWidget(il_patt_widget_[LEG]);
    il_patt_sw_->addWidget(il_patt_widget_[FOOT]);

    il_sa_ = new QScrollArea;
    il_sa_->setBackgroundRole(QPalette::Dark);
    il_sa_->setWidget(il_patt_sw_);
    il_sa_->setWidgetResizable(true);    
    il_sa_->setMaximumHeight(il_sa_->sizeHint().height());
//    intuitiveLegTypeChange(0);
    il_cb_->setCurrentIndex(LEG);
    intuitiveLegTypeChange(LEG);

    il_gl_ = new QGridLayout;
    il_gl_->addWidget(il_cb_,0,0,1,1);
    il_gl_->addWidget(il_legend_label_,0,1,1,1);
    il_gl_->addWidget(il_sa_,1,0,1,2);
    il_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),2,2);

    il_widget_ = new QWidget;
    il_widget_->setLayout(il_gl_);

    //-----

    walk_params_tw_ = new QTabWidget;
    walk_params_tw_->addTab(wp_widget_, tr("Params"));
    walk_params_tw_->addTab(mp_widget_, tr("Motion Primitives"));
    walk_params_tw_->addTab(il_widget_, tr("Intuitive Leg"));

    walk_params_gl_ = new QGridLayout;
    walk_params_gl_->addWidget(walk_params_tw_,0,0,1,1);
    walk_params_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),1,1);

    walk_params_dialog_->setLayout(walk_params_gl_);    

    //-------------------------------------------------------------------------------------------------    

    menu_tw_ = new QTabWidget(walk_params_dialog_);
    menu_tw_->addTab(joint_driver_wid_, tr("Joint Control"));
    menu_tw_->addTab(new QWidget(), tr("IK Test"));
    menu_tw_->addTab(gait_widget_, tr("Gait"));
//    menu_tw_->setMaximumSize(menu_tw_->minimumSizeHint());
    menu_tw_->setMaximumHeight(menu_tw_->sizeHint().height());

    //-------------------------------------------------------------------------------------------------

    hw_cb_ = new QComboBox;
    hw_cb_->addItem(tr("Servo"));
    hw_cb_->addItem(tr("IMU"));

    hw_table_wid_ = new QTableWidget;
    hw_table_wid_->setRowCount(JointData::NUM_OF_JOINTS-1);
    hw_table_wid_->setColumnCount(2);

    table_header_ << "Name" << "Status";
    hw_table_wid_->setHorizontalHeaderLabels(table_header_);

    for(const auto id:JointData::ALL_ID){
//        hw_table_wid_->setItem(i,ID,new QTableWidgetItem(tr("%1").arg(i+1)));
        hw_table_wid_->setItem(id,
                               NAME,
                               new QTableWidgetItem(tr(JointData::ID2NAME[id].c_str())));
    }

    hw_servo_gl_ = new QGridLayout;
    hw_servo_gl_->addWidget(hw_table_wid_,0,0,1,1);
    hw_servo_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Ignored),1,1);

    hw_servo_widget_ = new QWidget;
    hw_servo_widget_->setLayout(hw_servo_gl_);

    hw_sw_ = new QStackedWidget;
    hw_sw_->addWidget(hw_servo_widget_);

    hw_gl_ = new QGridLayout;
    hw_gl_->addWidget(hw_cb_,0,0,1,1);
    hw_gl_->addWidget(hw_sw_,1,0,1,1);
    hw_gl_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Ignored),2,1);

    hw_gb_ = new QGroupBox;
    hw_gb_->setTitle(tr("Hardware Monitor"));
    hw_gb_->setStyleSheet("QGroupBox{border:2px solid rgb(14,76,146);border-radius:0px;}");
    hw_gb_->setLayout(hw_gl_);

//    hw_widget_ = new QWidget;
//    hw_widget_->setLayout(hw_gl_);

    //-------------------------------------------------------------------------------------------------


    viz = new Viz;
    viz->resize(320,480);
    viz->setMinimumSize(320,480);
//    viz->setContentsMargins(1,1,1,1);
//    viz->layout()->setContentsMargins(1,1,1,1);

    viz_gl_ = new QGridLayout;
    viz_gl_->addWidget(viz,0,0,1,1);

    viz_gb_ = new QGroupBox;
    viz_gb_->setStyleSheet(tr("QGroupBox{border:5px solid rgba(0,0,150,100); border-radius:10px; margin-top:10px;}"));
    viz_gb_->setLayout(viz_gl_);

    //-------------------------------------------------------------------------------------------------

    main_layout_->addWidget(hw_gb_,   0,0,2,1);
    main_layout_->addWidget(viz_gb_,  0,1,3,1);
    main_layout_->addWidget(menu_tw_, 0,2,1,1);
    main_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),3,2);

    main_widget_->setLayout(main_layout_);

    this->removeToolBar(ui->mainToolBar);
    this->setMaximumSize(900,600);
    this->setMinimumSize(900,600);
    this->setCentralWidget(main_widget_);

    // triggering initial condtion
    jointSliderPartChange(R_HAND);
    menuChangeAct(0);

}

void MainWindow::settingActions(){
    connect(joint_driver_cb_, SIGNAL(currentIndexChanged(int)), this, SLOT(jointSliderPartChange(int)));
    for(const auto id:JointData::ALL_ID){
        connect(joint_driver_sld_[id], SIGNAL(valueChanged(int)), this, SLOT(jointSliderAct(int)));
    }

    connect(this, SIGNAL(updateViz()), this, SLOT(updateVizAct()));

    connect(gait_start_pb_, SIGNAL(clicked(bool)), this, SLOT(walkStartAct()));
    connect(gait_stop_pb_, SIGNAL(clicked(bool)), this, SLOT(walkStopAct()));
    connect(gait_set_pb_, SIGNAL(clicked(bool)), this, SLOT(walkSetAct()));
    connect(gait_params_pb_, SIGNAL(clicked(bool)), this, SLOT(walkParamsAct()));

    connect(wp_legend_pb_, SIGNAL(clicked(bool)), this, SLOT(walkParamsLegend()));

    connect(this, SIGNAL(updatePlot()), this, SLOT(updatePlotAct()));

    connect(mp_cb_, SIGNAL(currentIndexChanged(int)), this, SLOT(motionPatternTypeChange(int)));
    connect(il_cb_, SIGNAL(currentIndexChanged(int)), this, SLOT(intuitiveLegTypeChange(int)));

    connect(menu_tw_, SIGNAL(currentChanged(int)), this, SLOT(menuChangeAct(int)));
}

void MainWindow::updateVizAct(){
    for(const auto id:JointData::ALL_ID){
        viz->updateAngle(id, viz_states_.angular_displacement[id] * MathUtils::RAD2DEG);
    }
    viz->update();
}

void MainWindow::updatePlotAct(){
    auto motion_phase = g_eng_data_.motion_phase;
    auto wp_tab_idx = walk_params_tw_->currentIndex();
    if(wp_tab_idx == 1){
        auto r_signal = g_eng_data_.motion_primitive.r_signal;
        auto l_signal = g_eng_data_.motion_primitive.l_signal;
        switch(mp_cb_->currentIndex()){
        case HALT:{
            mp_signals_[P_HALT_LEG_EXT]->updateData(motion_phase,
                                                    r_signal[P_HALT_LEG_EXT],
                                                    l_signal[P_HALT_LEG_EXT]);
            mp_signals_[P_HALT_LEG_ROLL]->updateData(motion_phase,
                                                     r_signal[P_HALT_LEG_ROLL],
                                                     l_signal[P_HALT_LEG_ROLL]);
            mp_signals_[P_HALT_LEG_PITCH]->updateData(motion_phase,
                                                      r_signal[P_HALT_LEG_PITCH],
                                                      l_signal[P_HALT_LEG_PITCH]);
            mp_signals_[P_HALT_FOOT_ROLL]->updateData(motion_phase,
                                                      r_signal[P_HALT_FOOT_ROLL],
                                                      l_signal[P_HALT_FOOT_ROLL]);
            mp_signals_[P_HALT_FOOT_PITCH]->updateData(motion_phase,
                                                       r_signal[P_HALT_FOOT_PITCH],
                                                       l_signal[P_HALT_FOOT_PITCH]);
        }break;
        case LEG_LIFTING:{
            mp_signals_[P_LIFT_LEG_EXT]->updateData(motion_phase,
                                                    r_signal[P_LIFT_LEG_EXT],
                                                    l_signal[P_LIFT_LEG_EXT]);
        }break;
        case LEG_SWING:{
            mp_signals_[P_SWING_LEG_PITCH]->updateData(motion_phase,
                                                       r_signal[P_SWING_LEG_PITCH],
                                                       l_signal[P_SWING_LEG_PITCH]);
            mp_signals_[P_SWING_LEG_ROLL]->updateData(motion_phase,
                                                      r_signal[P_SWING_LEG_ROLL],
                                                      l_signal[P_SWING_LEG_ROLL]);
            mp_signals_[P_SWING_LEG_YAW]->updateData(motion_phase,
                                                     r_signal[P_SWING_LEG_YAW],
                                                     l_signal[P_SWING_LEG_YAW]);
        }break;
        case LATERAL_HIP_SWING:{
            mp_signals_[P_SWING_HIP_LATERAL]->updateData(motion_phase,
                                                         r_signal[P_SWING_HIP_LATERAL],
                                                         l_signal[P_SWING_HIP_LATERAL]);
        }break;
        case LEANING:{
            mp_signals_[P_LEAN_LEG_PITCH]->updateData(motion_phase,
                                                      r_signal[P_LEAN_LEG_PITCH],
                                                      l_signal[P_LEAN_LEG_PITCH]);
            mp_signals_[P_LEAN_LEG_ROLL]->updateData(motion_phase,
                                                     r_signal[P_LEAN_LEG_ROLL],
                                                     l_signal[P_LEAN_LEG_ROLL]);
        }break;
        }
    }else if(wp_tab_idx == 2){
        auto r_patt = g_eng_data_.intuitive_leg.r_pattern;
        auto l_patt = g_eng_data_.intuitive_leg.l_pattern;
        switch(il_cb_->currentIndex()){
        case LEG_EXT:{
            il_patt_[LEG_EXT]->updateData(motion_phase,
                                          r_patt[LEG_EXT],
                                          l_patt[LEG_EXT]);
        }break;
        case LEG:{
            il_patt_[YAW_LEG]->updateData(motion_phase,
                                          r_patt[YAW_LEG],
                                          l_patt[YAW_LEG]);
            il_patt_[ROLL_LEG]->updateData(motion_phase,
                                          r_patt[ROLL_LEG],
                                          l_patt[ROLL_LEG]);
            il_patt_[PITCH_LEG]->updateData(motion_phase,
                                          r_patt[PITCH_LEG],
                                          l_patt[PITCH_LEG]);
        }break;
        case FOOT:{
            il_patt_[PITCH_FOOT]->updateData(motion_phase,
                                          r_patt[PITCH_FOOT],
                                          l_patt[PITCH_FOOT]);
            il_patt_[ROLL_FOOT]->updateData(motion_phase,
                                          r_patt[ROLL_FOOT],
                                          l_patt[ROLL_FOOT]);
        }break;
        }
    }
}

void MainWindow::jointSliderPartChange(int _value){

    switch(_value){
    case R_HAND:{
        joint_driver_gb_->setTitle(tr("RIGHT_HAND"));
        joint_driver_gb_->setMaximumHeight(r_hand_wid_->sizeHint().height());
    }break;
    case L_HAND:{
        joint_driver_gb_->setTitle(tr("LEFT_HAND"));
        joint_driver_gb_->setMaximumHeight(l_hand_wid_->sizeHint().height());
    }break;
    case R_LEG:{
        joint_driver_gb_->setTitle(tr("RIGHT_LEG"));
        joint_driver_gb_->setMaximumHeight(r_leg_wid_->sizeHint().height());
    }break;
    case L_LEG:{
        joint_driver_gb_->setTitle(tr("LEFT_LEG"));
        joint_driver_gb_->setMaximumHeight(l_leg_wid_->sizeHint().height());
    }break;
    }
    joint_driver_sl_->setCurrentIndex(_value);

}

void MainWindow::jointSliderAct(int _value){
    auto target_joint = QObject::sender();

    auto target_id = JointData::R_SHO_PITCH;

    for(const auto id:JointData::ALL_ID){
        if(target_joint == joint_driver_sld_[id]){
            target_id = id;
            break;
        }
    }

    std::stringstream ss;
    ss << JointData::ID2NAME[target_id].c_str() << " : %1";
    joint_driver_label_[target_id]->setText(tr(ss.str().c_str()).arg(QString::number(_value,'f',2)));

    joint_angle_data_.position[target_id] = _value * MathUtils::DEG2RAD;
//    std::cout << joint_angle_data_.position[target_id] << std::endl;
    joint_angle_pub_.publish(joint_angle_data_);

}

void MainWindow::walkStartAct(){

    high_level_msgs::ActiveMotion active_motion;
    active_motion.motion_name = "walk";
    active_motion_pub_.publish(active_motion);

    high_level_msgs::WalkCmd walk_cmd;
    walk_cmd.walk_status = true;
    walk_cmd_pub_.publish(walk_cmd);
}

void MainWindow::walkStopAct(){

    high_level_msgs::WalkCmd walk_cmd;
    walk_cmd.walk_status = false;
    walk_cmd_pub_.publish(walk_cmd);

    high_level_msgs::ActiveMotion active_motion;
    active_motion.motion_name = "none";;
    active_motion_pub_.publish(active_motion);
}

void MainWindow::walkSetAct(){
    high_level_msgs::WalkInput walk_input_;
    walk_input_.v_x = gait_vx_dsb_->value();
    walk_input_.v_y = gait_vy_dsb_->value();
    walk_input_.v_phi = gait_vphi_dsb_->value();

    walk_input_pub_.publish(walk_input_);

    updateDynamicReconfigure();
}

void MainWindow::walkParamsLegend(){
    wp_legend_dialog_->show();
}

void MainWindow::updateDynamicReconfigure(){
    std::array<dynamic_reconfigure::DoubleParameter, 29 > params;
    dynamic_reconfigure::Config conf;

    params[C1].name = "C1";params[C2].name = "C2";
    params[C3].name = "C3";params[C4].name = "C4";
    params[C5].name = "C5";params[C6].name = "C6";
    params[C7].name = "C7";params[C8].name = "C8";
    params[C9].name = "C9";params[C_TAU_0].name = "C_TAU_0";
    params[C_TAU_1].name = "C_TAU_1";params[C10].name = "C10";
    params[C11].name = "C11";params[C12].name = "C12";
    params[C13].name = "C13";params[C14].name = "C14";
    params[C15].name = "C15";params[C16].name = "C16";
    params[C17].name = "C17";params[C18].name = "C18";
    params[C19].name = "C19";params[C20].name = "C20";
    params[C21].name = "C21";params[C22].name = "C22";
    params[C23].name = "C23";params[C24].name = "C24";
    params[C25].name = "C25";params[C26].name = "C26";
    params[C27].name = "C27";

    for(int i(0); i < 29; i++){
        params[i].value = walk_config_dsb_[i]->value();
    }

    conf.doubles.insert(conf.doubles.begin(),params.begin(), params.end());
    srv_req_.config = conf;

    ros::service::call("/motion_assistance_node/set_parameters", srv_req_, srv_res_);
}

void MainWindow::walkParamsAct(){
    walk_params_dialog_->show();
//    walk_params_dialog_->setFocus();
}

void MainWindow::motionPatternTypeChange(int _value){
    for(int i(0);i<mp_type_sw_->count();i++){
        QSizePolicy::Policy policy = QSizePolicy::Ignored;
        if(i == _value)
            policy = QSizePolicy::Expanding;

        mp_type_sw_->widget(i)->setSizePolicy(policy,policy);
    }

    mp_type_sw_->setCurrentIndex(_value);
    mp_sa_->resize(mp_sa_->width(), mp_type_sw_->sizeHint().height());
}

void MainWindow::intuitiveLegTypeChange(int _value){
    for(int i(0);i<il_patt_sw_->count();i++){
        QSizePolicy::Policy policy = QSizePolicy::Ignored;
        if(i == _value)
            policy = QSizePolicy::Expanding;

        il_patt_sw_->widget(i)->setSizePolicy(policy,policy);
    }

    il_patt_sw_->setCurrentIndex(_value);    
    il_sa_->resize(il_sa_->width(), il_patt_sw_->sizeHint().height());
}

void MainWindow::menuChangeAct(int _value){
    high_level_msgs::ActiveMotion active_motion;
    high_level_msgs::WalkCmd walk_cmd;
    high_level_msgs::WalkInput walk_input;
    switch(_value){
    case 0:{
        active_motion.motion_name = "joint_control";
        walk_cmd.walk_status = false;
        walk_input.v_x = walk_input.v_y = walk_input.v_phi = .0;
        walk_cmd_pub_.publish(walk_cmd);
        walk_input_pub_.publish(walk_input);
        active_motion_pub_.publish(active_motion);
    }break;
    case 1:{
        active_motion.motion_name = "none";
        walk_cmd.walk_status = false;
        walk_input.v_x = walk_input.v_y = walk_input.v_phi = .0;
        walk_cmd_pub_.publish(walk_cmd);
        walk_input_pub_.publish(walk_input);
        active_motion_pub_.publish(active_motion);
    }break;
    case 2:{
        active_motion.motion_name = "walk";
        walk_cmd.walk_status = false;
        walk_input.v_x = walk_input.v_y = walk_input.v_phi = .0;
        walk_cmd_pub_.publish(walk_cmd);
        walk_input_pub_.publish(walk_input);
        active_motion_pub_.publish(active_motion);
    }break;
    }
}

void MainWindow::loadGaitEngineConfig(){
    YAML::Node config_file_;

    try{
        std::stringstream file_path;
        file_path << ros::package::getPath("high_level_cfg") << "/config/gait_config.yaml";
        config_file_ = YAML::LoadFile(file_path.str().c_str());
    }catch(std::exception &e){
        std::cerr << "[gait_tuner] Unable to load config file: " << e.what() << std::endl;
    }

    walk_config_dsb_[C1]->setValue(config_file_["C1"].as<double>());
    walk_config_dsb_[C2]->setValue(config_file_["C2"].as<double>());
    walk_config_dsb_[C3]->setValue(config_file_["C3"].as<double>());
    walk_config_dsb_[C4]->setValue(config_file_["C4"].as<double>());
    walk_config_dsb_[C5]->setValue(config_file_["C5"].as<double>());
    walk_config_dsb_[C6]->setValue(config_file_["C6"].as<double>());
    walk_config_dsb_[C7]->setValue(config_file_["C7"].as<double>());
    walk_config_dsb_[C8]->setValue(config_file_["C8"].as<double>());
    walk_config_dsb_[C9]->setValue(config_file_["C9"].as<double>());
    walk_config_dsb_[C_TAU_0]->setValue(config_file_["C_TAU_0"].as<double>());
    walk_config_dsb_[C_TAU_1]->setValue(config_file_["C_TAU_1"].as<double>());
    walk_config_dsb_[C10]->setValue(config_file_["C10"].as<double>());
    walk_config_dsb_[C11]->setValue(config_file_["C11"].as<double>());
    walk_config_dsb_[C12]->setValue(config_file_["C12"].as<double>());
    walk_config_dsb_[C13]->setValue(config_file_["C13"].as<double>());
    walk_config_dsb_[C14]->setValue(config_file_["C14"].as<double>());
    walk_config_dsb_[C15]->setValue(config_file_["C15"].as<double>());
    walk_config_dsb_[C16]->setValue(config_file_["C16"].as<double>());
    walk_config_dsb_[C17]->setValue(config_file_["C17"].as<double>());
    walk_config_dsb_[C18]->setValue(config_file_["C18"].as<double>());
    walk_config_dsb_[C19]->setValue(config_file_["C19"].as<double>());
    walk_config_dsb_[C20]->setValue(config_file_["C20"].as<double>());
    walk_config_dsb_[C21]->setValue(config_file_["C21"].as<double>());
    walk_config_dsb_[C22]->setValue(config_file_["C22"].as<double>());
    walk_config_dsb_[C23]->setValue(config_file_["C23"].as<double>());
    walk_config_dsb_[C24]->setValue(config_file_["C24"].as<double>());
    walk_config_dsb_[C25]->setValue(config_file_["C25"].as<double>());
    walk_config_dsb_[C26]->setValue(config_file_["C26"].as<double>());
    walk_config_dsb_[C27]->setValue(config_file_["C27"].as<double>());

}
