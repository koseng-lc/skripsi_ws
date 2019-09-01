/**
 * @author koseng : lintangerlangga@gmail.com
 */

#pragma once

#include <QMainWindow>
#include <QGridLayout>
#include <QTabWidget>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QGroupBox>
#include <QStackedLayout>
#include <QStackedWidget>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QDialog>
#include <QTableWidget>
#include <QTextEdit>
#include <QtCharts/QChartView>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <high_level_msgs/ActiveMotion.h>
#include <high_level_msgs/GaitEngineData.h>
#include <high_level_msgs/VizStates.h>
#include <high_level_msgs/WalkCmd.h>
#include <high_level_msgs/WalkInput.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "gait_tuner/viz.h"
#include "gait_tuner/signal_plotter.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:

    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;

    boost::mutex spin_mtx_;
    boost::condition_variable spin_cv_;
    void spinThread();
    boost::thread spin_thread_;

    ros::Subscriber viz_states_sub_;
    high_level_msgs::VizStates viz_states_;
    void vizStatesCb(const high_level_msgs::VizStatesConstPtr &_msg);

    ros::Subscriber g_eng_data_sub_;
    high_level_msgs::GaitEngineData g_eng_data_;
    void gaitEngineDataCb(const high_level_msgs::GaitEngineDataConstPtr &_msg);

    ros::Publisher active_motion_pub_;
    ros::Publisher walk_cmd_pub_;
    ros::Publisher walk_input_pub_;

    Ui::MainWindow *ui;

    QGridLayout* main_layout_;
    QWidget* main_widget_;

    QTabWidget* menu_tw_;

    QWidget* joint_driver_wid_;
    QGridLayout* joint_driver_gl_;
    QComboBox* joint_driver_cb_;
    QGroupBox* joint_driver_gb_;

    enum{
        R_HAND = 0,
        L_HAND = 1,
        R_LEG = 2,
        L_LEG = 3,
    };

    QStackedLayout* joint_driver_sl_;

    QLabel *joint_driver_label_[JointData::NUM_OF_JOINTS];
    QSlider *joint_driver_sld_[JointData::NUM_OF_JOINTS];
    sensor_msgs::JointState joint_angle_data_;
    ros::Publisher joint_angle_pub_;

//    Kinematics* kinematics_;

    QWidget* r_hand_wid_;
    QGridLayout* r_hand_gl_;

    QWidget* l_hand_wid_;
    QGridLayout* l_hand_gl_;

    QWidget* r_leg_wid_;
    QGridLayout* r_leg_gl_;

    QWidget* l_leg_wid_;
    QGridLayout* l_leg_gl_;

    //---------

    QWidget* gait_widget_;
    QGridLayout* gait_gl_;

    QGroupBox* gait_input_gb_;
    QGridLayout* gait_input_gl_;

    QLabel* gait_vx_label_;
    QDoubleSpinBox* gait_vx_dsb_;

    QLabel* gait_vy_label_;
    QDoubleSpinBox* gait_vy_dsb_;

    QLabel* gait_vphi_label_;
    QDoubleSpinBox* gait_vphi_dsb_;

    QPushButton* gait_start_pb_;
    QPushButton* gait_stop_pb_;
    QPushButton* gait_set_pb_;
    QPushButton* gait_params_pb_;

    //---------

    QDialog* walk_params_dialog_;
    QTabWidget* walk_params_tw_;
    QGridLayout* walk_params_gl_;

    enum Config{
        C1 = 0, C2, C3, C4, C5, C6, C7, C8, C9, C_TAU_0, C_TAU_1,
        C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20,
        C21, C22, C23, C24, C25, C26, C27
    };

    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_res_;

    QLabel* walk_config_label_[29];
    QDoubleSpinBox* walk_config_dsb_[29];

    QPushButton* wp_legend_pb_;
    QDialog* wp_legend_dialog_;
    QGridLayout* wp_legend_gl_;
    QTextEdit* wp_legend_le_;

    QGroupBox* wp_halt_gb_;
    QGridLayout* wp_halt_gl_;
    QGroupBox* wp_leg_lifting_gb_;
    QGridLayout* wp_leg_lifting_gl_;
    QGroupBox* wp_leg_swing_gb_;
    QGridLayout* wp_leg_swing_gl_;
    QGroupBox* wp_lat_hip_swing_gb_;
    QGridLayout* wp_lat_hip_swing_gl_;
    QGroupBox* wp_leaning_gb_;
    QGridLayout* wp_leaning_gl_;
    QGroupBox* wp_ctrl_interface_gb_;
    QGridLayout* wp_ctrl_interface_gl_;

    QGridLayout* wp_gl_;
    QWidget* wp_widget_;

    //---------

    enum MotionPrimtive{
        // Halt Position
        P_HALT_LEG_EXT = 0,
        P_HALT_LEG_ROLL,
        P_HALT_LEG_PITCH,
        P_HALT_FOOT_ROLL,
        P_HALT_FOOT_PITCH,
        // Leg Lifting
        P_LIFT_LEG_EXT,
        // Leg Swing
        P_SWING_LEG_PITCH,
        P_SWING_LEG_ROLL,
        P_SWING_LEG_YAW,
        // Hip Swing Lateral
        P_SWING_HIP_LATERAL,
        // Leaning
        P_LEAN_LEG_PITCH,
        P_LEAN_LEG_ROLL
    };

    enum IntuitiveLegParam{
        LEG_EXT = 0,
        YAW_LEG,
        ROLL_LEG,
        PITCH_LEG,
        PITCH_FOOT,
        ROLL_FOOT
    };


    enum {
        HALT = 0,
        LEG_LIFTING,
        LEG_SWING,
        LATERAL_HIP_SWING,
        LEANING
    };

    enum{
        LEG = 1,
        FOOT = 2
    };

    QComboBox* mp_cb_;
    QLabel* mp_legend_label_;
    SignalPlotter* mp_signals_[12];
    QChartView* mp_signals_cview_[12];    
    QGridLayout* mp_type_gl_[5];
    QWidget* mp_type_widget_[5];
    QStackedWidget* mp_type_sw_;
    QScrollArea *mp_sa_;
    QGridLayout* mp_gl_;    
    QWidget* mp_widget_;


    QComboBox* il_cb_;
    QLabel* il_legend_label_;
    SignalPlotter* il_patt_[6];
    QChartView* il_patt_cview_[6];
    QGridLayout* il_patt_gl_[3];
    QWidget* il_patt_widget_[3];
    QStackedWidget* il_patt_sw_;
    QScrollArea* il_sa_;
    QGridLayout* il_gl_;
    QWidget* il_widget_;

    //---------

    enum TablePart{
        NAME = 0,
        STATUS
    };

    QComboBox* hw_cb_;
    QStringList table_header_;
    QTableWidget* hw_table_wid_;
    QGridLayout* hw_servo_gl_;
    QWidget* hw_servo_widget_;
    QGroupBox* hw_gb_;
    QStackedWidget* hw_sw_;
    QGridLayout* hw_gl_;
    QWidget* hw_widget_;

    //---------

    QGroupBox* viz_gb_;
    QGridLayout* viz_gl_;

    Viz* viz;

    void settingWidgets();
    void settingActions();

    void updateDynamicReconfigure();
    void loadGaitEngineConfig();
signals:
    void updateVizState(int _joint_id, double _delta_tetha);
    void updateViz();
    void updatePlot();
private slots:
    void updateVizAct();
    void updatePlotAct();
    void jointSliderPartChange(int _value);
    void jointSliderAct(int _value);
    void walkStartAct();
    void walkStopAct();
    void walkSetAct();
    void walkParamsAct();
    void walkParamsLegend();
    void motionPatternTypeChange(int _value);
    void intuitiveLegTypeChange(int _value);
    void menuChangeAct(int _value);
};
