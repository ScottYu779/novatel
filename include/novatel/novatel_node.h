#ifndef _NOVATEL_NODE_H
#define _NOVATEL_NODE_H
#include <ros/ros.h>

#include "msgs_ht/Ephemeris.h"
#include "msgs_ht/L1L2Range.h"
#include "msgs_ht/Gps_Data_Ht.h"
#include "novatel/novatel.h"
//#include "novatel/novatel_structures.h"


using namespace novatel;
using namespace std;


typedef enum
{
    release_state_ = 0,
  min_ = release_state_,

        team_debug_on_car_,
      team_debug_min_ = team_debug_on_car_,//范围控制边界
    debug_min_ = team_debug_min_,//范围控制边界
        team_debug_rtk_,
        team_debug_rtk_drift_err_,
        team_debug_single_gps_device_,
      team_debug_max_ = team_debug_single_gps_device_,  //范围控制边界 解决基本代码是否有问题，是否能够联通gps设备，是否有数据上来的级别的bug
                                                        //如果出了代码级别问题，就把这个打开，日志的设定肯定能够帮助找到bug

        solo_debug_on_car_,
      solo_debug_min_ = solo_debug_on_car_,//范围控制边界
        solo_debug_rtk_,
        solo_debug_single_gps_device_,
      solo_debug_max_ = solo_debug_single_gps_device_,  //范围控制边界 解决基本代码是否有问题，是否能够联通gps设备，是否有数据上来的级别的bug
                                                        //如果出了代码级别问题，就把这个打开，日志的设定肯定能够帮助找到bug
        
        file_xyh_state,
      test_file_io_min_ = file_xyh_state,
      ori_gps_file_convert_,      //特殊测试状态
        file_xy_state,
        file_xyv_state,
        file_precision_state,
      test_file_io_max_ = file_precision_state,

        simulate_single_node_debug_,                 //不连接设备的调试
      simulate_debug_min_ = simulate_single_node_debug_,//范围控制边界
        simulate_multi_nodes_debug_,//多节点虚拟数据测试
      simulate_debug_max_ = simulate_multi_nodes_debug_,//范围控制边界
    debug_max_ = simulate_debug_max_,//调试阶段和非调试阶段的分界线
  
  max_ = debug_max_,
}Debug_state;//调试节奏是从下往上


int CODE_STATE  = team_debug_rtk_drift_err_;



typedef struct
{
  uint32_t num;
  double x;
  double y;
  double h;
}track_data;

// ROS Node class
class NovatelNode
{
public:

  NovatelNode();
  ~NovatelNode();

  void SIGINT_handler(int sig);

  inline double psi2theta(double psi) { return M_PI / 2 - psi; }
  inline double theta2psi(double theta) { return M_PI / 2 - theta; }

  void BestUtmHandler(UtmPosition &pos, double &timestamp);
  void BestVelocityHandler(Velocity &vel, double &timestamp);
  void InsPvaHandler(InsPositionVelocityAttitude &ins_pva, double &timestamp);

  void run();

public://data
  static int track_file_simulate_freq_;
  static std::string file_xy_fd_path;
  static int file_xyh_flag_;
  static std::string file_xyh_fd_path;
  static int file_precision_flag_;
  static std::string file_precision_fd_path_;
  struct timeval tv_;
  struct timezone tz_;
  time_t current_time_;
  struct tm *local_time_;
  static std::ofstream track_file_out_1_;
  static std::ofstream track_file_out_2_;
  static string path_temp_1_;
  static string path_temp_2_;

protected:
  void send_rest_locate_data_frq_func();
  int get_ori_track_data_for_init_data();
  void disconnect()
  {
    //em_.stopReading();
    //em_.disconnect();
    //gps_.SendCommand("UNLOGALL");
    gps_.Disconnect();
  }

  bool getParameters();

  ////////////////////////////////////////////////////////////////
  // ROSNODE Members
  ////////////////////////////////////////////////////////////////
  ros::NodeHandle nh_;
  std::string name_;
  ros::Publisher odom_publisher_;
  ros::Publisher exhibition_odom_publisher_;
  //ros::Publisher gps_init_data_publisher_;
  //ros::ServiceServer gps_init_data_server_;
  ros::Publisher nav_sat_fix_publisher_;

  Novatel gps_; //

  // topics - why are we not using remap arguments?
  std::string odom_topic_;
  std::string exhibition_odom_topic_;
  std::string gps_init_data_topic_;

  std::string port_;
  std::string log_commands_;
  std::string configure_port_;
  double gps_default_logs_period_;
  double span_default_logs_period_;
  double range_default_logs_period_;
  double psrpos_default_logs_period_;
  std::string ephem_log_;
  int baudrate_;
  double poll_rate_;

  Velocity cur_velocity_;
  msgs_ht::Ephemeris cur_ephem_;
  msgs_ht::L1L2Range cur_range_;
  InsCovariance cur_ins_cov_;
  Position cur_psrpos_;
  UtmPosition cur_utm_bestpos_;

  msgs_ht::Gps_Data_Ht gps_data_ht_;
  //msgs_ht::Gps_Init_Data_Ht gps_init_data_ht_;
  int pub_init_data_done;
  std::string ori_track_file_path_;
  
  std::string track_file_input_path_for_test_simulate_;
  bool track_file_channel = false;
}; // end class NovatelNode

#endif //_NOVATEL_NODE_H
