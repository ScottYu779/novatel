#ifndef _NOVATEL_NODE_H
#define _NOVATEL_NODE_H
#include <ros/ros.h>

#include "gps_msgs/Ephemeris.h"
#include "gps_msgs/L1L2Range.h"
#include "gps_msgs/Gps_Data_Ht.h"
#include "novatel/novatel.h"
//#include "novatel/novatel_structures.h"


using namespace novatel;
using namespace std;


typedef enum
{
  release_state = 0,
  debug_state,//调试阶段和非调试阶段的分界线
  real_team_debug = debug_state,
  real_team_debug_single_gps_device,
  simulate_nodes_debug,
  test_catch_track_file,
  coding_debug,//解决基本代码是否有问题，是否能够联通gps设备，是否有数据上来的级别的bug
  max_state = coding_debug,
}Debug_state;//调试节奏是从下往上


int CODE_STATE  = real_team_debug;



typedef struct
{
  uint32_t num;
  double x;
  double y;
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

  void run();

public://data
  static int gps_update_hz_;
  static std::string track_file_output_path_;

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
  ros::ServiceServer gps_init_data_server_;

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
  gps_msgs::Ephemeris cur_ephem_;
  gps_msgs::L1L2Range cur_range_;
  InsCovariance cur_ins_cov_;
  Position cur_psrpos_;
  UtmPosition cur_utm_bestpos_;

  gps_msgs::Gps_Data_Ht gps_data_ht_;
  //gps_msgs::Gps_Init_Data_Ht gps_init_data_ht_;
  int pub_init_data_done;
  std::string ori_track_file_path_;
  
  std::string track_file_input_path_;
  bool track_file_channel = false;
}; // end class NovatelNode

#endif //_NOVATEL_NODE_H