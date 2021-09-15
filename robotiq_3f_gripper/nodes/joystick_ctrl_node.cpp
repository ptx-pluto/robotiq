#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>

using robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput;
using robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput;

template<typename T>
void clamp( T & val, T min_val, T max_val ){
  val = std::min(val,max_val);
  val = std::max(val,min_val);
}

struct joystick_ctrl_node_t  {

    double msg_timeout = 1;

    double ctrl_rate = 10;

    ros::Subscriber sub_js;

    // 10hz
    ros::Subscriber sub_input;

    ros::Publisher pub_output;

    ros::Timer cmd_loop;

    Robotiq3FGripperRobotInput latest_input;

    ros::Time latest_input_ts;

    double vel_ratio = 50;

    // state
    ros::Time ts;

    double pos_a = 0;
    double pos_b = 0;
    double pos_c = 0;
    double pos_s = 0;

    double vel_a = 0;
    double vel_b = 0;
    double vel_c = 0;
    double vel_s = 0;

    joystick_ctrl_node_t(){

      ros::NodeHandle nh("~");

      sub_js = nh.subscribe<sensor_msgs::Joy>(
              "/joy",
              10,
              &joystick_ctrl_node_t::on_js,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_input = nh.subscribe<Robotiq3FGripperRobotInput>(
              "/Robotiq3FGripperRobotInput",
              1,
              &joystick_ctrl_node_t::on_input,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      pub_output = nh.advertise<Robotiq3FGripperRobotOutput>("/Robotiq3FGripperRobotOutput", 10);

      cmd_loop = nh.createTimer(
              ros::Rate(ctrl_rate),
              &joystick_ctrl_node_t::on_cmd_loop,
              this
      );

    }

    void on_js( sensor_msgs::JoyConstPtr const & msg ){
      vel_a = vel_ratio * msg->axes.at(0);
      vel_b = vel_ratio * msg->axes.at(1);
      vel_c = vel_ratio * msg->axes.at(4);
      vel_s = vel_ratio * 2 * msg->axes.at(3);
    }

    void on_input( Robotiq3FGripperRobotInput::ConstPtr const & msg ){
      latest_input = *msg;
      latest_input_ts = ros::Time::now();
    }

    void on_cmd_loop( ros::TimerEvent const & e ){

      Robotiq3FGripperRobotOutput cmd;
      cmd.rACT = 1; // activate
      cmd.rGTO = 1; // goto

      cmd.rICF = 1; // individual finger control
      cmd.rICS = 1; // control sissor

      // finger force
      cmd.rFRA = 150;
      cmd.rFRB = 150;
      cmd.rFRC = 150;
      cmd.rFRS = 150;

      // finger speed
      cmd.rSPA = 255;
      cmd.rSPB = 255;
      cmd.rSPC = 255;
      cmd.rSPS = 255;

      // apply action

      double dt = 1./ctrl_rate;
      pos_a += vel_a * dt;
      pos_b += vel_b * dt;
      pos_c += vel_c * dt;
      pos_s += vel_s * dt;

      clamp<double>(pos_a,0,255);
      clamp<double>(pos_b,0,255);
      clamp<double>(pos_c,0,255);
      clamp<double>(pos_s,0,255);

      cmd.rPRA = (uint8_t)pos_a;
      cmd.rPRB = (uint8_t)pos_b;
      cmd.rPRC = (uint8_t)pos_c;
      cmd.rPRS = (uint8_t)pos_s;

      pub_output.publish(cmd);

    }

    bool ready() const {
      return (latest_input_ts - ros::Time::now()).toSec() < msg_timeout
             && latest_input.gACT > 0 // activated
             && latest_input.gGTO > 0; // in ready state
    }

};


int main(int argc, char** argv){

  ros::init(argc,argv,"joystick_ctrl_node");

  joystick_ctrl_node_t node;

  ros::spin();

  ros::shutdown();

}