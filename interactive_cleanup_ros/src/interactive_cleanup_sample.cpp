#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "interactive_cleanup/msg/interactive_cleanup_msg.hpp"

class InteractiveCleanupSample : public rclcpp::Node {
private:
  enum Step {
    Initialize,
    Ready,
    WaitForInstruction,
    GoToRoom1,
    GoToRoom2,
    MoveToInFrontOfTarget,
    Grasp,
    WaitForGrasping,
    ComeBack,
    ReleaseObject,
    WaitForReleasing,
    TaskFinished,
  };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_CLEAN_UP         = "Clean_up!";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";

  const std::string MSG_I_AM_READY      = "I_am_ready";
  const std::string MSG_OBJECT_GRASPED  = "Object_grasped";
  const std::string MSG_TASK_FINISHED   = "Task_finished";

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  int step_;

  bool has_been_instructed_;

  bool is_started_;
  bool is_finished_;
  bool is_failed_;

  void init() {
    // Arm Joint Trajectory
    std::vector<std::string> arm_joint_names {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

    arm_joint_trajectory_.joint_names = arm_joint_names;
    arm_joint_trajectory_.points.push_back(arm_joint_point);

    step_ = Initialize;

    reset();
  }

  void reset() {
    is_started_          = false;
    has_been_instructed_ = false;
    is_finished_         = false;
    is_failed_           = false;

    std::vector<double> arm_positions { 0.0, 0.0, 0.0, 0.0, 0.0 };
    arm_joint_trajectory_.points[0].positions = arm_positions;
  }


  void messageCallback(const interactive_cleanup::msg::InteractiveCleanupMsg::ConstPtr& message) {
    RCLCPP_INFO(this->get_logger(), "Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message.c_str()==MSG_ARE_YOU_READY) {
      if(step_==Ready) is_started_ = true;
    }
    if(message->message.c_str()==MSG_CLEAN_UP){
      if(step_==WaitForInstruction) has_been_instructed_ = true;
    }
    if(message->message.c_str()==MSG_TASK_SUCCEEDED) {
      if(step_==TaskFinished) is_finished_ = true;
    }
    if(message->message.c_str()==MSG_TASK_FAILED) is_failed_ = true;
    if(message->message.c_str()==MSG_MISSION_COMPLETE) exit(EXIT_SUCCESS);
  }

  void sendMessage(rclcpp::Publisher<interactive_cleanup::msg::InteractiveCleanupMsg>::SharedPtr publisher, const std::string &message) {
    RCLCPP_INFO(this->get_logger(), "Send message:%s", message.c_str());

    interactive_cleanup::msg::InteractiveCleanupMsg interactive_cleanup_msg;
    interactive_cleanup_msg.message = message;
    publisher->publish(interactive_cleanup_msg);
  }

  geometry_msgs::msg::TransformStamped getTfBase(tf2_ros::Buffer &tf_buffer) {
    geometry_msgs::msg::TransformStamped tf_transform;

    try {
      tf_transform = tf_buffer.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }

    return tf_transform;
  }

  void moveBase(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, double linear_x, double linear_y, double angular_z) {
    geometry_msgs::msg::Twist twist;

    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    publisher->publish(twist);
  }

  void stopBase(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher) {
    moveBase(publisher, 0.0, 0.0, 0.0);
  }

  void moveArm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::vector<double> &positions, rclcpp::Duration &duration) {
    arm_joint_trajectory_.points[0].positions = positions;
    arm_joint_trajectory_.points[0].time_from_start = duration;

    publisher->publish(arm_joint_trajectory_);
  }

  void operateHand(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, bool should_grasp) {
    std::vector<std::string> joint_names {"hand_motor_joint"};
    std::vector<double> positions;

    if(should_grasp) {
      RCLCPP_DEBUG(this->get_logger(), "Grasp");
      positions.push_back(-0.105);
    }
    else {
      RCLCPP_DEBUG(this->get_logger(), "Open hand");
      positions.push_back(+1.239);
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher->publish(joint_trajectory);
  }


public:
  InteractiveCleanupSample() : Node("interactive_cleanup_sample"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
  }
  int run(int argc, char **argv) {
    rclcpp::Rate loop_rate(10);

    std::string sub_msg_to_robot_topic_name = "/interactive_cleanup/message/to_robot";
    std::string pub_msg_to_moderator_topic_name = "/interactive_cleanup/message/to_moderator";
    std::string pub_base_twist_topic_name = "/hsrb/command_velocity";
    std::string pub_arm_trajectory_topic_name = "/hsrb/arm_trajectory_controller/command";
    std::string pub_gripper_trajectory_topic_name = "/hsrb/gripper_controller/command";

    this->declare_parameter<std::string>("sub_msg_to_robot_topic_name", "/interactive_cleanup/message/to_robot");
    this->declare_parameter<std::string>("pub_msg_to_moderator_topic_name", "/interactive_cleanup/message/to_moderator");
    this->declare_parameter<std::string>("pub_base_twist_topic_name", "/hsrb/command_velocity");
    this->declare_parameter<std::string>("pub_arm_trajectory_topic_name", "/hsrb/arm_trajectory_controller/command");
    this->declare_parameter<std::string>("pub_gripper_trajectory_topic_name", "/hsrb/gripper_controller/command");

    this->get_parameter("sub_msg_to_robot_topic_name", sub_msg_to_robot_topic_name);
    this->get_parameter("pub_msg_to_moderator_topic_name", pub_msg_to_moderator_topic_name);
    this->get_parameter("pub_base_twist_topic_name", pub_base_twist_topic_name);
    this->get_parameter("pub_arm_trajectory_topic_name", pub_arm_trajectory_topic_name);
    this->get_parameter("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name);

    init();

    rclcpp::Time waiting_start_time;

    RCLCPP_INFO(this->get_logger(), "InteractiveCleanup sample start!");

    auto sub_msg = this->create_subscription<interactive_cleanup::msg::InteractiveCleanupMsg>(sub_msg_to_robot_topic_name, 100, std::bind(&InteractiveCleanupSample::messageCallback, this, std::placeholders::_1));
    auto pub_msg = this->create_publisher<interactive_cleanup::msg::InteractiveCleanupMsg>(pub_msg_to_moderator_topic_name, 10);
    auto pub_base_twist = this->create_publisher<geometry_msgs::msg::Twist>(pub_base_twist_topic_name, 10);
    auto pub_arm_trajectory = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    auto pub_gripper_trajectory = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);

    while (rclcpp::ok()) {
      if(is_failed_) {
        RCLCPP_INFO(this->get_logger(), "Task failed!");
        step_ = Initialize;
      }

      switch(step_) {
        case Initialize: {
          reset();
          step_++;
          break;
        }
        case Ready: {
          if(is_started_) {
            sendMessage(pub_msg, MSG_I_AM_READY);

            RCLCPP_INFO(this->get_logger(), "Task start!");

            step_++;
          }
          break;
        }
        case WaitForInstruction: {
          if(has_been_instructed_) {

            operateHand(pub_gripper_trajectory, false);

            step_++;
          }
          break;
        }
        case GoToRoom1: {
          geometry_msgs::msg::TransformStamped tf_transform = getTfBase(tf_buffer_);

          if (tf_transform.transform.translation.y <= 0.2) moveBase(pub_base_twist, 1.0, 0.0, 1.0);
          else {
            stopBase(pub_base_twist);
            step_++;
          }
          break;
        }
        case GoToRoom2:
        {
          geometry_msgs::msg::TransformStamped tf_transform = getTfBase(tf_buffer_);

          if (tf_transform.transform.translation.y <= 0.6) moveBase(pub_base_twist, 1.0, 0.0, 0.0);
          else {
            stopBase(pub_base_twist);

            step_++;
          }
          break;
        }
        case MoveToInFrontOfTarget:
        {
          geometry_msgs::msg::TransformStamped tf_transform = getTfBase(tf_buffer_);

          if (tf_transform.transform.translation.y <= 1.0) {
            std::vector<double> positions { 0.22, -1.57, 0.0, 0.0, 0.0 };
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);

            moveBase(pub_base_twist, 1.0, 0.0, 0.0);
            moveArm(pub_arm_trajectory, positions, duration);
          }
          else {
            stopBase(pub_base_twist);

            step_++;
          }
          break;
        }
        case Grasp: {
          geometry_msgs::msg::TransformStamped tf_transform = getTfBase(tf_buffer_);

          if (tf_transform.transform.translation.y >= 1.2) moveBase(pub_base_twist, 0.3, 0.0, 0.0);
          else {
            stopBase(pub_base_twist);
            operateHand(pub_gripper_trajectory, true);

            waiting_start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();;
            step_++;
          }
          break;
        }
        case WaitForGrasping: {
          if (rclcpp::Clock(RCL_SYSTEM_TIME).now() - waiting_start_time > rclcpp::Duration::from_seconds(3.0)) {
            sendMessage(pub_msg, MSG_OBJECT_GRASPED);
            step_++;
          }
          break;
        }
        case ComeBack: {
          geometry_msgs::msg::TransformStamped tf_transform = getTfBase(tf_buffer_);

          if (tf_transform.transform.translation.y <= 0.0) moveBase(pub_base_twist, -1.0, 0.0, 0.0);
          else {
            stopBase(pub_base_twist);
            sendMessage(pub_msg, MSG_TASK_FINISHED);

            step_++;
          }
          break;
        }
        case ReleaseObject: {
          operateHand(pub_gripper_trajectory, false);
          waiting_start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

          step_++;

          break;
        }
        case WaitForReleasing: {
          if(rclcpp::Clock(RCL_SYSTEM_TIME).now() - waiting_start_time > rclcpp::Duration::from_seconds(3.0))
          {
            sendMessage(pub_msg, MSG_TASK_FINISHED);
            step_++;
          }

          break;
        }
        case TaskFinished: {
          if(is_finished_) {
            RCLCPP_INFO(this->get_logger(), "Task finished!");
            step_ = Initialize;
          }

          break;
        }
      }

      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
  }
};


int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  InteractiveCleanupSample interactive_cleanup_sample;
  return interactive_cleanup_sample.run(argc, argv);
};
