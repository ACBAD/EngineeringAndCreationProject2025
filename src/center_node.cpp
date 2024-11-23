#include <ros/ros.h>
#include <iostream>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
uint8_t sound_cmd = 0;
bool cmd_changed = false;
move_base_msgs::MoveBaseGoal start_pose;
move_base_msgs::MoveBaseGoal base_goal;
move_base_msgs::MoveBaseGoal right_store;
move_base_msgs::MoveBaseGoal left_store;
uint8_t left_position = 0;
uint8_t right_position = 0;
uint8_t item_shape = 0;

void soundCallback(const std_msgs::UInt8 msg) {
  if(sound_cmd != msg.data)
    cmd_changed = true;
  sound_cmd = msg.data;
}

void recognizeCallback(const std_msgs::UInt8 msg) {
  left_position = msg.data/10;
  right_position = msg.data%10;
  item_shape = msg.data/100;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "center_node");
  MoveBaseClient ac("move_base", true);
  ros::NodeHandle node_handle;
  ros::Subscriber cmd_sub = node_handle.subscribe("/sound_cmd", 2, soundCallback);
  ros::Subscriber recognize_sub = node_handle.subscribe("/recognize", 2, recognizeCallback);
  const ros::Publisher sound_pub = node_handle.advertise<std_msgs::UInt8>("/play", 2);

  right_store.target_pose.header.frame_id = "map";
  left_store.target_pose.header.frame_id = "map";
  base_goal.target_pose.header.frame_id = "map";
  start_pose.target_pose.header.frame_id = "map";

  start_pose.target_pose.pose.position.x = -0.3155733346939087;
  start_pose.target_pose.pose.position.y = 0.01106572151184082;
  start_pose.target_pose.pose.position.z = 0;
  start_pose.target_pose.pose.orientation.x = 0;
  start_pose.target_pose.pose.orientation.y = 0;
  start_pose.target_pose.pose.orientation.z = 0.5624274813080514;
  start_pose.target_pose.pose.orientation.w = 0.8268466171361418;

  base_goal.target_pose.pose.position.x = 0.1624593734741211;
  base_goal.target_pose.pose.position.y = 1.2944345474243164;
  base_goal.target_pose.pose.position.z = 0;
  base_goal.target_pose.pose.orientation.x = 0;
  base_goal.target_pose.pose.orientation.y = 0;
  base_goal.target_pose.pose.orientation.z = 0.56240042414644;
  base_goal.target_pose.pose.orientation.w = 0.8268650209797875;

  right_store.target_pose.pose.position.x = 3.0733237266540527;
  right_store.target_pose.pose.position.y = -0.7802897691726685;
  right_store.target_pose.pose.position.z = 0;
  right_store.target_pose.pose.orientation.x = 0;
  right_store.target_pose.pose.orientation.y = 0;
  right_store.target_pose.pose.orientation.z = -0.8261343476802377;
  right_store.target_pose.pose.orientation.w = 0.5634731933135311;

  left_store.target_pose.pose.position.x = 3.4320435523986816;
  left_store.target_pose.pose.position.y = 0.2914857566356659;
  left_store.target_pose.pose.position.z = 0;
  left_store.target_pose.pose.orientation.x = 0;
  left_store.target_pose.pose.orientation.y = 0;
  left_store.target_pose.pose.orientation.z = 0.5695906677489723;
  left_store.target_pose.pose.orientation.w = 0.821928507361364;

  while (ac.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for move base server...");
  }
  ROS_WARN("HayaseYuuka!");
  while (ros::ok()) {
    ros::spinOnce();
    if(cmd_changed) {
      if(sound_cmd == 1) {
        std_msgs::UInt8 say_what;
        // Main Logic
        ROS_INFO("Accept start request");
        say_what.data = 0;
        sound_pub.publish(say_what);
        base_goal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(base_goal);
        ROS_INFO("Base goal sent, waiting for navi result...");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Reach base goal");
          // Recongnize shape
          say_what.data = 1;
          sound_pub.publish(say_what);
          ROS_INFO("Start recongnize shape...");
          for(int i = 0;i < 10; i++) {
            sleep(1);
            ros::spinOnce();
          }
          if(item_shape == 0) {
            ROS_DEBUG("Timeout!");
            item_shape = 2;
          }
          ROS_INFO("recognize OK");
          say_what.data = 1 + item_shape;
          sound_pub.publish(say_what);
        }
        else {
          ROS_WARN("Failed go to base");
          // TODO
        }
        right_store.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(right_store);
        ROS_INFO("Right goal sent, waiting for nav result...");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Reach right goal, start mission");
          // Excute clip
          ROS_INFO("Start recongnize shape...");
          for(int i = 0;i < 5; i++) {
            sleep(1);
            ros::spinOnce();
          }
          if(right_position == 0) {
            ROS_DEBUG("Timeout!");
            right_position = 3;
          }
          ROS_INFO("recognize OK");
          // TODO
        }else {
          ROS_WARN("Failed navigation, go back");
        }
        start_pose.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(start_pose);
        ROS_INFO("Return cmd sent, wait for result...");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Return success");
          // Excute put down
          // TODO
        }
        else {
          ROS_WARN("Return failed");
          // Clear hand
          // TODO
        }
        left_store.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(left_store);
        ROS_INFO("Left goal sent, waiting for result...");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Reach left goal, start mission");
          // Excute clip
          ROS_INFO("Start recongnize shape...");
          for(int i = 0;i < 5; i++) {
            sleep(1);
            ros::spinOnce();
          }
          if(left_position == 0) {
            ROS_DEBUG("Timeout!");
            left_position = 1;
          }
          ROS_INFO("recognize OK");
          // TODO
        }
        else {
          ROS_WARN("Failed navigation, go back");
        }
        ac.sendGoal(start_pose);
        ROS_INFO("Return cmd sent, wait for result...");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Return success");
          // Excute put down
          // TODO
        }
        else {
          ROS_WARN("Return failed");
          // Clear hand
          // TODO
        }
        ROS_INFO("All clear");
        say_what.data = 4;
        sound_pub.publish(say_what);
      }
      cmd_changed = false;
    }
  }
}

