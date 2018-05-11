/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/time.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/time_conversion.h"
#include "gflags/gflags.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_msgs/TFMessage.h"
#include <stack>

DEFINE_string(bag_filename, "", "Bag to publish.");

const int kQueueSize = 1;



template <typename MessagePtrType>
void PublishWithModifiedTimestamp(MessagePtrType message,
                                  const ros::Publisher& publisher,
                                  ros::Time stamp) {
  message->header.stamp = stamp;
  message->time_increment = 0.0;
  publisher.publish(message);
}

template <>
void PublishWithModifiedTimestamp<tf2_msgs::TFMessage::Ptr>(
    tf2_msgs::TFMessage::Ptr message, const ros::Publisher& publisher,
    ros::Time stamp) {
  for (auto& transform : message->transforms) {
    transform.header.stamp = stamp;
  }
  publisher.publish(message);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "This replays and publishes messages from a given bag file, modifying "
      "their header timestamps to match current ROS time.\n\n"
      "Messages are published in the same sequence and with the same delay "
      "they were recorded."
      "Contrary to rosbag play, it does not publish a clock, so time is"
      "hopefully smoother and it should be possible to reproduce timing"
      "issues.\n"
      "It only plays message types related to Cartographer.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";

  ros::init(argc, argv, "rosbag_publisher");
  ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  rosbag::Bag bag;
  bag.open(FLAGS_bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);
  ros::NodeHandle node_handle;
  bool use_sim_time;
  node_handle.getParam("/use_sim_time", use_sim_time);
  if (use_sim_time) {
    LOG(ERROR) << "use_sim_time is true but not supported. Expect conflicting "
                  "ros::Time and message header times or weird behavior.";
  }
  std::map<std::string, ros::Publisher> topic_to_publisher;
  for (const rosbag::ConnectionInfo* c : view.getConnections()) {
    const std::string& topic = c->topic;
    if (topic_to_publisher.count(topic) == 0) {
      ros::AdvertiseOptions options(c->topic, kQueueSize, c->md5sum,
                                    c->datatype, c->msg_def);
      topic_to_publisher[topic] = node_handle.advertise(options);
    }
  }
  ros::Duration(1).sleep();
  CHECK(ros::ok());

  std::stack<rosbag::MessageInstance> message_stack;

  ros::Time current_start = ros::Time::now();
  ros::Time bag_start = view.getBeginTime();
  ros::Time bag_end = view.getEndTime();
  ros::Time last_publish_time;
  ros::Duration bag_to_current = current_start - bag_start;
  ros::Duration bag_start_to_end = bag_end - bag_start;
  for (const rosbag::MessageInstance& message : view) {
    ros::Duration after_bag_start = message.getTime() - bag_start;
    if (!::ros::ok()) {
      break;
    }
    ros::Time planned_publish_time = current_start + after_bag_start;
    if(planned_publish_time < last_publish_time) {
      LOG(ERROR)<<"planned_publish_time < last_publish_time"<<planned_publish_time <<" < " <<last_publish_time;
    }
    last_publish_time = planned_publish_time;
    ros::Time::sleepUntil(planned_publish_time);

    ros::Publisher& publisher = topic_to_publisher.at(message.getTopic());
    if (message.isType<sensor_msgs::LaserScan>()) {
      PublishWithModifiedTimestamp(
          message.instantiate<sensor_msgs::LaserScan>(), publisher,
          planned_publish_time);
      message_stack.push(message);
    }  else if (message.isType<tf2_msgs::TFMessage>()) {
      PublishWithModifiedTimestamp(message.instantiate<tf2_msgs::TFMessage>(),
                                   publisher, planned_publish_time);
      message_stack.push(message);
    }
      else {
      //LOG(WARNING) << "Skipping message with type " << message.getDataType();
    }

    ros::Time current_time = ros::Time::now();
    double simulation_delay = cartographer::common::ToSeconds(
        cartographer_ros::FromRos(current_time) -
        cartographer_ros::FromRos(planned_publish_time));
    if (std::abs(simulation_delay) > 0.001) {
      LOG(WARNING) << "Playback delayed by " << simulation_delay
                   << " s. planned_publish_time: " << planned_publish_time
                   << " current_time: " << current_time;
    }
  }
  while(!message_stack.empty()) {
    const rosbag::MessageInstance& message = message_stack.top();
    ros::Duration after_bag_start = message.getTime() - bag_start;
    if (!::ros::ok()) {
      break;
    }
    ros::Time planned_publish_time =
        current_start + bag_start_to_end * 2.0 - after_bag_start + ros::Duration(0.1);
    if (planned_publish_time < last_publish_time) {
      LOG(ERROR) << "reverse: planned_publish_time < last_publish_time"
                 << planned_publish_time << " < " << last_publish_time;
    }
    last_publish_time = planned_publish_time;
    ros::Time::sleepUntil(planned_publish_time);

    ros::Publisher& publisher = topic_to_publisher.at(message.getTopic());
    if (message.isType<sensor_msgs::LaserScan>()) {
      PublishWithModifiedTimestamp(
          message.instantiate<sensor_msgs::LaserScan>(), publisher,
          planned_publish_time);
    } else if (message.isType<tf2_msgs::TFMessage>()) {
      PublishWithModifiedTimestamp(message.instantiate<tf2_msgs::TFMessage>(),
                                   publisher, planned_publish_time);
    } else {
      //LOG(WARNING) << "Skipping message with type " << message.getDataType();
    }

    ros::Time current_time = ros::Time::now();
    double simulation_delay = cartographer::common::ToSeconds(
        cartographer_ros::FromRos(current_time) -
            cartographer_ros::FromRos(planned_publish_time));
    if (std::abs(simulation_delay) > 0.001) {
      LOG(WARNING) << "Playback delayed by " << simulation_delay
                   << " s. planned_publish_time: " << planned_publish_time
                   << " current_time: " << current_time;
    }
    message_stack.pop();
    if (message_stack.size() % 100 == 0) {
      LOG(INFO) << "Remaining stack elements: " << message_stack.size();
    }
  }
  bag.close();

  ros::shutdown();
}
