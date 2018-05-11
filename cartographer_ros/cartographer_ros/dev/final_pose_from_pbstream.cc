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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cartographer/transform/transform.h"
#include "cartographer/common/math.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"

DEFINE_string(pbstream_filename, "",
"Proto stream file containing the pose graph.");

namespace cartographer_ros {
namespace {



void Run(const std::string& pbstream_filename) {
  cartographer::io::ProtoStreamReader reader(pbstream_filename);
  cartographer::mapping::proto::PoseGraph pose_graph_proto;
  CHECK(reader.ReadProto(&pose_graph_proto));
  const cartographer::mapping::proto::Trajectory& last_trajectory_proto =
      *pose_graph_proto.mutable_trajectory()->rbegin();
  const cartographer::transform::TransformInterpolationBuffer
      transform_interpolation_buffer(last_trajectory_proto);

  //cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::transform::Rigid3d> node_poses;

  int max_node_index = -1;
  cartographer::transform::Rigid3d final_node_transform;
  for (const cartographer::mapping::proto::Trajectory::Node& node_proto : last_trajectory_proto.node()) {
    if(node_proto.node_index() > max_node_index) {
      max_node_index = node_proto.node_index();
      final_node_transform = cartographer::transform::ToRigid3(node_proto.pose());
    }
  }
  LOG(INFO)<<"final node transform: "<<final_node_transform;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
          "This compares a trajectory from a bag file against the "
          "last trajectory in a pbstream file.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  ::cartographer_ros::Run(FLAGS_pbstream_filename);
}