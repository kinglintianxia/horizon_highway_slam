// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               Livox@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// For save pose
std::vector<nav_msgs::Odometry::ConstPtr> odom_vec;
bool SAVE_ODOM = false;

// receive odomtry
void OdometrySubHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    // Just push_back in vector.
    odom_vec.push_back(laserOdometry);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_sub");
    ros::NodeHandle nh;

    nh.param<bool>("save_odom", SAVE_ODOM, false);
    // Save directory.
    std::string save_dir;
    nh.param<std::string>("save_dir", save_dir, "./");

    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
        "/livox_odometry", 100, OdometrySubHandler);

    ros::spin();

    // Save pose to file
    if (SAVE_ODOM)
    {
        std::string pose_fname = save_dir + "/laser_pose.txt";
        std::FILE *pose_file = fopen(pose_fname.c_str(), "w");
        for (auto stamped_odom : odom_vec)
        {
            std::fprintf(pose_file, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                         stamped_odom->header.stamp.toSec(), // time
                         stamped_odom->pose.pose.position.x,
                         stamped_odom->pose.pose.position.y,
                         stamped_odom->pose.pose.position.z,
                         stamped_odom->pose.pose.orientation.x,
                         stamped_odom->pose.pose.orientation.y,
                         stamped_odom->pose.pose.orientation.z,
                         stamped_odom->pose.pose.orientation.w);
        }
        // Close file
        fclose(pose_file);
    }

    return 0;
}
