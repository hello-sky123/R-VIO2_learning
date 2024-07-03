/**
 * This file is part of R-VIO2.
 *
 * Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang
 * <ghuang@udel.edu> For more information see <http://github.com/rpng/R-VIO2>
 *
 * R-VIO2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * R-VIO2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "rvio2/System.h"

class ImageGrabber {
 public:
  explicit ImageGrabber(RVIO2::System* pSys): mpSys(pSys) {}

  void GrabImage(const sensor_msgs::ImageConstPtr& msg) const;

  RVIO2::System* mpSys;
};

class ImuGrabber {
 public:
  explicit ImuGrabber(RVIO2::System* pSys): mpSys(pSys) {}

  void GrabImu(const sensor_msgs::ImuConstPtr& msg) const;

  RVIO2::System* mpSys;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) const {
  static int lastseq = -1;
  // 如果不是第一帧图像，检查图像序列号是否连续
  if ((int)msg->header.seq != lastseq + 1 && lastseq != -1)
    ROS_DEBUG("Image message drop! curr seq: %d expected seq: %d.",
              msg->header.seq, lastseq + 1);
  lastseq = msg->header.seq;

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // 将ROS图像消息转换为OpenCV图像，如果图像类型相同，则不会复制数据
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  auto* pData = new RVIO2::ImageData(); // InputBuffer封装的图像数据结构，包含图像和时间戳
  pData->Image = cv_ptr->image.clone(); // 深拷贝图像数据
  pData->Timestamp = cv_ptr->header.stamp.toSec(); // 时间戳，单位秒
  // 将图像数据保存到InputBuffer类的mlImuFIFO队列中（是一个list）
  mpSys->PushImageData(pData);

  mpSys->run(); // R-VIO2的主要处理流程，r-vio里是MonoVIO()函数
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg) const {
  static int lastseq = -1;
  // 如果不是第一帧IMU数据，检查IMU数据序列号是否连续
  if ((int)msg->header.seq != lastseq + 1 && lastseq != -1)
    ROS_DEBUG("IMU message drop! curr seq: %d expected seq: %d.",
              msg->header.seq, lastseq + 1);
  lastseq = msg->header.seq;

  // 将ROS消息中的角速度转换为Eigen类型
  Eigen::Vector3f angular_velocity(msg->angular_velocity.x,
                                   msg->angular_velocity.y,
                                   msg->angular_velocity.z);

  // 将ROS消息中的线性加速度转换为Eigen类型
  Eigen::Vector3f linear_acceleration(msg->linear_acceleration.x,
                                      msg->linear_acceleration.y,
                                      msg->linear_acceleration.z);

  double currtime = msg->header.stamp.toSec();

  auto* pData = new RVIO2::ImuData();
  pData->AngularVel = angular_velocity;
  pData->LinearAccel = linear_acceleration;
  pData->Timestamp = currtime;

  static double lasttime = -1;
  if (lasttime != -1)
    pData->TimeInterval = currtime - lasttime; // 计算时间间隔
  else
    pData->TimeInterval = 0;
  lasttime = currtime;

  mpSys->PushImuData(pData);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rvio2_mono");

  ros::start(); // 初始化ros客户端，并启动节点

  RVIO2::System Sys(argv[1]); // 第2个参数是配置文件的路径，读取配置文件

  ImageGrabber igb1(&Sys); // 创建图像和IMU数据的抓取类的实例
  ImuGrabber igb2(&Sys);

  // 订阅IMU和图像消息，将对应的的数据存到InputBuffer类的队列中
  ros::NodeHandle nodeHandler;
  ros::Subscriber imu_sub =
      nodeHandler.subscribe("/imu0", 100, &ImuGrabber::GrabImu, &igb2);
  ros::Subscriber image_sub = nodeHandler.subscribe(
      "/cam0/image_raw", 1, &ImageGrabber::GrabImage, &igb1);

  ros::spin();

  ros::shutdown();

  return 0;
}
