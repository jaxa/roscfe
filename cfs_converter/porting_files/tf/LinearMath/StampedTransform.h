/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef STAMPED_TRANSFORM_H
#define STAMPED_TRANSFORM_H

#include "../../std_msgs/RosTime.h"
#include "Transform.h"
#include <string>
extern "C" {
namespace tf {
class StampedTransform : public tf::Transform {
public:
  // Convert
  // ros::Time stamp_;
  RosTime stamp_;
  std::string frame_id_;
  std::string child_frame_id_;
  // StampedTransform(const tf::Transform& input, const ros::Time& timestamp,
  // const std::string & frame_id, const std::string & child_frame_id):
  StampedTransform(const tf::Transform &input, const RosTime &timestamp,
                   const std::string &frame_id,
                   const std::string &child_frame_id)
      : tf::Transform(input), stamp_(timestamp), frame_id_(frame_id),
        child_frame_id_(child_frame_id){};

  StampedTransform(){};

  void setData(const tf::Transform &input) {
    *static_cast<tf::Transform *>(this) = input;
  };
};
}
}

#endif // STAMPED_TRANSFORM_H