// Copyright [2015]
#ifndef PERCEPTUAL_SCHEMA_HPP
#define PERCEPTUAL_SCHEMA_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "object_types.hpp"

/**
 * Template class for all types of perceptual schema. Percept is the processed
 * value of the raw sensor data frame. Template arguments are types of the 
 * sensor frame and percept respectively.
 */
template<typename SENSOR_T, typename PERCEPT_T>
class PerceptualSchema {
 public:
  PerceptualSchema() {};
  virtual PERCEPT_T GetPercept(SENSOR_T raw_frame)=0;
};

namespace pandubot_perceptual_schemas {

/**
 * Perceptual schema specialisation for april tag detection. The sensor data
 * type is cv::Mat image frame and percept is the vector<int> of tag_ids.
 */
class AprilTag : public PerceptualSchema <cv::Mat, int> {
 public:
  AprilTag();
  int GetPercept(cv::Mat frame);
};

/** 
 * Perceptual schema specialisation for object detection. Sensor data is a 
 * captured frame of type cv::Mat and percept is vector<kObjectsSet> of object 
 * types.
 */
class ObjectDetection : public PerceptualSchema<cv::Mat,
                                      std::vector<object_types::kObjectsSet> > {
 public:
  ObjectDetection();
  std::vector<object_types::kObjectsSet> GetPercept(cv::Mat frame);
};

}  // pandubot_perceptual_schemas
#endif  // PERCEPTUAL_SCHEMA_HPP
