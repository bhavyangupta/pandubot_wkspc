//  Copyright [2015]
#ifndef APRIL_TAG_DETECTOR_HPP
#define APRIL_TAG_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

class AprilTagDetector{
private:
  double image_height_px;
  double image_width_px;
  double tag_size_mtr;
  double focal_length_x_px;
  double focal_length_y_px;
  bool   print_debug;
  AprilTags::TagCodes tag_type;
  AprilTags::TagDetector tag_detector;
  std::vector<AprilTags::TagDetection> detected_tags;
  long frame_count;
public:
  AprilTagDetector(double image_width,double image_height,double tag_size,
                     double fx_pixels, double fy_pixels, bool print_debug);
  bool DetectTags(cv::Mat& frame, bool highlight_tag);
  std::vector<int> GetTagId();
  void PrintStringLinespec();
  std::string GetTagDataString();
};

#endif  // APRIL_TAG_DETECTOR_HPP
