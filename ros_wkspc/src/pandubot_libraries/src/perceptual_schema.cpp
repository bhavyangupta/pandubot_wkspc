#include "perceptual_schema.hpp"
#include "object_types.hpp"
#include <cv_bridge/cv_bridge.h>
#include <vector>

namespace pandubot_perceptual_schemas {
AprilTag::AprilTag() {

}

int AprilTag::GetPercept(cv::Mat frame){
  return 1;
}

ObjectDetection::ObjectDetection() {

}

std::vector<object_types::kObjectsSet> ObjectDetection::GetPercept
                                                               (cv::Mat frame) {

}

} // pandubot_perceptual_schemas