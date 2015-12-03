#include "april_tag_detector.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <boost/foreach.hpp>

using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::ostringstream;

void push_tag_info_to_stream(ostringstream& stream,Eigen::Matrix4d tf, int id);


AprilTagDetector::AprilTagDetector(double image_width_px,double image_height_px,
                                       double tag_size_mtr,double fx,double fy, 
                                       bool print_debug) 
:tag_type(AprilTags::tagCodes36h11),// ************** Tag type : 36h11********//
 tag_detector(tag_type)
{
  this->image_width_px    = image_width_px;
  this->image_height_px   = image_height_px;
  this->tag_size_mtr      = tag_size_mtr;
  this->focal_length_x_px = fx;
  this->focal_length_y_px = fy;
  this->print_debug       = print_debug;
  frame_count             = 0;
}

bool AprilTagDetector::DetectTags(cv::Mat& frame, bool highlight_tag){
  bool tag_found = false;
  cv::Mat gray_frame;
  cv::cvtColor(frame,gray_frame,CV_BGR2GRAY);
  this->detected_tags = tag_detector.extractTags(gray_frame);
  if(!detected_tags.empty()){
    tag_found = true;
    if(print_debug){
      // cout<<"tags found "<<detected_tags.size()<<endl;
    }
    if(highlight_tag) {
      BOOST_FOREACH(AprilTags::TagDetection tag, detected_tags){
        tag.draw(frame);
      }
    }
  }
  frame_count++;
  return tag_found;
}

void AprilTagDetector::PrintStringLinespec(){
  cout<<"Frame definition: "<<endl;
  cout<<"camera frame: (z forward, x right, y down)"<<endl;
  cout<<"object frame: (x forward, y left, z up)"<<endl;
  cout<<"Tag information display format:"<<endl;
  cout<<"frame_number,number of tags,id_1,h111,h112,h113,h114,h121,h122,h123,h124,h131,h132,h133,h134,h141,h142,h143,h144,....."<<endl;
}

string AprilTagDetector::GetTagDataString(){
  string tag_info_string;
  vector<AprilTags::TagDetection>::iterator itr;
  ostringstream str_stream; // used for conversion of non-string type to string;
  str_stream<<frame_count<<","<<detected_tags.size();
  for(itr = detected_tags.begin(); itr != detected_tags.end(); itr++){
    AprilTags::TagDetection detection = *itr;
    Eigen::Matrix4d tf = detection.getRelativeTransform(tag_size_mtr,
                                                        focal_length_x_px,
                                                        focal_length_y_px,
                                                        image_width_px/2,
                                                        image_height_px/2);
    int id = detection.id;
    push_tag_info_to_stream(str_stream,tf,id);
  }
  tag_info_string = str_stream.str();
  if(print_debug){
    cout<<tag_info_string<<endl;
  }
  return tag_info_string;
}

vector<int> AprilTagDetector::GetTagId() {
  assert(detected_tags.size() != 0);
  vector<int> tag_ids;
  vector<AprilTags::TagDetection>::iterator itr;
  for(itr = detected_tags.begin(); itr != detected_tags.end(); itr++) {
    tag_ids.push_back(itr->id);
  }
  return tag_ids;
}

void push_tag_info_to_stream(ostringstream& str_stream,Eigen::Matrix4d tf, int id){
  string tag_info;
  str_stream<<","<<id;
  for(int i = 0;i <= 3;i++){
    for(int j = 0;j <=3 ;j++){
      str_stream<<","<<tf(i,j);
    }
  }
}

