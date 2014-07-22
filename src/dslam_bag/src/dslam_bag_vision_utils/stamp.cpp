/**
 * @file /dslam_ecto_vision/src/stamp.cpp
 * 
 * Takes in a CV::Mat and outputs a stamped sensor_msgs/Image msg. 
 * 
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecto/ecto.hpp>
#include <iostream>
#include <memory>
#include <ecl/time.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace dslam_ecto_vision
{
  using ecto::tendrils;
  using std::string;
  using namespace sensor_msgs;
  namespace enc = sensor_msgs::image_encodings;
/*****************************************************************************
 ** Cell
 *****************************************************************************/

class Mat2ImageStamped
{
public:

  void
  toImageMsg(const cv::Mat& mat, sensor_msgs::Image& ros_image)
  {
    ros_image.height = mat.rows;
    ros_image.width = mat.cols;
    switch(mat.type())
    {
      case CV_8U:
        ros_image.encoding = enc::MONO8;
      default:
        ros_image.encoding = enc::RGB8;
    } 
    ros_image.is_bigendian = false;
    ros_image.step = mat.step;
    size_t size = mat.step * mat.rows;
    ros_image.data.resize(size);
    memcpy((char*) (&ros_image.data[0]), mat.data, size);
  }
  
  static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("frame_id", "Frame this data is associated with", "default_frame");
      p.declare<std::string>("encoding", "ROS image message encoding override.");
      p.declare<bool>("swap_rgb", "Swap the red and blue channels", false);

    }

    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<ecl::TimeStamp>("time", "Stamp this time on the outgoing message.");
      i.declare<cv::Mat>("image", "A cv::Mat.");
      o.declare<ImageConstPtr>("image", "A sensor_msg::Image message.");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)    
    {
      mat_ = i["image"];
      cloud_msg_out_ = o["image"];
      frame_id_ = p.get<std::string>("frame_id");
      header_.frame_id = frame_id_;
      encoding_ = p["encoding"];
      swap_rgb_ = p.get<bool>("swap_rgb");
    }
    int
    process(const tendrils& i, const tendrils& o)
    {
      timestamp_ = i.get<ecl::TimeStamp>("time");
      ImagePtr image_msg(new Image);
      cv::Mat mat;
      if (swap_rgb_)
        cv::cvtColor(*mat_, mat, CV_BGR2RGB);
      else
        mat = *mat_;

      toImageMsg(mat, *image_msg);
      if (encoding_.user_supplied())
      {
        image_msg->encoding = *encoding_;
      }

      header_.seq++;
      header_.stamp = ros::Time(timestamp_.sec(), timestamp_.nsec());
      image_msg->header = header_;
      *cloud_msg_out_ = image_msg;
      return ecto::OK;
    }
    ecl::TimeStamp timestamp_;
    std_msgs::Header header_;
    std::string frame_id_;
    ecto::spore<ImageConstPtr> cloud_msg_out_;
    ecto::spore<cv::Mat> mat_;
    ecto::spore<std::string> encoding_;
    bool swap_rgb_;




}; //
} // namespace dslam_ecto_vision
// This macro is required to register the cell with the module
// first argument: the module that it is to be part of
// second argument: the cell we want to expose in the module
// third argument: the name of that cell as seen in Python
// fourth argument: a description of what that cell does
ECTO_CELL(dslam_bag_vision_utils, dslam_ecto_vision::Mat2ImageStamped, "Mat2ImageStamped", "Stamps the incoming message.");
