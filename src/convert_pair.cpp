/**
 * @file /dslam_ecto_vision/src/convert_pair.cpp
 * 
 * @brief Converts incoming CV:Mat pair to Image MSG
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

#include "../../include/dslam_ecto_vision/image_pair.hpp"

#include <opencv2/core/core.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace dslam_ecto_vision {
namespace cells {

/*****************************************************************************
 ** Cell
 *****************************************************************************/

class ConvertPair
{
public:
  typedef std::shared_ptr<ImagePair> ImagePairPtr;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<std::string>("frame_id", "Frame this data is associated with", "camera_default");
    params.declare<std::string>("encoding", "ROS image message encoding override.");
    params.declare<bool>("swap_rgb", "Swap the red and blue channels", false);
  }

  static void
  declare_io(const ecto::tendrils& params, ecto::tendrils& in, ecto::tendrils& out)
  {
    in.declare<ImagePairPtr>("pair", "Incoming image pair.").required(true);
    out.declare<ImageConstPtr>("left", "The converted and stamped left image.");
    out.declare<ImageConstPtr>("right", "The converted and stamped right images.");
  }


  void configure(const ecto::tendrils& params, const ecto::tendrils& in, const ecto::tendrils& out)
  {
    frame_id_ = params.get<std::string>("frame_id");
    encoding_ = params.get<std::string>("encoding");
    swap_rgb_ = params.get<bool>("swap_rgb");
    time_ = ecl::TimeStamp();
  }

  int process(const ecto::tendrils& in, const ecto::tendrils& out)
  {
    ImagePairPtr pair = in.get<ImagePairPtr>("pair");

    ImagePtr image_msg_left(new Image);
    ImagePtr image_msg_right(new Image);
    cv::Mat res_left;
    cv::Mat res_right;
    mat_left = pair->left.clone(); 
    mat_right = pair->right.clone(); 
    if(swap_rgb_)
    {
      cv::cvtColor(*mat_left, res_left, CV_BGR2RGB);
      cv::cvtColor(*mat_right, res_right, CV_BGR2RGB);
    }
    else
    {
      res_left = *mat_left;
      res_right = *mat_right;
    }
    toMsg(res_left, *image_msg_left); // Convert Left to MSG
    toMsg(res_right, *image_msg_right); // Convert Right to MSG
    if(encoding_.user_supplied())
    {
      image_msg_left->encoding = *encoding_;
      image_msg_right->encoding = *encoding_;
    }

    image_msg_left->header->stamp = time_;
    image_msg_left->header->frame_id = frame_id_;
    image_msg_right->header->stamp = time_;
    image_msg_right->header->frame_id = frame_id_;

    //Set output to converted and stamped msgs
    out.get<ImageConstPtr>("left") = image_msg_left;
    out.get<ImageConstPtr>("right") = image_msg_right;

    return ecto::OK;
  }

  private void toMsg(const cv::Mat& mat, sensor_msgs::Image& ros_image)
  {
    ros_image.height = mat.rows;
    ros_image.width = mat.cols;
    ros_image.encoding = mattype2enconding(mat.type());
    ros_image.is_bigendian = false;
    ros_image.step = mat.step;
    size_t size = mat.step * mat.rows;
    ros_image.data.resize(size);
    memcpy((char*) (&ros_image.data[0]), mat.data, size);
  }

  ecl::TimeStamp time_;
  std::string frame_id_;
  std::string encoding_; 
  bool swap_rgb_;
};

} // namespace cells
} // namespace dslam_ecto_vision

// This macro is required to register the cell with the module
// first argument: the module that it is to be part of
// second argument: the cell we want to expose in the module
// third argument: the name of that cell as seen in Python
// fourth argument: a description of what that cell does
ECTO_CELL(dslam_vision_utils, dslam_ecto_vision::cells::ConvertPair, "ConvertPair", "Average image pairs over a specified window.");
