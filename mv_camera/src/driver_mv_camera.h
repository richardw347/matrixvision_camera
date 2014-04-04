/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010 Jack O'Quin
 *  Copyright (c) 2012 Markus Achtelik, Luc Oth
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>

#include "dev_mv_camera.h"
#include <mv_camera/MVCameraConfig.h>
#include <mv_camera/PropertyMap.h>

/** @file

 @brief ROS driver interface for Matrix Vision digital cameras.

 */

namespace mv_camera
{

class MVCameraDriver
{
public:

  MVCameraDriver(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh);
  ~MVCameraDriver();

  /** device poll
   *
   * This function is called in the main loop to acquire frames.
   * It creates a new image message and gives its pointer to the read() function
   * which then queries the device for the image data. On success, the image is
   * published. Does not run concurrently with reconfig().
   */
  void poll(void);

  /** Driver initialization
   *
   * Define dynamic reconfigure callback, which gets called immediately with
   * level 0xffffffff.  The reconfig() method will set initial parameter values,
   * then open the device if it can.
   */
  void setup(void);

  /** driver termination */
  void shutdown(void);

  void pollSingle(std::string& outputString);

  bool pollPropertyMapCallback(PropertyMap::Request &req, PropertyMap::Response &res);
  std::string getPropertyData(const mvIMPACT::acquire::Property& prop);

  std::pair<int,std::string> setProperty(const std::string & key, const std::string & value);

private:

  void closeCamera();

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   */
  bool openCamera(MVCameraConfig &newconfig);

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void publish(const sensor_msgs::ImagePtr &image);

  /** Read camera data .
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool read(sensor_msgs::ImagePtr &image);

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void reconfig(MVCameraConfig &newconfig, uint32_t level);

  void processParameterList();

  /** Non-recursive mutex for serializing callbacks with device polling. */
  boost::mutex mutex_;

  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;        // true when reconfig() running

  ros::NodeHandle priv_nh_;             // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  std::string camera_name_;             // camera name

  /** mv_camera camera device interface */
  boost::shared_ptr<mv_camera::MVCamera> dev_;

  /** dynamic parameter configuration */
  MVCameraConfig config_;
  dynamic_reconfigure::Server<mv_camera::MVCameraConfig> srv_;
  ros::Rate cycle_;                     // polling rate when closed

  /** camera calibration information */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  bool calibration_matches_;            // CameraInfo matches video mode

  /** image transport interfaces */
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_;

  bool continuousPoll_;

  ros::ServiceServer serviceServer_;

  bool processedParameterList_;

};
// end class MVCameraDriver

}
;
// end namespace mv_camera
