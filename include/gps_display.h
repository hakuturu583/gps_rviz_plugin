#ifndef GPS_PANEL_H
#define GPS_PANEL_H

#define ROADMAP 0
#define TERRAIN 1
#define SATELLITE 2
#define HYBRID 3

#define PNG 0
#define GIF 1
#define JPEG 2

#include "overlay_utils.h"

//headers for ROS
#include <ros/ros.h>
#include <ros/package.h>

//headers for Qt
#include <QWidget>

//headers for rviz
#include <rviz/message_filter_display.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/uniform_string_stream.h>

//headers for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//headers for messages
#include <sensor_msgs/NavSatFix.h>

//headers for python
#include <Python.h>

//headers for stl
#include <string>

namespace gps_rviz_plugin
{
  class gps_display: public rviz::MessageFilterDisplay<sensor_msgs::NavSatFix>
  {
    Q_OBJECT
    public:
      gps_display();
      virtual ~gps_display();
    protected:
      virtual void onInitialize();
      virtual void reset();
    private:
      void processMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);
      void download_map(std::string request_url);
      void load_map_downloader_script();
      bool build_request_url(const sensor_msgs::NavSatFix::ConstPtr& msg, std::string& request_url);
      rviz::IntProperty* zoom_property_;
      rviz::IntProperty* width_property_;
      rviz::IntProperty* height_property_;
      rviz::IntProperty* scale_property_;
      rviz::FloatProperty* alpha_property_;
      rviz::StringProperty* api_key_property_;
      rviz::EnumProperty* maptype_property_;
      PyObject* map_downloader_function_;
      std::string map_image_path_;
      OverlayObject::Ptr overlay_;
    private Q_SLOTS:
      void updateGooleMapAPIProperty();
      void updateDisplayProperty();
  };
}
#endif
