#include "gps_display.h"

namespace gps_rviz_plugin
{
  gps_display::gps_display()
  {
    load_map_downloader_script();
    zoom_property_ = new rviz::IntProperty("Zoom Property", 19, "zoom of map", this, SLOT(updateZoomProperty()));
    zoom_property_->setMax(22);
    zoom_property_->setMin(0);
    width_property_  = new rviz::IntProperty("Width", 640, "request map image width", this, SLOT(updateWidthProperty()));
    width_property_->setMax(640);
    width_property_->setMin(0);
    height_property_ = new rviz::IntProperty("Height", 640, "request map image height", this, SLOT(updateHeightProperty()));
    height_property_->setMax(640);
    height_property_->setMin(0);
    scale_property_ = new rviz::IntProperty("Scale", 1, "request map image scale", this, SLOT(updateScaleProperty()));
    scale_property_->setMax(2);
    scale_property_->setMin(1);
    api_key_property_ = new rviz::StringProperty("API Key", "", "Google Static Map API Key", this, SLOT(updateAPIKeyProperty()));
    maptype_property_ = new rviz::EnumProperty("Map Type", "roadmap", "map type", this, SLOT(updateMapTypeProperty()));
    maptype_property_->addOption("roadmap", ROADMAP);
    maptype_property_->addOption("terrain", TERRAIN);
    maptype_property_->addOption("satellite", SATELLITE);
    maptype_property_->addOption("hybrid", HYBRID);
  }

  gps_display::~gps_display()
  {

  }

  void gps_display::download_map(std::string request_url)
  {
    /*
    boost::asio::ip::tcp::iostream map_image_stream("maps.googleapis.com/maps/api/staticmap?center=Brooklyn+Bridge,New+York,NY&zoom=13&size=600x300&maptype=roadmap&markers=color:blue|label:S|40.702147,-74.015794&markers=color:green|label:G|40.711614,-74.012318&markers=color:red|label:C|40.718217,-73.998284&key=AIzaSyAMNQjuwTLGsZ3mLVUq5S6lmWA3cQksJaY", "https");
    if(!map_image_stream)
    {
      ROS_ERROR_STREAM("Unable to fetch map image.");
    }
    */
    PyObject* args = PyTuple_New(1);
    PyObject* kw_args = PyDict_New();
    PyObject* request_url_str = PyString_FromString(request_url.c_str());
    PyTuple_SetItem(args, 0, request_url_str);
    PyObject* responce = PyObject_Call(map_downloader_function_, args, kw_args);
    //Py_DECREF(args);
    //Py_DECREF(kw_args);
  }

  void gps_display::onInitialize()
  {
    MFDClass::onInitialize();
  }

  void gps_display::reset()
  {
    MFDClass::reset();
  }

  void gps_display::processMessage(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    try
    {
      download_map("https");
    }
    catch(...)
    {
      ROS_ERROR_STREAM("failed to request map");
    }
  }

  void gps_display::updateZoomProperty()
  {

  }

  void gps_display::updateMapTypeProperty()
  {

  }

  void gps_display::updateWidthProperty()
  {

  }

  void gps_display::updateHeightProperty()
  {

  }

  void gps_display::updateScaleProperty()
  {

  }

  void gps_display::updateAPIKeyProperty()
  {

  }

  void gps_display::load_map_downloader_script()
  {
    std::string map_downloader_path_ = ros::package::getPath("gps_rviz_plugin") + "/scripts/map_downloader.py";
    std::ifstream ifs(map_downloader_path_.c_str());
    int begin = static_cast<int>(ifs.tellg());
    ifs.seekg(0, ifs.end);
    int end = static_cast<int>(ifs.tellg());
    int size = end - begin;
    ifs.clear();
    ifs.seekg(0, ifs.beg);
    char* map_downloader_script_ = new char[size + 1];
    map_downloader_script_[size] = '\0';
    ifs.read(map_downloader_script_, size);
    Py_Initialize();
    PyRun_SimpleString(map_downloader_script_);
    PyObject* script_obj = PyModule_GetDict(PyImport_ImportModule("__main__"));
    map_downloader_function_ = PyDict_GetItemString(script_obj, "download_map");
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gps_rviz_plugin::gps_display, rviz::Display)
