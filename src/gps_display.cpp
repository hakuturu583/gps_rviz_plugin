#include "gps_display.h"

namespace gps_rviz_plugin
{
  gps_display::gps_display()
  {
    map_image_path_ = ros::package::getPath("gps_rviz_plugin") + "/data/map.png";
    load_map_downloader_script();
    zoom_property_ = new rviz::IntProperty("Zoom Property", 19, "zoom of map", this, SLOT(updateGooleMapAPIProperty()));
    zoom_property_->setMax(22);
    zoom_property_->setMin(0);
    width_property_  = new rviz::IntProperty("Width", 640, "request map image width", this, SLOT(updateGooleMapAPIProperty()));
    width_property_->setMax(640);
    width_property_->setMin(0);
    height_property_ = new rviz::IntProperty("Height", 640, "request map image height", this, SLOT(updateGooleMapAPIProperty()));
    height_property_->setMax(640);
    height_property_->setMin(0);
    scale_property_ = new rviz::IntProperty("Scale", 1, "request map image scale", this, SLOT(updateGooleMapAPIProperty()));
    scale_property_->setMax(2);
    scale_property_->setMin(1);
    api_key_property_ = new rviz::StringProperty("API Key", "", "Google Static Map API Key", this, SLOT(updateGooleMapAPIProperty()));
    maptype_property_ = new rviz::EnumProperty("Map Type", "roadmap", "map type", this, SLOT(updateGooleMapAPIProperty()));
    maptype_property_->addOption("roadmap", ROADMAP);
    maptype_property_->addOption("terrain", TERRAIN);
    maptype_property_->addOption("satellite", SATELLITE);
    maptype_property_->addOption("hybrid", HYBRID);
    alpha_property_ = new rviz::FloatProperty("Alpha", 0.8 , "image alpha", this, SLOT(updateDisplayProperty()));
    alpha_property_->setMax(1);
    alpha_property_->setMin(0);
  }

  gps_display::~gps_display()
  {
    delete zoom_property_;
    delete width_property_;
    delete height_property_;
    delete scale_property_;
    delete alpha_property_;
    delete api_key_property_;
    delete maptype_property_;
  }

  void gps_display::download_map(std::string request_url)
  {
    PyObject* args = PyTuple_New(1);
    PyObject* kw_args = PyDict_New();
    PyObject* request_url_str = PyString_FromString(request_url.c_str());
    PyTuple_SetItem(args, 0, request_url_str);
    PyObject* responce = PyObject_Call(map_downloader_function_, args, kw_args);
    Py_DECREF(args);
    Py_DECREF(kw_args);
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
      std::string request_url;
      build_request_url(msg, request_url);
      //download_map(request_url);
      cv::Mat map_image = cv::imread(map_image_path_);
      if(!overlay_)
      {
        static int count = 0;
        rviz::UniformStringStream ss;
        ss << "OverlayImageDisplayObject" << count++;
        overlay_.reset(new OverlayObject(ss.str()));
        overlay_->show();
      }
      if (overlay_)
      {
        overlay_->setDimensions(width_property_->getInt(), height_property_->getInt());
        overlay_->setPosition(0,0);
      }
      overlay_->updateTextureSize(width_property_->getInt(),height_property_->getInt());
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_);
      for (int i = 0; i < overlay_->getTextureWidth(); i++)
      {
        for (int j = 0; j < overlay_->getTextureHeight(); j++)
        {
          QColor color(map_image.data[j * map_image.step + i * map_image.elemSize() + 2],
                       map_image.data[j * map_image.step + i * map_image.elemSize() + 1],
                       map_image.data[j * map_image.step + i * map_image.elemSize() + 0],
                       alpha_property_->getFloat() * 255.0);
          Hud.setPixel(i, j, color.rgba());
        }
      }
    }
    catch(...)
    {
      ROS_ERROR_STREAM("failed to request map");
    }
  }

  void gps_display::updateGooleMapAPIProperty()
  {

  }

  void gps_display::updateDisplayProperty()
  {

  }

  void gps_display::load_map_downloader_script()
  {
    std::string map_downloader_path = ros::package::getPath("gps_rviz_plugin") + "/scripts/map_downloader.py";
    std::ifstream ifs(map_downloader_path.c_str());
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

  bool gps_display::build_request_url(const sensor_msgs::NavSatFix::ConstPtr& msg, std::string& request_url)
  {
    request_url = "https://maps.googleapis.com/maps/api/staticmap?";
    if(api_key_property_->getStdString() == "")
    {
      this->setStatus(rviz::StatusProperty::Level::Error, "APIKey", "API key is not exist");
      return false;
    }
    this->setStatus(rviz::StatusProperty::Level::Ok, "APIKey", "API key is exist");
    std::string center_request = "center=" + std::to_string(msg->latitude) + "," + std::to_string(msg->longitude);
    request_url = request_url + center_request;
    std::string markers_request = "&markers=color:red%7Clabel:C%7C" + std::to_string(msg->latitude) + "," + std::to_string(msg->longitude);
    request_url = request_url + markers_request;
    std::string zoom_request = "&zoom=" + std::to_string(zoom_property_->getInt());
    request_url = request_url + zoom_request;
    std::string size_request = "&size=" + std::to_string(width_property_->getInt()) + "x" + std::to_string(height_property_->getInt());
    request_url = request_url + size_request;

    std::string key_request = "&key=" + api_key_property_->getStdString();
    request_url = request_url + key_request;
    return true;
   }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gps_rviz_plugin::gps_display, rviz::Display)
