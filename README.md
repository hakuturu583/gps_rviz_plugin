# gps_rviz_plugin
rviz plugin for /fix(sensor_msgs/NavSatFix) message visualization  

![visualization](https://github.com/hakuturu583/gps_rviz_plugin/blob/master/images/OverlayGpsDisplay.png)

## how to use  
1. get Google Static Map API key  
https://developers.google.com/maps/documentation/static-maps/get-api-key?hl=ja  
2. paste API key to config/google_static_map_api_key.yaml like this  
![google_static_map_api_key_yaml](https://github.com/hakuturu583/gps_rviz_plugin/blob/master/images/sample.png)  

## node  
### OverLayGpsDisplay  
#### Subscribe  
- fix  
  - type:[sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)  
  - gps sensor data

#### Publish  
- None  

#### Parameters  
- overlay_gps_display/google_static_map_api_key
  - type:string

#### Properties  
- Topic: topic name  
- Zoom: Google Static Map API Zoom parameter  
- Width: map image width  
- Height: map image height  
- Scale: Google Static Map API Scale parameter  
- History Length: trajectory length
- Map Type: Google Static Map API Map Type parameter
- Alpha: Alpha value of map image
- Position X: upleft X position of image  
- Position Y: upleft Y position of image
- Message Per Plot: number of messages per single plot  

#### Remarks
- Google Static Map API limitation (2017/11/08)  
  - normal API user (supported in this package)
    - 25000 requests in 24 hours
    - Maximum resolution 640 x 640   
  - Google Maps APIs Premium Plan user (not supported in this package)
    - The ratio in accordance with the annual number of purchase  
    - 2048 x 2048

<font color="Red">this package is not support over 25000 access in 24 hours!!!!</font>
