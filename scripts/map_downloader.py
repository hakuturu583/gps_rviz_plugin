#! /usr/bin/env python
import rospkg
import sys
import requests


def download_map(request_url):
    rospack = rospkg.RosPack()
    map_image_path = rospack.get_path('gps_rviz_plugin') + "/config/map.png"
    try:
        f=open(map_image_path,'wb')
        f.write(requests.get(request_url).content)
        f.close()
        return 0
    except:
        f.close()
        return -1

if __name__ == '__main__':
    pass
