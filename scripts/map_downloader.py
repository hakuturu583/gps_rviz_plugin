#! /usr/bin/env python
import rospkg
import sys


def download_map(request_url):
    rospack = rospkg.RosPack()
    map_image_path = rospack.get_path('gps_rviz_plugin') + "/data/map.png"
    return 0

if __name__ == '__main__':
    pass
    #args = sys.argv
    #sys.exit(download_map(args[1]))
