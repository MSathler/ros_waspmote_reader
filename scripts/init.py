#!/usr/bin/env python

from sensors_publisher import wasp_reader

if __name__ == '__main__':
    try:
        wasp_obj = wasp_reader()
        wasp_obj.initiate()

    except rospy.ROSInterruptException:
		pass