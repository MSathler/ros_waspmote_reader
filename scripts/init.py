#!/usr/bin/env python

from sensors_publisher import wasp_reader
from co2_publisher import co2_reader

import rospy

if __name__ == '__main__':

    try:
        wasp_obj = co2_reader()
#        wasp_obj = wasp_reader()
        wasp_obj.initiate()

    except rospy.ROSInterruptException:
		pass
