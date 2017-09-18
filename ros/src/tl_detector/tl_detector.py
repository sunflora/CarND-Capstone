#!/usr/bin/env python

import rospy

from tl_detector_gt import TLDetector
#from tl_detector_image import TLDetector

if __name__ == '__main__':
    try:
    	#rospy.logerr("Inside TLDetecto.")
        detector = TLDetector()
        #rospy.logerr("TLDetector last_wp:", detector.last_wp)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
