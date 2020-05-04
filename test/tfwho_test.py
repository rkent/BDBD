#!/usr/bin/python2

# Send a face to be recognized. This is a ros node. First run node tfwho.
import rospy
import numpy as np
import logging
from cv_bridge import CvBridge

def main():
    bridge = CvBridge()
    logging.basicConfig(level='DEBUG')
    rospy.init_node('tfwho_test')

    # get the test image
    data = np.load('../src/bdbd/src/bdbd/bddata/faces.npz', allow_pickle=True)
    X = data['arr_0']
    print(X.dtype)

    # 
    rospy.loginfo('Starting initial embedding calculation to prime the pump')

if __name__ == '__main__':
    main()