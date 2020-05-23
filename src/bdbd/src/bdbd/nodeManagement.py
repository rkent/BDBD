#!/usr/bin/env python
# Service to manage startup and shutdown of launch files

import rospy
import roslaunch
from bdbd.srv import NodeCommand, NodeCommandResponse
from Queue import Queue
import sys

mainQueue = Queue()
class NodeManagement:
    def __init__(self):
        rospy.init_node("nodeManagement")
        self.launchers = {}
        self.service = rospy.Service('nodeManagement/command', NodeCommand, self.handle_request)
        rate = rospy.Rate(100)
        rospy.loginfo('Ready to process commands')
        while not rospy.is_shutdown():
            if not mainQueue.empty():
                filename, command, responseQueue = mainQueue.get()
                response = None

                try:
                    if command == 'start':
                        if filename in self.launchers:
                            response = 'active'
                        else:
                            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                            roslaunch.configure_logging(uuid)
                            self.launchers[filename] = roslaunch.parent.ROSLaunchParent(uuid, ['/home/kent/github/rkent/bdbd/src/bdbd/launch/' + filename + '.launch'])
                            self.launchers[filename].start()
                            response = 'started'
            
                    elif command == 'shutdown':
                        if filename in self.launchers:
                            self.launchers[filename].shutdown()
                            self.launchers.pop(filename)
                            response = 'shutdown'
                        else:
                            response = 'inactive'

                    elif command == 'status':
                        if filename in self.launchers:
                            response = 'active'
                        else:
                            response = 'inactive'

                    else:
                        response = 'invalid'

                except KeyboardInterrupt:
                    break
                except:
                    rospy.logerr(sys.exc_info()[1])
                    response = 'error'
                finally:
                    responseQueue.put(response)

            else:
                rate.sleep()

    def handle_request(self, req):
        rospy.loginfo('Got request: {} {}'.format(req.filename, req.command))
        responseQueue = Queue()
        mainQueue.put([req.filename, req.command, responseQueue])
        return(responseQueue.get())

def main():
    nodeManagement = NodeManagement()

if __name__ == '__main__':
    main()