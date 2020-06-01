#!/usr/bin/env python
'''
    Service to manage startup and shutdown of launch files

    valid commands are:
        start: start now
        startN: start in N seconds (example: start3 starts in 3 seconds)
        stop: shutdown
        shutdown: shutdown
        status: tell node status
'''

import rospy
import roslaunch
import time
from bdbd.srv import NodeCommand, NodeCommandResponse
from Queue import Queue
import sys

Behaviors = {
    'explore': [
        't265min',
        'speech',
        'gamepad',
        'drivers',
        'transforms',
        'sr305min',
        'detectBlocking',
        'explore',
    ]
}

mainQueue = Queue()
class NodeManagement:
    def __init__(self):
        rospy.init_node("bdnodes")
        self.launchers = {}
        self.launchService = rospy.Service('~launch', NodeCommand, self.handle_launch)
        self.behaviorService = rospy.Service('~behavior', NodeCommand, self.handle_behavior)
        rate = rospy.Rate(100)
        rospy.loginfo('Ready to process commands')
        while not rospy.is_shutdown():
            if not mainQueue.empty():
                type, name, command, responseQueue = mainQueue.get()
                response = None

                try:
                    if type == 'launch':
                        response = self.process_launch(name, command)
                
                    elif type == 'behavior':
                        response = self.process_behavior(name, command)

                    else:
                        rospy.logerr('Invalid nodeManagement type {}'.format(type))
                        response = 'error'

                except KeyboardInterrupt:
                    break
                except:
                    rospy.logerr(sys.exc_info()[1])
                    response = 'error'
                finally:
                    responseQueue.put(response)

            else:
                rate.sleep()

    def handle_launch(self, req):
        rospy.loginfo('Got launch request: {} {}'.format(req.name, req.command))
        # handle the delay here so that we don't slow down the main thread
        if req.command.startswith('start') and len(req.command) > 5:
            delay = float(req.command[5:])
            time.sleep(delay)
            req.command = 'start'
        responseQueue = Queue()
        mainQueue.put(['launch', req.name, req.command, responseQueue])
        response = responseQueue.get()
        rospy.loginfo('Launch command response: [{}] to [{} {}]'.format(response, req.name, req.command))
        return(response)

    def handle_behavior(self, req):
        rospy.loginfo('Got behavior request: {} {}'.format(req.name, req.command))
        responseQueue = Queue()
        mainQueue.put(['behavior', req.name, req.command, responseQueue])
        response = responseQueue.get()
        rospy.loginfo('Behavior command response: [{}] to [{} {}]'.format(response, req.name, req.command))
        return(response)

    def process_behavior(self, name, command):
        try:
            responses = set()
            names = Behaviors[name]
            for name in names:
                response = self.process_launch(name, command)
                if response != 'started':
                    rospy.loginfo('response {} to {}'.format(response, name))
                responses.add(response)
            if 'error' in responses:
                return('error')
            else:
                return('started')
        except AttributeError:
            rospy.logerr('Behavior {} not found'.format(name))
            return('error')

    def process_launch(self, name, command):
        if command.startswith('start'):
            if name in self.launchers:
                response = 'active'
            else:
                try:
                    if len(command) > 5:
                        delay = float(command[5:])
                        time.sleep(delay)
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    self.launchers[name] = roslaunch.parent.ROSLaunchParent(uuid, ['/home/kent/github/rkent/bdbd/src/bdbd/launch/' + name + '.launch'])
                    self.launchers[name].start()
                    response = 'started'
                except:
                    rospy.logerr(sys.exc_info()[1])
                    response = 'error'
                    self.launchers.pop(name)

        elif command == 'shutdown' or command == 'stop':
            if name in self.launchers:
                self.launchers[name].shutdown()
                self.launchers.pop(name)
                response = 'shutdown'
            else:
                response = 'inactive'

        elif command == 'status':
            if name in self.launchers:
                response = 'active'
            else:
                response = 'inactive'

        else:
            response = 'invalid'

        return response

def main():
    NodeManagement()

if __name__ == '__main__':
    main()