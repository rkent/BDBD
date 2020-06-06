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
import traceback
from bdbd.srv import NodeCommand, NodeCommandResponse
from Queue import Queue
import sys

Behaviors = {
    'explore': [
        'drivers',
        't265min',
        'sayit',
        'hearit',
        'speechResponse',
        'gamepad',
        'transforms',
        'sr305min',
        'detectBlocking',
        'explore',
    ],
    'chat': [
        'drivers',
        'sayit',
        'hearit',
        'speechResponse',
        'chat',
    ],
    'voice': [
        'drivers',
        'sayit',
        'hearit',
        'speechResponse',
    ],
}

mainQueue = Queue()
class NodeManagement:
    def __init__(self):
        rospy.init_node("bdnodes")
        self.launchers = {} # the launcher objects used to start/stop nodes
        self.behaviors = set() # the requested behaviors
        self.launches = set() # the requested launches
        start_behaviors_str = rospy.get_param('/bdbd/behaviors', '')

        for behavior in start_behaviors_str.split():
            mainQueue.put(['behavior', behavior, 'start', None])

        self.launchService = rospy.Service('~launch', NodeCommand, self.handle_launch)
        self.behaviorService = rospy.Service('~behavior', NodeCommand, self.handle_behavior)
        rate = rospy.Rate(100)
        rospy.loginfo('Ready to process commands')
        while not rospy.is_shutdown():
            if not mainQueue.empty():
                req_type, name, command, responseQueue = mainQueue.get()
                response = None

                try:
                    if req_type == 'launch':
                        response = self.process_launch(name, command)
                
                    elif req_type == 'behavior':
                        response = self.process_behavior(name, command)

                    else:
                        rospy.logerr('Invalid nodeManagement req_type {}'.format(req_type))
                        response = 'error'

                except KeyboardInterrupt:
                    break
                except:
                    rospy.logerr(sys.exc_info()[1])
                    response = 'error'
                finally:
                    if responseQueue:
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
        
        if req.command.startswith('start'):
            self.launches.add(name)
        elif req.command == 'stop' or req.command == 'shutdown':
            self.launces.discard(name)

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

    def process_behavior(self, behavior, command):
        try:
            if command == 'start':
                if behavior in self.behaviors:
                    return('active')
                self.behaviors.add(behavior)
                ret_response = 'started'
                launches = Behaviors[behavior]
                for launch in launches:
                    response = self.process_launch(launch, 'start')
                    if response == 'error':
                        rospy.logwarn('response {} to {}'.format(response, name))
                        ret_response = 'error'
                return ret_response

            elif command == 'stop':
                if behavior not in self.behaviors:
                    return 'inactive'
                self.behaviors.remove(behavior)
                self.process_stops()
                return 'stopped'
            else:
                rospy.logwarn('Invalid behavior command {}'.format(command))
                return 'error'

        except:
            rospy.logerr(traceback.format_exc())
            return('error')

    def process_launch(self, name, command):
        if command.startswith('start'):
            if name in self.launchers:
                response = 'active'
            else:
                try:
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
            self.process_stops()

        elif command == 'status':
            if name in self.launchers:
                response = 'active'
            else:
                response = 'inactive'

        else:
            response = 'invalid'

        rospy.loginfo('launcher command {} for {} result {}'.format(command, name, response))
        return response

    def process_stops(self):
        ''' combines all active launches and behaviors, and stops and unneeded active launches '''
        needed_launches = set()
        for behavior in self.behaviors:
            print('behavior {}'.format(behavior))
            for launch in Behaviors[behavior]:
                print('launch {}'.format(launch))
                needed_launches.add(launch)
        for launch in self.launches:
            needed_launches.add(launch)

        # stop any unneeded launchers
        for launch in self.launchers.copy():
            print('checking launcher {}'.format(launch))
            if launch not in needed_launches:
                rospy.loginfo('stopping {}'.format(launch))
                self.launchers[launch].shutdown()
                self.launchers.pop(launch)

def main():
    NodeManagement()

if __name__ == '__main__':
    main()