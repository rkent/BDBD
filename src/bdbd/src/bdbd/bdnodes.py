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
from bdbd_common.srv import NodeCommand
from bdbd_common.srv import DockerCommand
from Queue import Queue
import sys
import json
import os
import yaml

from rospy import topics

topics_launchers_yaml = """
    /bdbd/pantilt_camera/image_raw/compressed:
        - camera
"""
ROS_POLL_RATE = 100 # polls per second
FILE_POLL_TIME = 1.0 # seconds per poll

mainQueue = Queue()
class NodeManagement:
    def __init__(self):
        rospy.init_node("bdnodes")
        myname = rospy.get_name()

        # load topic/launch relationships
        self.topics_launchers = yaml.load(topics_launchers_yaml)
        print('topic_nodes:', self.topics_launchers)

        # load behaviors, and watch for changes
        self.BehaviorsModTime = None
        self.pollBehaviors()

        self.launchers = {} # the launcher objects used to start/stop nodes
        self.behaviors = set() # the requested behaviors
        self.launches = set() # the requested launches from launch request (that is, not by behavior)
        self.topics = set() # the requested topic
        start_behaviors_str = rospy.get_param('/bdbd/behaviors', '')
        rospy.loginfo('Initial behaviors: {}'.format(start_behaviors_str))

        # always start the remote docker service
        #mainQueue.put(['launch', 'dockers', 'start', myname, None])
        #self.launches.add('dockers')
        for behavior in start_behaviors_str.split():
            rospy.loginfo('Starting initial behavior {}'.format(behavior))
            mainQueue.put(['behavior', behavior, 'start', myname, None])

        self.launchService = rospy.Service('~launch', NodeCommand, self.handle_launch)
        self.behaviorService = rospy.Service('~behavior', NodeCommand, self.handle_behavior)
        self.topicService = rospy.Service('~topic', NodeCommand, self.handle_topic)
        rate = rospy.Rate(100)
        rospy.loginfo('Ready to process commands')
        while not rospy.is_shutdown():
            if not mainQueue.empty():
                self.pollBehaviors()
                req_type, name, command, callerid, responseQueue = mainQueue.get()
                response = None

                try:
                    if req_type == 'launch':
                        response = self.process_launch(name, command)
                
                    elif req_type == 'behavior':
                        response = self.process_behavior(name, command)

                    elif req_type == 'topic':
                        response = self.process_topic(name, command, callerid)

                    else:
                        rospy.logerr('Invalid nodeManagement req_type {}'.format(req_type))
                        response = 'error'

                except KeyboardInterrupt:
                    break
                except:
                    rospy.logerr(traceback.format_exc())
                    response = 'error'
                finally:
                    if responseQueue:
                        responseQueue.put(response)

            else:
                rate.sleep()

    def pollBehaviors(self):
        self.Behaviors_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'behaviors.json')
        modTime = os.stat(self.Behaviors_path).st_mtime
        if self.BehaviorsModTime and modTime == self.BehaviorsModTime:
            return
        rospy.loginfo('Loading behaviors file')
        with open(self.Behaviors_path) as behaviors_file:
            self.Behaviors = json.load(behaviors_file)
        self.BehaviorsModTime = modTime

    def handle_launch(self, req):
        rospy.loginfo('Got launch request: {} {}'.format(req.name, req.command))

        # handle the delay here so that we don't slow down the main thread
        if req.command.startswith('start') and len(req.command) > 5:
            delay = float(req.command[5:])
            time.sleep(delay)
            req.command = 'start'
        
        if req.command.startswith('start'):
            self.launches.add(req.name)
        elif req.command == 'stop' or req.command == 'shutdown':
            self.launches.discard(req.name)

        responseQueue = Queue()
        mainQueue.put(['launch', req.name, req.command, req._connection_header['callerid'], responseQueue])
        response = responseQueue.get()
        print('response type: {}'.format(type(response)))
        rospy.loginfo('Launch command response: [{}] to [{} {}]'.format(response, req.name, req.command))
        return(response)

    def handle_behavior(self, req):
        rospy.loginfo('Got behavior request: {} {}'.format(req.name, req.command))
        responseQueue = Queue()
        mainQueue.put(['behavior', req.name, req.command, req._connection_header['callerid'], responseQueue])
        response = responseQueue.get()
        rospy.loginfo('Behavior command response: [{}] to [{} {}]'.format(response, req.name, req.command))
        return(response)

    def handle_topic(self, req):
        # ensure that node required for a topic is started
        responseQueue = Queue()
        rospy.loginfo('Got topic request {} for {} from {}'.format(req.command, req.name, req._connection_header['callerid']))
        mainQueue.put(['topic', req.name, req.command, req._connection_header['callerid'], responseQueue])
        response = responseQueue.get()
        rospy.loginfo('Topic command response: [{}] to [{} {}]'.format(response, req.name, req.command))
        return(response)
        
    def process_behavior(self, behavior, command):
        try:
            if command == 'start':
                if behavior in self.behaviors:
                    return('active')
                self.behaviors.add(behavior)
                ret_response = 'started'
                launches = self.Behaviors[behavior]
                for launch in launches:
                    response = self.process_launch(launch, 'start')
                    if response == 'error':
                        rospy.logwarn('response {} to {}'.format(response, launch))
                        ret_response = 'error'
                return ret_response

            elif command == 'stop':
                if behavior not in self.behaviors:
                    return 'inactive'
                self.behaviors.remove(behavior)
                self.process_stops()
                return 'stopped'

            elif command == 'status':
                response = ''
                for behavior in self.behaviors:
                    response += behavior + ','
                return response

            else:
                rospy.logwarn('Invalid behavior command {}'.format(command))
                return 'error'

        except:
            rospy.logerr(traceback.format_exc())
            return('error')

    def process_docker(self, name, command):
        rospy.loginfo('process_docker: {} {}'.format(name, command))
        result = 'error'
        name = name.split(':')[0]
        try:
            rospy.wait_for_service('/bdbd_docker/containers', timeout=5)
            containers = rospy.ServiceProxy('/bdbd_docker/containers', DockerCommand)
            result = containers(name, command).response
        except:
            rospy.logerr('Error processing docker command: {}'.format(traceback.format_exc()))
        return result

    def process_launch(self, name, command):
        rospy.loginfo('process launch: {} {}'.format(name, command))
        if command.startswith('start'):
            if name in self.launchers:
                response = 'active'
            else:
                try:
                    if name.find(':docker') > 0:
                        # start using docker service
                        response = self.process_docker(name, command)
                        if response != 'error':
                            self.launchers[name] = 'docker'
                    else:    
                        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                        roslaunch.configure_logging(uuid)
                        self.launchers[name] = roslaunch.parent.ROSLaunchParent(uuid, ['/home/kent/github/rkent/bdbd/src/bdbd/launch/' + name + '.launch'])
                        self.launchers[name].start()
                        response = 'started'
                except:
                    rospy.logerr(traceback.format_exc())
                    response = 'error'
                    self.launchers.pop(name, None)

        elif command == 'shutdown' or command == 'stop':
            response = self.process_stops()

        elif command == 'status':
            if name in self.launchers:
                response = 'active'
            else:
                response = 'inactive'

        else:
            response = 'invalid'

        rospy.loginfo('launcher command {} for {} result {}'.format(command, name, response))
        return response

    def process_topic(self, topic, command, node):
        response = None
        rospy.loginfo('process topic {} {}'.format(topic, command))
        published_topics = rospy.get_published_topics()
        for published_topic in published_topics:
            if published_topic[0] == topic:
                rospy.loginfo('Found existing publisher for {}'.format(topic))
                response = 'active'
        if response:
            return response
            
        try:
            launchers = self.topics_launchers[topic]
        except KeyError:
            rospy.logwarn('Topic {} not implemented'.format(topic))
            return 'invalid'

        rospy.loginfo('found topic launcher configuration for topic {}: {}'.format(topic, launchers))
        response =  self.process_launch(launchers[0], command)

        return response

    def process_stops(self):
        ''' combines all active launches and behaviors, and stops and unneeded active launches '''
        needed_launches = set()
        for behavior in self.behaviors:
            print('behavior {}'.format(behavior))
            for launch in self.Behaviors[behavior]:
                print('launch {}'.format(launch))
                needed_launches.add(launch)
        for launch in self.launches:
            needed_launches.add(launch)

        # stop any unneeded launchers
        for launch in self.launchers.copy():
            if launch not in needed_launches:
                rospy.loginfo('stopping {}'.format(launch))
                if self.launchers[launch] == 'docker':
                    self.process_docker(launch, 'stop')
                else:
                    self.launchers[launch].shutdown()
                self.launchers.pop(launch)
            else:
                rospy.loginfo('continuing {}'.format(launch))
        return 'stopped'

def main():
    NodeManagement()

if __name__ == '__main__':
    main()