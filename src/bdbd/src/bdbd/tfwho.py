#!/usr/bin/env python
''' Interface to python3-only tensorflow model from ros and python2 '''

import rospy
import time
from subprocess import Popen
from bdbd.libpy.MessageClient import MessageClient
import numpy as np
import pickle
from Queue import Queue
from std_msgs.msg import ByteMultiArray
import logging



def on_get_embedding(msg):
    pass

def main():
    queue = Queue()

    logging.basicConfig(level='INFO')
    log = logging.getLogger('tfwho')

    rospy.init_node('tfwho')
    data_dir = 'bddata/'
    #(svm_model, svm_encoder) = pickle.load(open(data_dir + 'family_face_svm.p', 'rb'))

    mqtt_queue = Queue()
    ros_queue = Queue()
    mc = MessageClient('localhost', 'bdbd-tfwho', queue)
    mc.connect()
    mc.subscribe('beedee/command')
    mc.subscribe('bddata/put_embedding', msgType='bytes')

    rospy.Subscriber('tfwho/get_embedding', ByteMultiArray, on_get_embedding)
    #data = np.load('bddata/faces.npz', allow_pickle=True)
    #X = data['arr_0']

    # Wait for do_tr to signal ready
    rate = rospy.Rate(100)
    p = Popen(['python3', 'do_tf.py'])
    while not rospy.is_shutdown():

        # process mqtt subscriptions
        while not mqtt_queue.empty():
            topic, message = queue.get()
            if topic == 'beedee/command':
                if message == 'GET_EMBEDDING.READY':
                    break

        # process ros subscriptions
        while not ros_queue.empty():
            pass

        rate.sleep()

    mc.publish('beedee/command', '_quit')
    log.info('wait for process to accept _quit and end')
    p.wait()
    mc.close()

if __name__ == '__main__':
    main()

'''
start = time.time()
id = 'dummy'
pickled_image = pickle.dumps((id, X[0]))
mc.publish('beedee/get_embedding', pickled_image, msgType='bytes')

while True:
    try:
        topic, message = queue.get()
        if topic == 'bddata/put_embedding':
            id, embedding = pickle.loads(message)
            log.info(f'got embedding with id {id}')
            log.info(f'type of embedding: {type(embedding)}')
            log.info(f'elapsed time: {round(time.time() - start, 3)}')
            name, prob = classify_face(embedding)
            log.info(f'name {name} ({round(prob, 1)}%)')
    except KeyboardInterrupt:
        mc.publish('beedee/command', '_quit')
        break
log.info('wait for process to accept _quit and end')
p.join()
mc.close()

#!/usr/bin/env python
import rospy
from bdbd.msg import GamepadEvent
from libgamepad import GamePad

def publishEvents():
    rospy.init_node('gamepad')
    pub = rospy.Publisher('~events', GamepadEvent, queue_size=10)
    pad = GamePad()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        result = pad.getEvent()
        if result:
            rospy.loginfo(str(result))
            pub.publish(result[0], result[1])
        else:
            rate.sleep()
    print('rospy shutdown')

def main():
    # dummy
    import os
    print(os.getcwd())
    try:
        publishEvents()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
    
def publishEvents():
    rospy.init_node('gamepad')
    pub = rospy.Publisher('~events', GamepadEvent, queue_size=10)
    pad = GamePad()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        result = pad.getEvent()
        if result:
            rospy.loginfo(str(result))
            pub.publish(result[0], result[1])
        else:
            rate.sleep()
    print('rospy shutdown')

def main():
    # dummy
    import os
    print(os.getcwd())
    try:
        publishEvents()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

'''