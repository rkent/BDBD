# module defines the API for message handling, with an implementation in a specific messageing protocol

# currently configured for mqtt
#!pip install paho-mqtt

import paho.mqtt.client as mqtt
import logging
import traceback
import os
import time
import json
log = logging.getLogger(__name__)

def packId(id, data):
    # packs a string id before a data buffer
    return id.encode(encoding='ascii').ljust(50) + data

def unpackId(combined):
    id_bytes = combined[0:50].decode('ascii')
    data = combined[50:]
    return id_bytes, data

class MessageClient():
    def __init__(self, server, name, queue=None):
        self._server = server
        self._client = mqtt.Client(name)
        self._client.enable_logger()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._connected = False
        self._queue = queue
        self._topicType = {}

    @property
    def client(self):
        return self._client

    def connect(self):
        log.info('mqtt connect from {}'.format(os.getpid()))
        #log.debug(traceback.format_stack(limit=4))
        self._client.connect(self._server)
        self._client.loop_start()

    def subscribe(self, topic, msgType='str'):
        if topic in self._topicType:
            assert self._topicType[topic] == msgType, 'want type ' + msgType + ' but topic already ' + self._topicType[topic]
        assert msgType in ('bytes', 'str', 'json'), 'type not supported'
        assert self._queue, 'client needs a queue for subscriptions'
        self._topicType[topic] = msgType
        log.debug('Subscribing to topic ' + topic)

        if (self._connected):
            self._client.subscribe(topic)

    def unsubscribe(self, topic):
        log.debug('Unsubscribing from topic ' + topic)
        self._client.unsubscribe(topic)

    def publish(self, topic, message, msgType='str'):
        bytes_time = '{:14.4f} '.format(time.time()).encode('utf-8')[0:16]
        if topic in self._topicType:
            assert self._topicType[topic] == msgType, 'conflicting type for topic'
        else:
            self._topicType[topic] = msgType
        if msgType == 'str':
            bytes_message = message.encode('utf-8')
        elif msgType == 'bytes':
            bytes_message = message
        elif msgType == 'json':
            bytes_message = json.dumps(message).encode('utf-8')

        self._client.publish(topic, bytes_time + bytes_message)

    def _on_connect(self, client, userdata, flags, rc):
        try:
            log.info('mqtt connecting to %s from %s', self._server, os.getpid())
            for topic in self._topicType:
                client.subscribe(topic)
            self._connected = True
        except:
            exc = traceback.format_exc()
            log.error('Exception during on_connect:\n%s', exc)

    def _on_message(self, client, userdata, msg):
        try:
            # payload extended by 16 bytes with an ascii time for logging, throw away for local use
            real_payload = msg.payload[16:]
            if self._queue:
                if self._topicType[msg.topic] == 'bytes':
                    self._queue.put((msg.topic, real_payload))
                elif self._topicType[msg.topic] == 'json':
                    self._queue.put((msg.topic, json.loads(real_payload.decode('utf-8'))))
                else: # str
                    self._queue.put((msg.topic, real_payload.decode('utf-8')))
            else:
                log.error('on_message with no queue, topic %s PID %s', msg.topic, str(os.getpid()))
        except:
            exc = traceback.format_exc()
            print('msg during error: ', msg.topic, msg.payload)
            log.error('Exception during on_message:\n%s', exc)
         
    def close(self):
        self._client.disconnect()
        self._client.loop_stop()

# demo of usage
if __name__ == '__main__':
    def main():
        try:
            from queue import Queue
        except:
            from Queue import Queue
        logging.basicConfig(level = 'DEBUG')
        log.info('This is a demo of mqtt messaging')
        server = 'localhost'
        topic1 = 'topic1'
        topic2 = 'topic2'

        queue = Queue()
        messageClient = MessageClient(server, 'thename', queue)
        messageClient.subscribe(topic2)
        messageClient.connect()
        time.sleep(1)

        messageClient.subscribe(topic1, 'bytes')
        log.info('Publishing messages')
        messageClient.publish(topic1, b'content1', 'bytes')
        messageClient.publish(topic2, 'content2')
        log.info('waiting for queue')
        result = queue.get()
        log.info('got result ' + str(result))
        result = queue.get()
        log.info('got result ' + str(result))
        messageClient.close()
        log.info('all done')

    main()
