# ros node to listen for hearit audio, and play on speakers
import pyaudio
import wave
import pydub
import sys
import io
import os
import rospy
from audio_common_msgs.msg import AudioData

def main():

    def audio_cb(msg):
        wavFile = io.BytesIO(msg.data)
        rospy.loginfo('wav file data received length {}'.format(len(msg.data)))

        originalSegment = pydub.AudioSegment.from_file(wavFile)
        changedSegment = originalSegment.apply_gain(-15.)

        changedFile = io.BytesIO()
        changedSegment.export(changedFile, 'wav')

        #wavFile = 'e02-16kHz.wav'
        chunk = 4096

        changedFile.seek(0)
        wf = wave.open(changedFile)

        stream = p.open(
            format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            #output_device_index=11,
            output=True
        )

        data = wf.readframes(chunk)

        while len(data):
            stream.write(data)
            data = wf.readframes(chunk)

        stream.close()

    rospy.init_node('playaudio')
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    audiosub = rospy.Subscriber('/bdbd/audio', AudioData, audio_cb)
    p = pyaudio.PyAudio()

    #for i in range(p.get_device_count()):
    #    print(p.get_device_info_by_index(i))

    rospy.spin()

    p.terminate()

if __name__ == '__main__':
    main()
