# demo of generation of wav file, manipulation with pydub, and playing through speakers
import espeakng
import pyaudio
import wave
import pydub
import sys
import io
import logging
logging.basicConfig(level='INFO')
log = logging.getLogger(__name__)
log.info('test of logging')

p = pyaudio.PyAudio()
#for i in range(p.get_device_count()):
#    print(p.get_device_info_by_index(i))

es = espeakng.ESpeakNG(voice='en-gb-x-gbclan')
wav = es.synth_wav('Hello')
wavFile = io.BytesIO(wav)

originalSegment = pydub.AudioSegment.from_file(wavFile)
changedSegment = originalSegment.apply_gain(-12.)

changedFile = io.BytesIO()
changedSegment.export(changedFile, 'wav')

#wavFile = 'e02-16kHz.wav'
chunk = 4096

changedFile.seek(0)
wf = wave.open(changedFile)

channels=wf.getnchannels()
rate=wf.getframerate()
print(format, channels, rate)
if True:
    stream = p.open(
        format=p.get_format_from_width(wf.getsampwidth()),
        channels=wf.getnchannels(),
        rate=wf.getframerate(),
        #output_device_index=11,
        output=True
    )

    data = wf.readframes(chunk)

    while len(data):
        print('writing data')

        stream.write(data)
        data = wf.readframes(chunk)

    stream.close()
p.terminate()

