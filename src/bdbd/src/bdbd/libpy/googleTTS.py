# play using google TTS using my API key
import os
import rospy

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/kent/github/rkent/beedee/secrets/stalwart-bliss-270019-7159f52eb443.json'
from google.cloud import texttospeech

from tempfile import NamedTemporaryFile
import subprocess

PLAYER = '/usr/bin/ffplay'
effect = 'wearable-class-device'

class GoogleTTS():
    def __init__(self, language='en-AU', voiceName='en-AU-Standard-B', voiceGender='MALE'):
        # Instantiates a client
        self._client = texttospeech.TextToSpeechClient()
        self._voice = texttospeech.types.VoiceSelectionParams(
                name=voiceName,
                language_code=language,
                ssml_gender=texttospeech.enums.SsmlVoiceGender[voiceGender]
            )
        self._audio_config =  texttospeech.types.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16,
            volume_gain_db=-2.0,
            pitch=0.0,
            speaking_rate=1.0,
            effects_profile_id=[effect])

    def say(self, text):
        # Set the text input to be synthesized
        rospy.loginfo('Asked Google to say {}'.format(text))
        synthesis_input = texttospeech.types.SynthesisInput(text=text)

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        response = self._client.synthesize_speech(synthesis_input, self._voice, self._audio_config)
        rospy.loginfo('Got audio content from Google')

        with NamedTemporaryFile("w+b", suffix=".wav") as f:
            f.write(response.audio_content)
            subprocess.call([PLAYER, "-nodisp", "-autoexit", "-hide_banner", "-loglevel", "quiet", f.name])
        rospy.loginfo('Done playing')

if __name__ == '__main__':
    # demo
    rospy.init_node('test')
    googleTTS = GoogleTTS()
    googleTTS.say('Hello my name is BDBD robot')
