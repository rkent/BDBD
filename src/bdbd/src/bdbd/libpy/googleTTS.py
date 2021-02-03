# play using google TTS using my API key
import os
from io import BytesIO

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/kent/github/rkent/beedee/secrets/stalwart-bliss-270019-7159f52eb443.json'
from google.cloud import texttospeech

from pydub import AudioSegment
#import pydub.playback

from tempfile import NamedTemporaryFile
import subprocess
# cloned from pydub.playback._play_with_ffplay, adding -loglevel quiet
def rkj_play_with_ffplay(seg):
    PLAYER = '/usr/bin/ffplay'
    with NamedTemporaryFile("w+b", suffix=".wav") as f:
        seg.export(f.name, "wav")
        subprocess.call([PLAYER, "-nodisp", "-autoexit", "-hide_banner", "-loglevel", "quiet", f.name])

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
            audio_encoding=texttospeech.enums.AudioEncoding.MP3,
            volume_gain_db=-1.0,
            pitch=-2.0,
            speaking_rate=1.0)

    def say(self, text):
        # Set the text input to be synthesized
        synthesis_input = texttospeech.types.SynthesisInput(text=text)

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        response = self._client.synthesize_speech(synthesis_input, self._voice, self._audio_config)

        mp3_fd = BytesIO(response.audio_content)

        # convert an mp3 file to wav and play
        song = AudioSegment.from_mp3(mp3_fd)
        song = song.high_pass_filter(500)
        #pydub.playback._play_with_ffplay(song)
        rkj_play_with_ffplay(song)

if __name__ == '__main__':
    # demo
    googleTTS = GoogleTTS()
    googleTTS.say('Hello by name is BDBD robot')
