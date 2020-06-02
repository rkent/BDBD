# tuning of speech detection on respeaker

from bdbd.libpy.respeaker.tuning import find
from bdbd.libpy.rerecognizer import is_speech
import time

dev = find()

parms = ["DOAANGLE", "VOICEACTIVITY", "SPEECHDETECTED", "GAMMAVAD_SR"]
while True:
    outs = ""
    for parm in parms:
        outs += "{}: {:6.3f} ".format(parm, dev.read(parm))
    print(outs + " is_speech: {}".format(is_speech()))
    time.sleep(0.5)

