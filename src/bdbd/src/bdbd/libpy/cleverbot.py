# cleverbot chat
import requests
import traceback

class Cleverbot():
    def __init__(self):
        f = open('/home/kent/secrets/cleverbotkey.txt', 'r')
        self._key = f.read()
        self._url = 'https://www.cleverbot.com/getreply'
        self._cs = ''

    def getReply(self, text):
        params = {'key': self._key, 'input':text}
        if self._cs:
            params['cs'] = self._cs
        try:
            while True:
                r = requests.get(url=self._url, params=params, timeout=10.)
                result = {}
                result['status_code'] = r.status_code
                if r.status_code == 200:
                    response = r.json()
                    # problematic responses
                    text = response['output']
                    self._cs = response['cs']
                    result['output'] = text
                    break
        except:
            result = {'status_code': 0, 'output': ''}
            print(traceback.format_exc())
        return result
                           
