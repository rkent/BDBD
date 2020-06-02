# cleverbot chat
import requests

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
        r = requests.get(url=self._url, params=params, timeout=10.)
        result = {}
        result['status_code'] = r.status_code
        if r.status_code == 200:
            response = r.json()
            self._cs = response['cs']
            result['output'] = response['output']
        return result
                           
