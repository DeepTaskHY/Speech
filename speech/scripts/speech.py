import json
import requests
from typing import BinaryIO
from urllib.parse import urlencode


class Speech:
    __client_id: str = None
    __client_secret: str = None

    def __init__(self,
                 client_id: str,
                 client_secret: str):

        self.__client_id = client_id
        self.__client_secret = client_secret

    @property
    def client_id(self) -> str:
        return self.__client_id

    @property
    def client_secret(self) -> str:
        return self.__client_secret


class STT(Speech):
    STT_API_URL: str = 'https://naveropenapi.apigw.ntruss.com/recog/v1/stt?lang=Kor'

    def get_headers(self) -> dict:
        headers = {
            'Content-Type': 'application/octet-stream',
            'X-NCP-APIGW-API-KEY-ID': self.client_id,
            'X-NCP-APIGW-API-KEY': self.client_secret,
        }

        return headers

    def request(self, data: BinaryIO) -> str:
        response = requests.post(self.STT_API_URL,
                                 headers=self.get_headers(),
                                 data=data)

        status_code = response.status_code

        if status_code != 200:
            return None

        parsed_response = json.loads(response.text)
        text = parsed_response['text']

        return text


class TTS(Speech):
    TTS_API_URL: str = 'https://naveropenapi.apigw.ntruss.com/voice/v1/tts'

    __tts_speaker: str = None
    __tts_speed: float = 0

    def __init__(self,
                 tts_speaker: str,
                 tts_speed: float = 0,
                 *args, **kwargs):

        super(TTS, self).__init__(*args, **kwargs)

        self.__tts_speaker = tts_speaker
        self.__tts_speed = tts_speed

    @property
    def tts_speaker(self) -> str:
        return self.__tts_speaker

    @property
    def tts_speed(self) -> float:
        return self.__tts_speaker

    def get_headers(self) -> dict:
        headers = {
            'Content-Type': 'application/x-www-form-urlencoded',
            'X-NCP-APIGW-API-KEY-ID': self.client_id,
            'X-NCP-APIGW-API-KEY': self.client_secret,
        }

        return headers

    def get_body(self, text: str) -> dict:
        body = {
            'speaker': self.tts_speaker,
            'speed': self.tts_speed,
            'text': text
        }

        return urlencode(body)

    def request(self, text: str):
        response = requests.post(self.TTS_API_URL,
                                 headers=self.get_headers(),
                                 data=self.get_body(text))

        status_code = response.status_code

        if status_code != 200:
            return None

        speech = response.content

        return speech
