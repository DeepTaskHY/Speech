#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import os
import queue
import requests
import rospy
import sounddevice as sd
import soundfile as sf
import threading
import time
from dtroslib.helpers import get_package_path
from queue import Queue
from std_msgs.msg import Bool
from std_msgs.msg import String
from tempfile import mkstemp
from threading import Thread
from typing import Tuple

from speech import STT

test_path = get_package_path('speech')

_count = 0


def generate_message(text: str):
    global _count

    generated_message = {
        'header': {
            'id': _count + 1,
            'timestamp': str(time.time()),
            'source': 'stt',
            'target': ['planning'],
            'content': 'human_speech'
        },
        'human_speech': {
            'stt': text
        }
    }

    return json.dumps(generated_message)


def stt(audio_file):

    data = open(audio_file, 'rb')  # STT를 진행하고자 하는 음성 파일
    url = 'https://naveropenapi.apigw.ntruss.com/recog/v1/stt?lang=Kor'

    client_id = 'jqhycv20tf'  # 인증 정보의 Client ID
    client_secret = 'EFh6RhwU8DDuLX3O1qaSCqc2jRgu2P6asUl6wmiR'  # 인증 정보의 Client Secret

    headers = {
        'Content-Type': 'application/octet-stream',  # Fix
        'X-NCP-APIGW-API-KEY-ID': client_id,
        'X-NCP-APIGW-API-KEY': client_secret,
    }

    response = requests.post(url, data=data, headers=headers)
    rescode = response.status_code

    if rescode == 200:
        text_dict = json.loads(response.text)
        text_data = text_dict['text']
        rospy.loginfo('STT : {}'.format(text_data))
    else:
        rospy.loginfo('Error : {}'.format(response.text))
        return ''

    return text_data


class Recorder:
    __stt: STT = None
    __speech_publisher: rospy.Publisher = rospy.Publisher('/recognition/speech', String, queue_size=10)
    __speech_queue: Queue = Queue()
    __recorder_thread: Thread = None
    __recorded_file: file = None

    __switch: bool = False
    __recording: bool = False
    __mic_index: int = None

    def __init__(self,
                 client_id: str,
                 client_secret: str,
                 mic_index: int):

        self.__stt = STT(client_id=client_id,
                         client_secret=client_secret)

        self.__mic_index = mic_index

        rospy.Subscriber('/action/recorder_on', Bool, self.callback_switch_toggle)

    @property
    def switch(self) -> bool:
        return self.__switch

    @switch.setter
    def switch(self, value: bool):
        self.__switch = value

    @property
    def recording(self) -> bool:
        return self.__recording

    @recording.setter
    def recording(self, value: bool):
        self.__recording = value

    @property
    def recorded_file(self) -> file:
        if not self.__recorded_file:
            self.__recorded_file, recorded_path = mkstemp(suffix='.wav')

        return self.__recorded_file

    def unlink_recorded_file(self) -> bool:
        if not self.__recorded_file:
            return False

        os.close(self.__recorded_file)
        self.__recorded_file = None

        return True

    @property
    def mic_index(self) -> int:
        return self.__mic_index

    def switch_toggle(self, flag: bool) -> bool:
        self.switch = flag

        if self.switch:
            if self.recording:
                return False

            self.start_record()

        else:
            if not self.recording:
                return False

            self.stop_record()

        return True

    def callback_switch_toggle(self, msg: dict):
        flag = msg.data
        result = self.switch_toggle(flag)

        if flag:
            if result:
                rospy.loginfo('Started recording.')
            else:
                rospy.warninfo('Already recording.')
        else:
            if result:
                rospy.loginfo('Recording is finished.')
                text = self.__stt.request(self.recorded_file)
                self.__speech_publisher.publish(generate_message(text))
            else:
                rospy.warninfo('Not recording.')

    def start_record(self):
        self.stop_record()
        self.__recorder_thread = Thread(target=self.callback_record)
        self.__recorder_thread.start()
        self.recording = True

    def stop_record(self):
        if self.__recorder_thread:
            self.__recorder_thread.join()
            self.__recorder_thread = None
            self.recording = False

    def callback_record(self):
        with sd.InputStream(callback=self.callback_recording,
                            device=self.mic_index,
                            dtype='int16',
                            samplerate=44100,
                            channels=1):

            while self.recording:
                self.recorded_file.write(self.__speech_queue.get())

    def callback_recording(self, indata):
        self.__speech_queue.put(indata.copy())


if __name__ == '__main__':
    rospy.init_node('stt_node')
    rospy.loginfo('Start STT')

    client_id = os.environ['SPEECH_CLIENT_ID']
    client_secret = os.environ['SPEECH_CLIENT_SECRET']
    mic_index = int(os.environ['MIC_INDEX'])

    recorder = Recorder(client_id=client_id,
                        client_secret=client_secret,
                        mic_index=mic_index)

    rospy.spin()
