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
from std_msgs.msg import Bool
from std_msgs.msg import String


test_path = get_package_path('speech')

_count = 0

def to_ros_msg(data):
    global _count
    json_msg = {
        'header': {
            'source': 'stt',
            'target': ['planning'],
            'content': 'human_speech',
            'id': _count+1
        },
        'human_speech': {
            'stt': data,
            'timestamp': str(time.time())
        }
    }
    ros_msg = json.dumps(json_msg, ensure_ascii=False, indent=4)

    return ros_msg


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
    def __init__(self):
        self.switch_on = False
        self.recording = False
        self.q = queue.Queue()
        self.recorder = None
        self.save_name = test_path + '/data/human_speech.wav'
        self.now_pressed = None

        rospy.Subscriber('/action/recorder_on', Bool, self.switch_toggle)
        self.publisher = rospy.Publisher('/recognition/speech', String, queue_size=10)

    def record_voice(self):
        mic_index = int(os.environ['MIC_INDEX'])

        with sf.SoundFile(self.save_name, mode='w', subtype='PCM_16', samplerate=44100, channels=1) as f:
            with sd.InputStream(callback=self.save_voice, dtype='int16', samplerate=44100, channels=1, device=mic_index):
                while self.recording:
                    f.write(self.q.get())

    def save_voice(self, indata, frames, time, status):
        self.q.put(indata.copy())

    def switch_toggle(self, msg):
        self.switch_on = msg.data

        if self.switch_on is True:
            if self.recording is True:
                rospy.loginfo('Recording has started already.')
                return
            self.recording = True
            self.recorder = threading.Thread(target=self.record_voice)
            rospy.loginfo('Start recording.')
            self.recorder.start()

        if self.switch_on is False:
            if self.recording is False:
                rospy.loginfo('Recording has stopped already.')
                return
            self.recording = False
            self.recorder.join()
            rospy.loginfo('Stop recording.')
            text = stt(self.save_name)
            self.publisher.publish(to_ros_msg(text))


if __name__ == '__main__':
    rospy.init_node('stt_node')
    rospy.loginfo('Start STT')

    rec = Recorder()

    rospy.spin()
