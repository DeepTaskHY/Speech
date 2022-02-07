#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import os
import rospy
import sounddevice as sd
import soundfile as sf
import time
from dtroslib.helpers import get_package_path
from queue import Queue
from std_msgs.msg import Bool, String
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


class STTNode:
    __stt: STT = None
    __speech_publisher: rospy.Publisher = rospy.Publisher('/recognition/speech', String, queue_size=10)
    __speech_queue: Queue = Queue()
    __record_thread: Thread = None
    __record_fd: int = None
    __record_path: str = None

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
        sd.default.device = self.__mic_index

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
    def record_fd(self) -> int:
        if not self.__record_fd:
            self.generate_record_file()

        return self.__record_fd

    @property
    def record_path(self) -> str:
        if not self.__record_path:
            self.generate_record_file()

        return self.__record_path

    def generate_record_file(self):
        self.unlink_record_file()
        self.__record_fd, self.__record_path = mkstemp(suffix='.wav')

    def unlink_record_file(self) -> bool:
        if not self.__record_fd:
            return False

        os.unlink(self.__record_path)

        self.__record_fd = None
        self.__record_path = None

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
                rospy.logwarn('Already recording.')
        else:
            if result:
                rospy.loginfo('Recording is finished.')

                with os.fdopen(self.record_fd, 'rb') as f:
                    text = self.__stt.request(f)

                self.unlink_record_file()
                self.__speech_publisher.publish(generate_message(text))
            else:
                rospy.logwarn('Not recording.')

    def start_record(self):
        self.stop_record()
        self.recording = True
        self.__record_thread = Thread(target=self.callback_record)
        self.__record_thread.start()

    def stop_record(self):
        if self.__record_thread:
            self.recording = False
            self.__record_thread.join()
            self.__record_thread = None

    def callback_record(self):
        with sf.SoundFile(self.record_path,
                          mode='w',
                          subtype='PCM_16',
                          samplerate=44100,
                          channels=1) as f:

            with sd.InputStream(callback=self.callback_recording,
                                device=self.mic_index,
                                dtype='int16',
                                samplerate=44100,
                                channels=1):

                while self.recording:
                    f.write(self.__speech_queue.get())

    def callback_recording(self, data, frames, time, status):
        self.__speech_queue.put(data.copy())


if __name__ == '__main__':
    rospy.init_node('stt_node')
    rospy.loginfo('Start STT')

    client_id = os.environ['SPEECH_CLIENT_ID']
    client_secret = os.environ['SPEECH_CLIENT_SECRET']
    mic_index = int(os.environ['MIC_INDEX'])
    
    node = STTNode(client_id=client_id,
                   client_secret=client_secret,
                   mic_index=mic_index)

    rospy.spin()
