#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import rospy
from dtroslib.helpers import get_package_path
from playsound import playsound
from std_msgs.msg import String
from tempfile import mkstemp

from speech import TTS


test_path = get_package_path('speech')


class TTSNode:
    __tts: TTS = None

    __record_fd: int = None
    __record_path: str = None

    def __init__(self,
                 client_id: str,
                 client_secret: str,
                 tts_speaker: str,
                 tts_speed: float = 0):

        self.__tts = TTS(client_id=client_id,
                         client_secret=client_secret,
                         tts_speaker=tts_speaker,
                         tts_speed=tts_speed)

        rospy.Subscriber('/action/speech', String, self.callback_speech)

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
        self.__record_fd, self.__record_path = mkstemp(suffix='.mp3')

    def unlink_record_file(self) -> bool:
        if not self.__record_fd:
            return False

        os.unlink(self.__record_path)

        self.__record_fd = None
        self.__record_path = None

        return True

    def callback_speech(self, msg: dict):
        text = msg.data

        rospy.loginfo(f'TTS request: {text}')

        speech = self.__tts.request(text)

        if speech:
            with os.fdopen(self.record_fd, 'wb') as f:
                f.write(speech)

            playsound(self.record_path)
            self.unlink_record_file()

        else:
            rospy.logerr('TTS failed.')


if __name__ == '__main__':
    rospy.init_node('tts_node')
    rospy.loginfo('Start TTS')

    client_id = os.environ['SPEECH_CLIENT_ID']
    client_secret = os.environ['SPEECH_CLIENT_SECRET']
    tts_speaker = os.environ['SPEECH_TTS_SPEAKER']
    tts_speed = float(os.environ['SPEECH_TTS_SPEED'])

    node = TTSNode(client_id=client_id,
                   client_secret=client_secret,
                   tts_speaker=tts_speaker,
                   tts_speed=tts_speed)

    rospy.spin()
