#!/usr/bin/env python3

import rospy,os
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from gtts import gTTS
import pygame

audio_file = "/home/rsdlab/catkin_ws/src/speech_word/worp.mp3"

def text_to_speech_callback(msg):
    rospy.loginfo(f"Received text :{msg.data}")
    try:
        tts = gTTS(text=msg.data, lang="ja")
        tts.save(audio_file)
    except Exception as e:
        print(e)



def handle_start_request(req):
    rospy.loginfo("play_audio service called")

    if os.path.exists(audio_file):
        pygame.mixer.init()
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()
        
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        rospy.loginfo("Playback completed.")

        return TriggerResponse(success=True,message="OK")

    else:
        return TriggerResponse(success=False, message="ERROR")

# 音楽を停止するサービス
def handle_stop_request(req):
    rospy.loginfo("stop_audio service called")

    # 音楽が再生中であれば停止
    if pygame.mixer.music.get_busy():
        pygame.mixer.music.stop()
        rospy.loginfo("Playback stopped.")
        return TriggerResponse(success=True, message="OK")
    else:
        rospy.loginfo("No audio is playing.")
        return TriggerResponse(success=False, message="ERROR")

def trigger_service_server():
    rospy.init_node('play_audio_node')
    rospy.Subscriber('/text_to_speech',String,text_to_speech_callback)

    service = rospy.Service('/play_audio', Trigger, handle_start_request)
    stop_service = rospy.Service('/stop_audio', Trigger, handle_stop_request)

    rospy.loginfo("play_audio service server ready")
    rospy.spin()

if __name__ == "__main__":
    try:
        trigger_service_server()
    except rospy.ROSInterruptException:
        pass