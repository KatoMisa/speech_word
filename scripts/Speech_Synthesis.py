#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import actionlib
import rospy
from std_msgs.msg import String,Int32MultiArray
from std_srvs.srv import Trigger
from pygame import mixer      
import pygame
import rospy
import threading
from speech_word.msg import *
from speech_word.srv import *
from gtts import gTTS


audio_file = "/home/rsdlab/catkin_ws/src/speech_word/word.mp3"

#発話用(クラス追加)

class Speech_SynthesisService:
    def __init__(self):
        self.comp_state = "UNINITIALIZED"
        print(self.comp_state)

        self.comp_ref = "Speech_Synthesis"

        state_name = '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self.component_status)   #
        rospy.loginfo("component status server is ready")  

        self.text ="hello"
        self.word_num = 10
            

        exe_name = '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        print(exe_name)
        self.server.start()     

        self.set = rospy.Service('/speech_set_param', speech_set_param, self.set_parameter)  
        self.get = rospy.Service('/speech_get_param', speech_get_param, self.get_parameter)


        self.pub = rospy.Publisher('/completed_command', completed , queue_size=1)

        self.state = "idle"


        self.make_text = rospy.Publisher('/text_to_speech',String,queue_size=1)


        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする


        self.playback_thread = None 


    def set_parameter(self, s_req):
        print("set parameter")
        
        self.text = s_req.speech_text
        print(f"Synthesis text '{self.text}'")
            
        try:
            if len(self.text) > 0:

                tts = gTTS(text=self.text, lang="ja")
                tts.save(audio_file)
                set_return = "OK"
                return speech_set_paramResponse(set_return)

            else:
                set_return = "BAD_PARAMATER"
                return speech_set_paramResponse(set_return)

        except Exception as e:
                print(e)        
           
        

    def get_parameter(self,g_req):
        print("get_parameter")
        print(self.text)
        return speech_get_paramResponse(self.text)


    def execute(self, goal):
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする

        command = goal.command_name
        rospy.loginfo("Received command: %s", command)

        self.feedback = executeFeedback()
        self.result = executeResult()

        if command == "start":
            self.start()    
            
        elif command == "stop":
            self.stop()

        elif command == "suspend":
            self.suspend()

        elif command == "resume":
            self.resume()

        else:
            rospy.loginfo("No valid command received.")
            self.result.success = "False"
            self.server.set_aborted(self.result)
        


    def start(self):
        self.state = "playing"

        # 音楽再生を監視するスレッドを開始
        self.playback_thread = threading.Thread(target=self.monitor_playback)
        self.playback_thread.start()

        self.result.success = "True"
        self.server.set_succeeded(self.result)




    def monitor_playback(self):

        rospy.loginfo("Monitoring playback.")

        try:
            pygame.mixer.init()
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()


            while self.state == "playing":
                rospy.sleep(0.1)

                if not pygame.mixer.music.get_busy():
                    rospy.loginfo("Playback completed.")
                    self.state = "OK"
                    return

        except pygame.error as e:
            print(f"Pygame error occurred: {e}")
            self.state = "ERROR"

            self.comp_state = "ERROR"
            rospy.loginfo(f'Componemt status: {self.comp_state}') 
        except FileNotFoundError as e:
            print(f"File error: {e}")
            self.state = "ERROR"

            self.comp_state = "ERROR"
            rospy.loginfo(f'Componemt status: {self.comp_state}') 
        except Exception as e:
            print(f"Unexpected error: {e}")
            self.state = "ERROR"
            self.comp_state = "ERROR"
            rospy.loginfo(f'Componemt status: {self.comp_state}') 
        
        finally:
            self.completed_command(self.state)
        


    # 再生が終了したらこのメソッドが呼ばれる
    def completed_command(self, state):
        rospy.loginfo("Playback completed successfully.")
        pub_data = completed()
        pub_data.command_id = "Speech_Synthesis"
        pub_data.status = state
        print(f"{pub_data.command_id}:{pub_data.status}")
        self.pub.publish(pub_data)
        
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}') 
       



    def stop(self):
        if self.state == "playing":
            pygame.mixer.music.stop()
            self.state = "stopped"
            self.feedback.status = "playing stopped."
            
            pub_data = completed()
            pub_data.command_id = "speech_synthesis"
            pub_data.status = "stoped"
            self.pub.publish(pub_data)
            
            self.result.success = "True"
            self.server.set_succeeded(self.result)
            self.comp_state = "READY"
            rospy.loginfo(f'Componemt status: {self.comp_state}') 
        else:
            rospy.logwarn("No active playing.")
            self.feedback.status = "No active goal to stop."
            self.result.success = "False"
            self.server.set_aborted(self.result)


    def suspend(self):
        if self.state == "playing":
            pygame.mixer.music.pause()
            self.state = "suspended"
            self.feedback.status = "play suspended."
            self.result.success = "True"
            self.server.set_succeeded(self.result)
        else:
            rospy.logwarn("Cannot suspend; not playing.")
            self.result.success = "False"
            self.server.set_aborted(self.result)


    def resume(self):
        if self.state == "suspended":
            pygame.mixer.music.unpause()
            self.state = "playing"
            self.feedback.status = "Playing resumed."
            self.result.success = "True"
            self.server.set_succeeded(self.result)
              # 音楽再生を監視するスレッドを開始
            if not self.playback_thread or not self.playback_thread.is_alive():
                self.playback_thread = threading.Thread(target=self.monitor_playback)
                self.playback_thread.start()
        else:
            rospy.logwarn("No previous goal to resume.")
            self.result.success = "False"
            self.server.set_aborted(self.result)



    def component_status(self, req):
        # 現在の状態を返答
        if (req.component_name == "Speech_Synthesis"):
            rospy.loginfo("Current state requested: %s", self.comp_state)
            return component_statusResponse(self.comp_state)
        else:
            pass


    def run(self):
        rospy.loginfo("Service node is running...")
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node('Speech_Synthesis')
    print("time")
    service = Speech_SynthesisService()
    service.run()



