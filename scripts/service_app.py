#!/usr/bin/env python3
# -*- coding: utf-8 -*-

######################################​##################
import rospy
import math
import yaml
import os
from std_srvs.srv import Trigger, TriggerResponse
from speech_word.srv import *
import actionlib
from speech_word.msg import *




class ROIS_METHOD:
  def __init__(self):
    self.connect = rospy.ServiceProxy('/connect', system_interface)
    rospy.wait_for_service('/connect')
    self.disconnect = rospy.ServiceProxy('/disconnect', system_interface)
    rospy.wait_for_service('/bind_any')
    self.bind = rospy.ServiceProxy('/bind_any', bind_any)
    self.release = rospy.ServiceProxy('/release', release)
    self.subscribe = rospy.ServiceProxy('/subscribe', subscribe_)
    self.service_client = rospy.ServiceProxy('/execute', execute)

  def test_execute(self,component_ref):
    self.component_ref = component_ref
    print(f"test_execute {component_ref}")
    rospy.sleep(3)
    return 'succeeded'

  def connect(self,connect):
    print("connect_engine")
    
    result = self.connect(connect)
    
    return result.returncode_t 


  def disconnect(self, connect):
    print("connect_engine")
    system_if = rospy.ServiceProxy('/disconnect_engine', system_interface)
    result = system_if(connect)
    
    if result.returncode_t == "OK":
      return "succeeded"
    else:
      return "retry"

  def bind_any(self,component_ref):
    bind = self.bind(component_ref)
    return bind.Returncode_t

  def release(self, component_ref):
    release = self.release(component_ref)
    return release.Returncode_t

  def subscribe(self, event_type):
    subscribe = self.subscribe(event_type)
    return subscribe.Returncode_t

  def get_parameter(self, component_ref):
    if component_ref == "Move":
      get_name = '/get_parameter/' + component_ref  
      get_param = rospy.ServiceProxy(get_name, move_get_parameter)

    elif component_ref == "Speech_Synthesis":
      get_name = '/get_parameter/' + component_ref 
      get_param = rospy.ServiceProxy(get_name, speech_get_parameter)
      
      
      return get_param(component_ref)


  def set_parameter(self, component_ref, parameters):
    if component_ref == "Move":
      set_name = '/set_parameter/' + component_ref
      set_param = rospy.ServiceProxy(set_name, move_set_parameter)

      result = set_param(component_ref,parameters)


    elif component_ref == "Speech_Synthesis":
      set_name = '/set_parameter/' + component_ref
      set_param = rospy.ServiceProxy(set_name, speech_set_parameter)
      result = set_param(component_ref, parameters)
      
      print(result.Returncode_t)

      return 

  def execute(self, command_unit_list):
    """サービスで移動開始を要求する"""
    try:
      response = self.service_client(command_unit_list)
      if response.return_t == "OK":
        rospy.loginfo("コマンドを開始しました。アクションで進捗を確認します。")

        return response.return_t
      else:
        rospy.logwarn("コマンドの開始に失敗しました。")
    except rospy.ServiceException as e:
      rospy.logerr(f"サービス呼び出しに失敗しました: {e}")

  def monitor_progress(self):
    rospy.loginfo("待機中...")
    result = rospy.wait_for_message('/completed', completed)
    rospy.loginfo(f"Received message: {result.command_id}", )
    rospy.loginfo(f"Received message: {result.status}", )


  def monitor_event(self):
    rospy.loginfo("待機中...")
    result = rospy.wait_for_message('/notify_event', notifyevent)
    rospy.loginfo(f"Received message: {result.event_id}", )
    rospy.loginfo(f"Received message: {result.event_type}", )
    rospy.loginfo(f"Received message: {result.subscribe_id}", )

    return result.event_id


  def subscribe(self, event_type):
    subservice = rospy.ServiceProxy('/Subscribe', subscribe_)
    result = subservice(event_type)

    return result

  def unsubscribe(self, subscribe_id):
    unsubservice = rospy.ServiceProxy('/Unsubscribe', unsubscribe_)
    result = unsubservice(subscribe_id)

    return result



  def get_command_result(self, command_id):
    command_result = rospy.ServiceProxy('/get_command_result', get_command_result)
    result = command_result(command_id, "")
    return result


  def get_event_detail(self, event_id):
    print(event_id)
    if "recog" in  event_id:
      event_result = rospy.ServiceProxy('/recognized_event_detail', get_event_detail_speech_recognized)

    elif "detected" in event_id:
      event_result = rospy.ServiceProxy('/detected_event_detail', get_event_detail_person_detected)

    elif "local" in event_id:
      event_result = rospy.ServiceProxy('/local_event_detail', get_event_detail_person_localized)

    elif "ident" in event_id:
      event_result = rospy.ServiceProxy('/identified_event_detail', get_event_detail_person_identified)

    result = event_result(event_id)
    return result
  
    

def add_item(_list, item):
  if item not in _list:
    _list.append(item)
    print(f"{item} を追加しました")
    return True
  else:
    print(f"{item} は既にリストに存在します")
    print(_list)
    return False


def speech(speech_text):
    component_ref = "Speech_Synthesis"

    rois.bind_any(component_ref)

    rois.set_parameter(component_ref, speech_text)
    rois.execute("start")
    rospy.sleep(1)
    # result = rois.monitor_progress()
    rois.release(component_ref)

    return 

def speech_bind():
  component_ref = "Speech_Synthesis"
  rois.bind_any(component_ref)
  rois.release(component_ref)

def move_forward(line):
    component_ref = "Move"
    rois.bind_any(component_ref)

    # rois.set_parameter(component_ref, line)
    rois.execute("start")
    result = rois.monitor_progress()
    rois.release(component_ref)
    return result




def main():
    rois.connect("connect")

    speech("Speech_Synthesis" ,"こんにちは")

    speech("Speech_Synthesis" ,"voice")

  

def scenario(scenario):
    with open(scenario , 'r') as infile:
        data = yaml.safe_load(infile)


    scenario = data.get('scenario', [])

    task_list =[]

    rois.connect("connect")

    for task in scenario:
        task_name = task.get('task', '')
        task_arg = task.get('arg', '')
        task_list.append((task_name, task_arg))


        if task_name == "Speech_Synthesis":
          speech(task_arg)

          rospy.sleep(2)


        elif task_name == "Move":
          move_forward("Move", task_arg)   
          rospy.sleep(3)




    print(task_list)




if __name__ == '__main__':
  rospy.init_node('monitoring_smach_node')
  try:
    rois = ROIS_METHOD()

    user_input = input("選択してください: A.シナリオ,  B.デフォルト >>> ")
    print(user_input)
    if user_input == "A":
      # scenario_file = input("シナリオ名を入力してください: ")
      scenario_ = os.environ['HOME'] + "/RTSI_FW/scenario.yml" 
      scenario(scenario_)

    elif user_input == "B":
      main()

    else:
      print("finish")

  except rospy.ROSInterruptException:
    rospy.loginfo("クライアントが中断されました。")

