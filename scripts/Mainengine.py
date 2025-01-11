#!/usr/bin/env python3
import rospy
import tf
import subprocess
from speech_word.srv import *
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from speech_word.msg import *
from std_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


### 2024/10/24作成　Move足したい
### system_interface, bind, release, get, set, complete, get_command_result, subscribe, unsubscribe, notify_event, get_event_detail (24.10.15更新)
### Navigation, Speech_Synthesis, Speech_Recognition  Move


class EngineService:
    def __init__(self):
        
        #EngineとApp.が通信できるかどうかを決める変数
        self.ENGINE_RECEIVABLE = False
        
        # サービスサーバーを作成
        self.con = rospy.Service('/connect', system_interface, self.connect)
        self.discon = rospy.Service('/disconnect', system_interface, self.disconnect)
        self.bind = rospy.Service('/bind_any', bind_any, self.bind_any)
        self.release = rospy.Service('/release', release, self.release)
        self.service = rospy.Service('/execute', execute, self.execute_cb)
        self.subservice = rospy.Service('/subscribe', subscribe_, self.subscribe_cb)
        self.unsubservice = rospy.Service('/unsubscribe', unsubscribe_, self.unsubscribe_cb)


        self.completed_sub = rospy.Subscriber('/completed_command', completed, self.completed)
        self.completed_pub = rospy.Publisher('/completed', completed, queue_size = 1)
        self.completed_pub1 = rospy.Publisher('/completed/speech', completed, queue_size = 1)
        self.completed_pub2 = rospy.Publisher('/completed/move', completed, queue_size = 1)
        
        self.get_command_result = rospy.Service('/get_command_result', get_command_result, self.get_command_result_cb)
        
        self.command_list = []
        self.event_list = []


        self.last_timestamp = ""
        
        self.event_count = 0
        self.command_count = 0
        self.subcount = 0

        #　コンポーネントをbindしているかの変数
        self.BINDCOMP = None


        # 話す言葉を格納する変数と初期値
        self.text = ""

        rospy.loginfo("Engine Service is ready.")

        self.bindspeech = 0
        self.bindrecognition = 0


    #System_Interfaceのconnect()
    #ENGINE_RECEIVABLEがTrueになるとメッセージのやり取りが可能になる
    def connect(self, si_req):
        print("system_interface")
        if si_req.connect == "connect":

            self.ENGINE_RECEIVABLE = True  
            # rospy.loginfo(self.ENGINE_RECEIVABLE)
            rospy.loginfo("Engine is receivable")
            #Returncode_t
            return system_interfaceResponse("OK")           
        else:
            #Returncode_t
            return system_interfaceResponse("ERROR")


    #System_Interfaceの, disconnect()
    #ENGINE_RECEIVABLEがFalseになるとメッセージのやり取りが不可能になる
    def disconnect(self, si_req):
        print("system_interface")
        if si_req.connect == "disconnect":
            self.ENGINE_RECEIVABLE = False
            # rospy.loginfo(self.ENGINE_RECEIVABLE)
            rospy.loginfo("Engine is not receivable")
            #Returncode_t
            return system_interfaceResponse("OK")
      
        else:
            #Returncode_t
            return system_interfaceResponse("ERROR")

    def is_service_available(self,service_name):
        """ROSサービスが存在するか確認"""
        service_list = subprocess.getoutput("rosservice list").split('\n')
        return service_name in service_list

    def is_action_server_active(self,action_name):
        """アクションサーバーが存在しているか確認"""
        try:
            state_topic = f"{action_name}/status"
            rospy.wait_for_message(state_topic, GoalStatusArray, timeout=1.0)
            rospy.loginfo(f"Action server '{action_name}' is active.")
            return True
        except rospy.ROSException:
            rospy.logwarn(f"Action server '{action_name}' is not available.")
            return False    

    #Command_Interfaceのbind_any()
    #component_list_unit内のHRI-Cと接続する関数
    def bind_any(self,b_req):
        
        #エンジンにつながっているHRI-Cの一覧
        self.component_ref_list = ["Speech_Synthesis"]

        #ENGINE_RECEIVABLEがTrueの場合
        if self.ENGINE_RECEIVABLE:
            #　self.component_ref…扱うコンポーネントの名前
            self.component_ref = b_req.component_ref
            
            if self.BINDCOMP == None:
            #重複防止のためにサービス名を作成
                get_name = '/get_parameter/' + self.component_ref  
                set_name = '/set_parameter/' + self.component_ref
                exe_name = '/execute/' + self.component_ref

                try:
                    #リクエストされたコンポーネント名がcomponent_ref_listにある場合に実行
                    if self.component_ref in self.component_ref_list:
                        #Speech_Synthesisをbindする場合は
                        if self.component_ref == "Speech_Synthesis":
                            print(f"bind Speech_Synthesis time is {self.bindspeech}")
                            if self.bindspeech > 0: 
                                pass
                            else:
                                self.text = "hello"
                                if self.is_service_available(get_name):
                                    rospy.loginfo(f"Service '{get_name}' already exists. Skipping...")
                                else:
                                    self.get_parameter = rospy.Service(get_name, speech_get_parameter, self.get_speech)
                                    rospy.loginfo(f"{get_name} Service is ready.")

                                if self.is_service_available(set_name):
                                    rospy.loginfo(f"Service '{set_name}' already exists. Skipping...")
                                else:
                                    self.set_parameter = rospy.Service(set_name, speech_set_parameter, self.set_speech) 
                                    rospy.loginfo(f"{set_name} Service is ready.")

                                
                                self.set_text = rospy.ServiceProxy('/speech_set_param', speech_set_param)
                                rospy.loginfo(f'action client "speech_set_param" start...')

                            self.client = actionlib.SimpleActionClient(exe_name, executeAction)
                            self.client.wait_for_server()
                            rospy.loginfo(f'action client {exe_name} start...')

                            self.BINDCOMP = self.component_ref 
                            rospy.loginfo(f"Bind {self.component_ref}")
                            self.bindspeech += 1

                            #Returncode_t
                            return bind_anyResponse("OK")


                    #　component_ref_listにない場合
                    else:
                        rospy.loginfo("HRI-Component doesn't exist.")
                        #Returncode_t
                        return bind_anyResponse("ERROR")

                except Exception as e:
                    rospy.logerr(f"An error occurred: {e}")
                    return bind_anyResponse("ERROR")
            else:
                print("Bind_any is failed")
                #Returncode_t
                return bind_anyResponse("ERROR")   

        #ENGINE_RECEIVABLEがFalseの場合
        else:
            print("bind_any is failed")
            #Returncode_t
            return bind_anyResponse("ERROR")   


    #Command_Interfaceのrelease()
    #BindしていたHRI-Cを開放する関数
    def release(self,re_req):
        print("Release")
        try:

            self.BINDCOMP = None
            return releaseResponse("OK")
        except:
            return releaseResponse("ERROR")





    # Command_Interfaceのset_parameter()2
    # Speech_Synthesis用    
    def set_speech(self,s_req):
        print(self.text)

        if self.ENGINE_RECEIVABLE:
            command_id = "set_speech_synthesis"
            self.text = s_req.speech_text
        
            try:
                # self.set_text = rospy.ServiceProxy('/speech_set_param', speech_set_param)
                s_result = self.set_text(self.text)
                print(s_result)

                if s_result.set_return == "OK":
                    return speech_set_parameterResponse(s_result.set_return,command_id)
                else :
                    return speech_set_parameterResponse(s_result.set_return,command_id)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed")
                returncode_t = "ERROR"
                
                return speech_set_parameterResponse(returncode_t,command_id)
        else:
            print(f"Command {command_id} is failed")
            returncode_t = "ERROR"
            return speech_set_parameterResponse(returncode_t, command_id)

    # Command_Interfaceのset_parameter()
    # Speech_Synthesis用   
    def get_speech(self,g_req):
        print("get_parameter")
        if self.ENGINE_RECEIVABLE:
            try:
                self.get_text = rospy.ServiceProxy('/speech_get_param', speech_get_param)
                                
                g_text = self.get_text()
                self.text = g_text.speech_text
                print(self.text)
                re = "OK"
                return speech_get_parameterResponse(re,self.text)

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                self.text = " "
                re ="ERROR"
                return speech_get_parameterResponse(re,self.text)
        else:
            print("get parameter is failed")
            self.text = " "
            re ="ERROR"
            return speech_get_parameterResponse(re,self.text)




    #Component_statusを取得するようの関数
    def component_status(self, co_name):
        try:
            state_name = '/get_state/' + self.component_ref
            get_component_status = rospy.ServiceProxy(state_name, component_status)
            return get_component_status(co_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return get_component_status(co_name)


    def add_item(self, _list, item):
        if item[1] not in _list:
            _list.append(item)
            print(f"{item} を追加しました")
            # print(_list)
            return True
        else:
            print(f"{item} は既にリストに存在します")
            print(_list)
            return False


    def remove_item(self, _list, item):
        if item  in _list:
            _list.remove(item)
            print(f"{item} を削除しました")
            # print(_list)
            return True
        else:
            print(f"{item} はリストにありません")
            return False


    def notify_event(self,event_id, event_type, subscribe_id):
        rospy.loginfo(f'event_id:{event_id}, event_type:{event_type}, subscribe_id:{subscribe_id}')
        
        notify_event_pub = notifyevent()
        notify_event_pub.event_id = event_id
        notify_event_pub.event_type = event_type
        notify_event_pub.subscribe_id = subscribe_id
        print("pub event_id")
        self.notify_event_pub.publish(notify_event_pub)


    def event_recognized_cb(self,req):

        print(req.event_id)

        if "recog" in req.event_id:
            data_list = self.speech_recognition_list
            print(data_list)

            search_event_id = req.event_id

            for data in data_list:

                if data[0] == search_event_id:
                    print(data[0])
                    timestamp = data[1][1]
                    recognized_text = data[1][2]
                    print(f"timestamp: {timestamp}")
                    print(f"recognized_text: {recognized_text}")
                    return get_event_detail_speech_recognizedResponse(timestamp,recognized_text)
            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")
                return get_event_detail_speech_recognizedResponse("", "")



    def event_detected_cb(self,gr_req):
        if "detected" in gr_req.event_id:
            data_list = self.person_detected_list
            print(data_list)

            search_event_id = gr_req.event_id

            for data in data_list:

                # 文字列内に0番目の要素が含まれているかチェック
                if data[0] == search_event_id:
                    print(f"関連データ: {data[1][1]}")
                    
                    return get_event_detail_person_detectedResponse(data[1][1],data[1][2])


            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")     
                get_event_detail_person_detectedResponse("",0)   


    def event_local_cb(self,gr_req):
                

        if "local" in gr_req.event_id:
            data_list = self.person_position_list
            print(self.person_position_list)
            print(data_list)

            search_event_id = gr_req.event_id

            for data in data_list:
                key = data[0]  # 各配列の0番目の要素を取得
                
                # 文字列内に0番目の要素が含まれているかチェック
                if data[0] == search_event_id:
                    print(data[0])
                    print(f"関連データ: {data[1][1]}")
                    return get_event_detail_person_localizedResponse(data[1][1],data[1][2],data[1][3])
                    break  

            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")        
                return get_event_detail_person_localizedResponse("", "")


    def event_identified_cb(self,gr_req):
        print(gr_req.event_id)
                

        if "identi" in gr_req.event_id:
            data_list = self.person_identi_list
            print(self.person_identi_list)

            search_event_id = gr_req.event_id

            for data in data_list:
                key = data[0]  # 各配列の0番目の要素を取得
                
                # 文字列内に0番目の要素が含まれているかチェック
                if data[0] == search_event_id:
                    print(data[0])
                    print(f"関連データ: {data[1][2]}")
                    return get_event_detail_person_identifiedResponse(data[1][1],[data[1][2]])
                    break  

            else:
                print(f"{search_event_id}に一致するデータは見つかりませんでした。")        
                return get_event_detail_person_identifiedResponse("", [""])

        # except Exception as e:
        #     print(e)
        #     return get_event_detail_person_identifiedResponse("", [""])



    def subscribe_cb(self, sub_req):
        # self.component_ref = "Speech_Recognition"
        self.subcount += 1
        self.component_ref = self.BINDCOMP
        print(self.component_ref)
        self.event_type = sub_req.event_type
        #self.event_id = sub_req.event_type
        self.subscribe_id = sub_req.event_type + str(self.subcount)
        
        
        #アプリに送信するためのトピック通信(notify_event())
        self.notify_event_pub = rospy.Publisher('/notify_event', notifyevent,queue_size = 1)
        self.notify_event_pub1 = rospy.Publisher('/notify_event_pub/recog', notifyevent,queue_size = 1)
        self.notify_event_pub2 = rospy.Publisher('/notify_event_pub/detec', notifyevent,queue_size = 1)
        self.notify_event_pub3 = rospy.Publisher('/notify_event_pub/local', notifyevent,queue_size = 1)
        self.notify_event_pub4 = rospy.Publisher('/notify_event_pub/inden', notifyevent,queue_size = 1)

        #コンポーネントとやり取りするためのサービス通信
        # sub_ = '/Subscribe/' + self.component_ref
        # _subscribe = rospy.ServiceProxy(sub_, subscribe_set)

        #コンポーネントからの結果を受信するためのトピック通信
        self.event_subscriber1 = rospy.Subscriber('/event_speechrec', notify_speechrec3,self.event_cb1)
        self.event_subscriber2 = rospy.Subscriber('/event_persondetec', notify_persondetect,self.event_cb2)
        self.event_subscriber3 = rospy.Subscriber('/event_localization', notify_localrec,self.event_cb3)
        self.event_subscriber4 = rospy.Subscriber('/event_identified', notify_identified,self.event_cb4)
        

        # print(sub_req)
        # if sub_set.result == "True":
        return subscribe_Response("OK",self.subscribe_id)
        # else :
        #     return subscribe_Response("ERROR",self.subscribe_id)



    def unsubscribe_cb(self, unsub_req):
        if self.subscribe_id == unsub_req.subscribe_id:
            print("AAA")


        try:
            print(f"unsubscribe: {unsub_req}")

            #結果を格納する
            # self.speech_recognition_list  = []
            # self.person_detected_list = []
            # self.person_position_list = []
            # self.person_identi_list = []

            #topicの購読をやめる
            self.event_subscriber1.unregister()
            self.event_subscriber2.unregister()
            self.event_subscriber3.unregister()
            self.event_subscriber4.unregister()

            #アプリへ通知送信をやめる
            self.notify_event_pub1.unregister()
            self.notify_event_pub2.unregister()
            self.notify_event_pub3.unregister()
            self.notify_event_pub4.unregister()

            return unsubscribe_Response("OK")

        except:
            return unsubscribe_Response("ERROR")


    # execute()を要求されたときの関数
    def execute_cb(self, exe_req):
        print(f"{exe_req.command_unit_list} {self.component_ref}")
        
        #要求されたcommand
        self.command = exe_req.command_unit_list

        goal = executeGoal()


        
        try:

            if self.command == "start":
                # self.component_ref = "Speech_Synthesis"
                self.component_ref = self.BINDCOMP
                print(self.component_ref)
                #コンポーネントの状態をとるサービス通信
                self.c_status = self.component_status(self.component_ref) 
                print(f"{self.component_ref} is {self.c_status.status}")
                
                #コンポーネントがREADYなら

                if self.c_status.status == "READY":

            
                    try:
                        #コンポーネントのstart()メソッドを指定する
                        goal.command_name = self.command
                        print(goal.command_name)

                        #アクションリクエストを送信
                        self.client.send_goal(goal, done_cb=self.action_done_callback)

                        #メッセージを遅れたらOKを返答
                        return_t ="OK"
                        
                        #Returncode_t
                        return executeResponse(return_t)
                    except Exception as e :
                        return_t ="ERROR"
                        print("aaa")
                        rospy.logerr("Connection failed: %s", str(e)) 
                        return executeResponse(return_t)

                #コンポーネントが準備完了ではないなら
                else:
                    
                    rospy.loginfo(f"This component is {self.c_status.status}")
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)

            elif self.command == "stop":
                print("stop")
                try:
                    goal.command_name = self.command #コンポーネントのstop()メソッドを指定する
                    self.client.send_goal(goal, done_cb=self.action_done_callback)#アクションリクエストを送信

                    return_t ="OK"
                    #Returncode_t
                    return executeResponse(return_t)
                except:
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)
                    
            elif self.command == "suspend":
                print("suspend")

                try:
                    goal.command_name = self.command #コンポーネントのsuspend()メソッドを指定する
                    self.client.send_goal(goal, done_cb=self.action_done_callback)#アクションリクエストを送信

                    return_t ="OK"
                    #Returncode_t
                    return executeResponse(return_t)

                except:
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)            
            
            elif  self.command == "resume":
                print("resume")
                try:
                    goal.command_name = self.command #コンポーネントのsuspend()メソッドを指定する
                    self.client.send_goal(goal, done_cb=self.action_done_callback)#アクションリクエストを送信

                    return_t ="OK"
                    return executeResponse(return_t)#Returncodeを返答                    

                except Exception as e :
                    rospy.logerr("Connection failed: %s", str(e)) 
                    return_t ="ERROR"
                    #Returncode_t
                    return executeResponse(return_t)
            
            else:
                return_t ="ERROR"
                #Returncode_t
                return executeResponse(return_t)

        except Exception as e :
            return_t ="ERROR"
            print("aaa")
            rospy.logerr("Connection failed: %s", str(e)) 
            print("finish")
            return executeResponse(return_t)


    def action_done_callback(self, cb_status, cb_result):# アクションが完了したら呼び出される
        # rospy.loginfo(f'Action completed with result: {result.return_t}')
        if cb_status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Action succeeded with result: {cb_result.success}")
        else:
            rospy.loginfo(f"Action failed or was canceled with state: {cb_status}")

    #Componentからの完了通知が来たらApp.に送信
    def completed(self,c_req):
        self.command_count += 1
        command_id = c_req.command_id + str(self.command_count)
        status = c_req.status
        rospy.loginfo(f"command_id {command_id} is {status}")
        
        completed_Pub = completed()
        completed_Pub.command_id = command_id
        completed_Pub.status = status
        self.completed_pub.publish(completed_Pub)

        # if "Speech" in command_id:
        #     self.completed_Pub1 = completed()

        #     if self.add_item(self.command_list, [command_id,status]):
        #         print("publish command_id")
        #         self.completed_Pub1.command_id = command_id
        #         self.completed_Pub1.status = status
        #         self.completed_pub1.publish(self.completed_Pub1)

        # elif "Move" in command_id:
        #     self.completed_Pub2 = completed()

        #     if self.add_item(self.command_list, [command_id,status]):
        #         print("publish command_id")
        #         self.completed_Pub2.command_id = command_id
        #         self.completed_Pub2.status = status
        #         self.completed_pub2.publish(self.completed_Pub2)


    
    def get_command_result_cb(self,req):
        print("get_command_result")

        data_list = self.command_list

        search_command_id = req.command_id

        for data in data_list:
            if data[0] == search_command_id:
                command_id = data[0]
                result = data[1]
                print(f"command_id: {command_id}")
                print(f"result: {result}")
                return get_command_resultResponse(result)
        else:
            print(f"{search_command_id}に一致するデータは見つかりませんでした。")
            return get_command_resultResponse("")




if __name__ == "__main__":
    rospy.init_node('Engine_sample')
    print("time")
    try:
        _service = EngineService()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        print("finish")
        exit


