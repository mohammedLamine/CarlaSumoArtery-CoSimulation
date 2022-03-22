"""
Script to inject attacks in recieved artery messages in python
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================
import carla
import numpy as np
from collections import deque

class Detector(object):

    def __init__(self,net):
        self.previous_messages_per_host={}
        self.current_messages_per_host={}
        self.net = net
    def check(self,artery2sumo_ids,current_step_cams,arteryAttackers_ids):
        return []

class SSC(Detector):# Simple Speed Check (SSC)
    def __init__(self,net,threshold):
        super().__init__(net)
        self.threshold=threshold

    def ssc_check(self,current_cam,previous_cam): 
        estimated_spd = np.sqrt((previous_cam['sender_pos_x'] - current_cam['sender_pos_x'])**2+( previous_cam['sender_pos_y']-current_cam['sender_pos_y'])**2)/((current_cam['Generation Delta Time']- previous_cam['Generation Delta Time'])/100000)
        current_cam['estimated_spd']=estimated_spd

        current_cam['speed_diff']=current_cam['Speed']-estimated_spd
        return abs(current_cam['speed_diff'])>self.threshold

    def check_cam(self,cam):
        previous_message = self.previous_messages_per_host.get((cam['Station ID'],cam['receiver_artery_id']))
        if not previous_message :
            # if not self.current_messages_per_host.get((cam['Station ID'],cam['receiver_artery_id'])):
            #     self.current_messages_per_host[(cam['Station ID'],cam['receiver_artery_id'])]= cam
            return False
        return self.ssc_check(cam,previous_message)

    def check(self,artery2sumo_ids,current_step_cams,arteryAttackers_ids):
        detections= set([])
        self.current_messages_per_host={}
        for cam in current_step_cams:
            if not cam['receiver_artery_id'] in artery2sumo_ids:
                artery2sumo_ids[cam['receiver_artery_id']] = cam['receiver_sumo_id']
            self.current_messages_per_host[(cam['Station ID'],cam['receiver_artery_id'])]= cam
            cam['ssc']=self.check_cam(cam)
            if cam['ssc']:
                if cam['Station ID'] in artery2sumo_ids:
                    detections.add(artery2sumo_ids[cam['Station ID']])
        self.previous_messages_per_host.update(self.current_messages_per_host)

        return detections

class RLDetector(Detector):

    def __init__(self,rl_model):
        self.rl_model = rl_model
        self.current_state = np.zeros(self.rl_model.train_py_env.latent_history_size) # latent init
        self.previous_action =-1
        self.las_1000_check = deque([0.5], maxlen = 1000)
        self.las_1000_check_on_attacks = deque([0.5], maxlen = 1000)


    def check(self,artery2sumo_ids,current_step_cams,arteryAttackers_ids):
        detections= set([])
        all_logits = []
        for current_msg in current_step_cams:
            
            state = np.concatenate((self.current_state,self.rl_model.train_py_env.numerize_input(current_msg),[self.previous_action])).astype('float32')
            res = self.rl_model.agent._q_network.call(np.array(state).reshape(-1, *np.array(state).shape))
            self.current_state = self.rl_model.train_py_env.update_net.__call__(np.array(state).reshape(-1, *np.array(state).shape)).numpy()[0]
            # print(res[0].numpy())
            all_logits.append(res[0].numpy())
            self.previous_action=res[0].numpy().argmax()
            # print(self.previous_action)
            if self.previous_action:
                if current_msg['Station ID'] in artery2sumo_ids:
                    detections.add(artery2sumo_ids[current_msg['Station ID']])
                self.las_1000_check_on_attacks.append((current_msg['Station ID'] in arteryAttackers_ids)==self.previous_action)
            self.las_1000_check.append((current_msg['Station ID'] in arteryAttackers_ids)==self.previous_action)
        print("avereage perf on last 1000 messages",np.mean(self.las_1000_check),np.mean(self.las_1000_check_on_attacks))
        return detections