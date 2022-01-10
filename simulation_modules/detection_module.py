"""
Script to inject attacks in recieved artery messages in python
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================
import carla
import numpy as np
class Detector(object):

    def __init__(self,net):
        self.previous_messages_per_host={}
        self.current_messages_per_host={}
        self.net = net
    def check(self,current_step_cams):
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

    def check(self,artery2sumo_ids,current_step_cams):
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