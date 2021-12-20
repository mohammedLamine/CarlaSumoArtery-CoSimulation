"""
Script to integrate recieve artery messages in python for each simulation step
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import numpy as np
import errno
import socket
import select
HOST = '127.0.0.1'  
PORT = 65432        


class ArterySynchronization(object):

    def __init__(self):
        self.conn = None
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
        self.s.bind((HOST, PORT))
        self.s.setblocking(0)
        self.s.listen(5)
        self.on_going_message_recv={"state":False}

    def checkAndConnectclient(self):
        if not self.is_connected():
            print('checking readable')
            readable, _, _ = select.select([self.s], [], [],0.01)
            if readable:
                print("connecting artery client")
                self.conn, _ = self.s.accept() 
                self.conn.setblocking(0)

    def is_connected(self):
        return not (self.conn is None)
                
    def camToDict(self,cam):
        full_split=[x.split(':') for x in cam.split('\n')]
        full_cam= {k:v for k,v in full_split[:-1]}

        full_cam['receiver_artery_id']=int(full_cam['receiver_artery_id'])
        # full_cam['receiver_sumo_id']=str(full_cam['receiver_sumo_id'])
        full_cam['receiver_long']= np.double(full_cam['receiver_long'])
        full_cam['receiver_lat']=np.double(full_cam['receiver_lat'])
        full_cam['receiver_speed']= np.double(full_cam['receiver_speed'])

        # full_cam['ITS PDU Header']=str(full_cam['ITS PDU Header'])
        full_cam['Protocol Version']=int(full_cam['Protocol Version'])
        full_cam['Message ID']=int(full_cam['Message ID'])
        full_cam['Station ID']=int(full_cam['Station ID'])
        # full_cam['CoopAwarensess']=str(full_cam['CoopAwarensess'])
        full_cam['Generation Delta Time']=int(full_cam['Generation Delta Time'])
        # full_cam['Basic Container']=str(full_cam['Basic Container'])
        full_cam['Station Type']=int(full_cam['Station Type'])
        # full_cam['Reference Position']=str(full_cam['Reference Position'])
        full_cam['Longitude']=float(full_cam['Longitude'])/10**7
        full_cam['Latitude']=float(full_cam['Latitude'])/10**7
        full_cam['Semi Major Orientation']=int(full_cam['Semi Major Orientation'])
        full_cam['Semi Major Confidence' ]=int(full_cam['Semi Major Confidence'])
        full_cam['Semi Minor Confidence' ]=int(full_cam['Semi Minor Confidence'])
        # full_cam['Altitude [Confidence]']=str(full_cam['Altitude [Confidence]'])
        full_cam['Altitude']=float(full_cam['Altitude [Confidence]'].split('[')[0])
        full_cam['Altitude_Confidence']=float(full_cam['Altitude [Confidence]'].split('[')[1].split(']')[0])
        full_cam.pop('Altitude [Confidence]')
        # full_cam['High Frequency Container [Basic Vehicle]']=str(full_cam['High Frequency Container [Basic Vehicle]'])

        # full_cam['Heading [Confidence]']=str(full_cam['Heading [Confidence]'])
        full_cam['Heading']=float(full_cam['Heading [Confidence]'].split('[')[0])
        full_cam['Heading_Confidence']=float(full_cam['Heading [Confidence]'].split('[')[1].split(']')[0])
        full_cam.pop('Heading [Confidence]')

        # full_cam['Speed [Confidence]']=str(full_cam['Speed [Confidence]'])

        full_cam['Speed']=float(full_cam['Speed [Confidence]'].split('[')[0])
        full_cam['Speed_Confidence']=float(full_cam['Speed [Confidence]'].split('[')[1].split(']')[0])
        full_cam.pop('Speed [Confidence]')

        full_cam['Drive Direction']=int(full_cam['Drive Direction'])
        full_cam['Longitudinal Acceleration']=float(full_cam['Longitudinal Acceleration'])

        # full_cam['Vehicle Length [Confidence Indication]']=str(full_cam['Vehicle Length [Confidence Indication]'])
        full_cam['Vehicle_Length']=float(full_cam['Vehicle Length [Confidence Indication]'].split('[')[0])
        full_cam['Vehicle_Length_Confidence']=float(full_cam['Vehicle Length [Confidence Indication]'].split('[')[1].split(']')[0])
        full_cam.pop('Vehicle Length [Confidence Indication]')

        full_cam['Vehicle Width']=float(full_cam['Vehicle Width'])
        # full_cam['Curvature [Confidence]']=str(full_cam['Curvature [Confidence]'])
        full_cam['Curvature']=float(full_cam['Curvature [Confidence]'].split('[')[0])
        full_cam['Curvature_Confidence']=float(full_cam['Curvature [Confidence]'].split('[')[1].split(']')[0])
        full_cam.pop('Curvature [Confidence]')

        full_cam['Curvature Calculation Mode']=int(full_cam['Curvature Calculation Mode'])
        # full_cam['Yaw Rate [Confidence]']=str(full_cam['Yaw Rate [Confidence]'])
        full_cam['Yaw_Rate']=float(full_cam['Yaw Rate [Confidence]'].split('[')[0])
        full_cam['Yaw_Rate_Confidence']=float(full_cam['Yaw Rate [Confidence]'].split('[')[1].split(']')[0])
        full_cam.pop('Yaw Rate [Confidence]')

        
        # full_cam['Low Frequency Container']=str(full_cam['Low Frequency Container'])

        # removing non usefull keys
        full_cam.pop('Low Frequency Container')
        full_cam.pop('High Frequency Container [Basic Vehicle]')
        full_cam.pop('Reference Position')
        full_cam.pop('Basic Container')
        full_cam.pop('ITS PDU Header')

        return full_cam

    def get_ongoing_recv( self ):
        if  self.on_going_message_recv['state'] :
            data = self.conn.recv(self.on_going_message_recv['full_message_size']-len(self.on_going_message_recv['fragment']))
            self.on_going_message_recv['fragment']=self.on_going_message_recv['fragment']+data
            if len(self.on_going_message_recv['fragment'])!= self.on_going_message_recv['full_message_size']:
                self.on_going_message_recv['state']=True
            else:
                self.on_going_message_recv['state']=False
        return self.on_going_message_recv['state']


    def set_ongoing_recv( self,data,size ):
        if len(data)!= size:
            self.on_going_message_recv['state']=True
            self.on_going_message_recv['fragment']=data
            self.on_going_message_recv['full_message_size']=size
        return self.on_going_message_recv['state']


    def recieve_cam_messages(self):
        current_step_cams=[]
        while True:
            data = None
            try :
                if self.on_going_message_recv['state']:
                    if  self.get_ongoing_recv():
                        break
                    else:
                        data = self.on_going_message_recv['fragment']
                        if self.on_going_message_recv['full_message_size'] ==6 :
                            next_cam_size = int(data.decode('utf-8'))
                            data = self.conn.recv(next_cam_size)
                            if self.set_ongoing_recv(data,next_cam_size):
                                continue
                else :
                    data = self.conn.recv(6)
                    if len(data)<=0:
                        break
                    if self.set_ongoing_recv(data,6):
                        continue
                    next_cam_size = int(data.decode('utf-8'))
                    data = self.conn.recv(next_cam_size)
                    if self.set_ongoing_recv(data,next_cam_size):
                        continue
            except IOError as e:
                if e.errno != errno.EWOULDBLOCK: 
                    print(e)
                    print(data)
                break
            full_cam = self.camToDict(data.decode('utf-8'))
            current_step_cams.append(full_cam)
        return current_step_cams