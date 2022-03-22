"""
Script to integrate recieve artery messages in python for each simulation step
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

from ast import Try
import numpy as np
import errno, time
import socket
import select
import carla
import traci
import logging
import traceback

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position

HOST = '127.0.0.1'  
PORT = 65432        


class ArterySynchronization(object):

    def __init__(self):
        self.conn = None
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((HOST, PORT))
        self.s.setblocking(0)
        self.s.listen(5)
        self.on_going_message_recv={"state":False}

    def checkAndConnectclient(self):
        if not self.is_connected():
            try :
                readable, _, _ = select.select([self.s], [], [],0.01)
                if readable:
                    print("connecting artery client")
                    self.conn, _ = self.s.accept() 
                    self.conn.setblocking(0)
            except Exception as e:
                print("Socket check problem")
                logging.error(traceback.format_exc())    
                
    def is_connected(self):
        return not (self.conn is None)
        
    def getCarlaLocation(self,synchronization,cam ):

        x,y = synchronization.net.convertLonLat2XY(cam['receiver_long'],cam['receiver_lat'])
        extent    = carla.Vector3D(cam['Vehicle_Length'] / 200.0, cam['Vehicle_Width'] / 200.0, 0/ 2.0)
        transform = carla.Transform(carla.Location(x, y, 0),
                                carla.Rotation())
        receiver_carla_transform = BridgeHelper.get_carla_transform(transform, extent)
        cam['receiver_pos_x']=receiver_carla_transform.location.x
        cam['receiver_pos_y']=receiver_carla_transform.location.y

        x,y = synchronization.net.convertLonLat2XY(cam['Longitude'],cam['Latitude'])
        extent    = carla.Vector3D(cam['Vehicle_Length'] / 400.0, cam['Vehicle_Width'] / 400.0, 0/ 2.0)
        transform = carla.Transform(carla.Location(x, y, 0),
                                carla.Rotation())
        sender_carla_transform = BridgeHelper.get_carla_transform(transform, extent)
        cam['sender_pos_x']=sender_carla_transform.location.x
        cam['sender_pos_y']=sender_carla_transform.location.y


    def camToDict(self,synchronization,cam):
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

        full_cam['Vehicle_Width']=float(full_cam['Vehicle Width'])
        full_cam.pop('Vehicle Width')

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

        # adding transformations
        self.getCarlaLocation(synchronization,full_cam)
        if not full_cam['receiver_artery_id'] in synchronization.arteryAttackers_ids:
            if full_cam['receiver_sumo_id'].startswith('carla'):
                synchronization.arteryAttackers_ids.add(full_cam['receiver_artery_id'] )

        if not full_cam['receiver_artery_id'] in synchronization.artery2sumo_ids:
            synchronization.artery2sumo_ids[full_cam['receiver_artery_id']] = full_cam['receiver_sumo_id']

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


    def recieve_cam_messages(self,synchronization):
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
                if not data is None :
                    if len(data) == 6:
                        self.set_ongoing_recv(b'', int(data.decode('utf-8')))
                if e.errno != errno.EWOULDBLOCK: 
                    print("e.errno != errno.EWOULDBLOCK",e)
                    logging.error(traceback.format_exc())
                    print(data)
                break
            full_cam = self.camToDict(synchronization,data.decode('utf-8'))
            current_step_cams.append(full_cam)
        return current_step_cams

    def shutdownAndClose(self):
        try :
            self.s.shutdown(socket.SHUT_RDWR)
            self.s.close()
        except OSError as e :
            print("trying to close already closed socket")
            