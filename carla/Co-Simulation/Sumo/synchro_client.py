#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time
import carla
import pandas as pd
# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys
from numpy import  random
import socket
import select
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position
import run_synchronization
from run_synchronization import SimulationSynchronization  
# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================
cams=[]
def camToDict(cam):
    full_split=[x.split(':') for x in cam.split('\n')]
    return {k:v for k,v in full_split[:-1]}

def recieve_cam_messages(s,conn,addr):

            # print('Connected by', addr)
            while True:
                data = None
                try :
                    # receiver data 
                    data = conn.recv(6)
                    if len(data)<=0:
                        break
                    next_cam_size = int(data.decode('utf-8'))
                    data = conn.recv(next_cam_size)
                    receiver_data = camToDict(data.decode('utf-8'))
                    # cam data
                    data = conn.recv(6)
                    if len(data)<=0:
                        break
                    next_cam_size = int(data.decode('utf-8'))
                    data = conn.recv(next_cam_size)
                    cams.append({**receiver_data, **camToDict(data.decode('utf-8'))})
                except Exception as e :
                    break
                if not data:
                    print('not data')
                    break



def color_agents(world,sumo2carla_ids,victimes,attackers):
    debug = world.debug
    attackers_carla_ids = [sumo2carla_ids.get(actor_id) for actor_id in attackers]
    victimes_carla_ids = [sumo2carla_ids.get(actor_id) for actor_id in victimes]
    for actor_id in attackers_carla_ids:
        if actor_id:
            actor=world.get_actor(actor_id)
            debug.draw_box(carla.BoundingBox(actor.get_transform().location,carla.Vector3D(1,0.5,1)),actor.get_transform().rotation, 3, carla.Color(255,0,0),0.05)
    
    for actor_id in victimes_carla_ids:
        if actor_id:
            actor=world.get_actor(actor_id)
            debug.draw_box(carla.BoundingBox(actor.get_transform().location,carla.Vector3D(1,0.5,1)),actor.get_transform().rotation, 3, carla.Color(0,255,0),0.05)


def attacker5(world,sumo2carla_ids,ghosts,distance_multiplier,vehicle_bp,attackers,synchronization):
    debug = world.debug
    attackers_carla_ids = [sumo2carla_ids.get(actor_id) for actor_id in attackers]

    for ghost_id in ghosts :
        world.get_actor(ghost_id).destroy()
    ghosts=[]

    for actor_id in attackers_carla_ids:
        if actor_id:
            actor=world.get_actor(actor_id)
            controlght = actor.get_control()


            new_location= actor.get_transform()
            new_location.location += new_location.get_forward_vector()*distance_multiplier
            # if world.get_map().get_waypoint(new_location.location).road_id not in [33,779,775]:
            #     continue
            #print(world.get_map().get_waypoint(new_location.location).road_id)
            new_location = world.get_map().get_waypoint(new_location.location).transform
            new_location.location=new_location.location+carla.Vector3D(0,0,1)
            if len(ghosts)==0:
                print("creating the ghost")
                #spawn_points = world.get_map().get_spawn_points()
                actor = world.try_spawn_actor(vehicle_bp, new_location)
                if actor : 
                    actor.set_autopilot(True)
                    actor.set_simulate_physics(enabled=False)  
                    ghosts.append(actor.id)
                    actor.apply_control(controlght)
                    # print(new_location.location)
                    # print(world.get_map().transform_to_geolocation(new_location.location))
            debug.draw_box(carla.BoundingBox(new_location.location+carla.Vector3D(0,0,15) ,carla.Vector3D(1,0.5,1)),new_location.rotation, 3, carla.Color(75,0,130),0.11)

    return ghosts

def attacker4(world,sumo2carla_ids,ghosts,distance_multiplier,vehicle_bp,attackers,synchronization):
    debug = world.debug
    attackers_carla_ids = [sumo2carla_ids.get(actor_id) for actor_id in attackers]
    if ghosts !=[]:
        for ghost_id in ghosts :
            synchronization.sumo.unsubscribe(synchronization.carla2sumo_ids[ghost_id])
            synchronization.sumo.destroy_actor(synchronization.carla2sumo_ids.pop(ghost_id))
            synchronization.carla.destroy_actor(ghost_id)

        ghosts=[]
        return ghosts
    for actor_id in attackers_carla_ids:
        if actor_id:
            actor=world.get_actor(actor_id)
            controlght = actor.get_control()
            
            new_location= actor.get_transform()
            new_location.location += new_location.get_forward_vector()*distance_multiplier
            # if world.get_map().get_waypoint(new_location.location).road_id not in [33,779,775]:
            #     continue
            #print(world.get_map().get_waypoint(new_location.location).road_id)
            new_location = world.get_map().get_waypoint(new_location.location).transform
            new_location.location=new_location.location+carla.Vector3D(0,0,1)
            if len(ghosts)==0:
                #print("creating the ghost")
                #spawn_points = world.get_map().get_spawn_points()
                spawned_actor_id = synchronization.carla.spawn_actor(vehicle_bp, new_location)
                actor=world.get_actor(spawned_actor_id)
                if actor : 
                    actor.set_autopilot(True)
                    actor.set_simulate_physics(enabled=False)  
                    ghosts.append(actor.id)
                    actor.apply_control(controlght)
                    print(new_location.location)
                    print(world.get_map().transform_to_geolocation(new_location.location))
            debug.draw_box(carla.BoundingBox(new_location.location+carla.Vector3D(0,0,15) ,carla.Vector3D(1,0.5,1)),new_location.rotation, 3, carla.Color(75,0,130),0.11)

    return ghosts

def attacker3(world,ghost_move_delta_time,ghosts,distance_multiplier,vehicle_bp,old_location):
    debug = world.debug
    world_snapshot = world.get_snapshot()
    for ghost_id in ghosts : 
        world.get_actor(ghost_id).destroy()
    ghosts=[]
    for actor_snapshot in world_snapshot:
        actual_actor = world.get_actor(actor_snapshot.id)
        if actual_actor.id in ghosts :
            continue

        if actual_actor.type_id.startswith('vehicle.mercedes.coup') :
            if not world.get_actor(actual_actor.id).is_alive:
                print("Attacker Died !!!!") 

            debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(1,0.5,1)),actor_snapshot.get_transform().rotation, 3, carla.Color(255,0,0),0.11)
            
            
            new_location= actor_snapshot.get_transform()
            new_location.location += new_location.get_forward_vector()*distance_multiplier
            if world.get_map().get_waypoint(new_location.location).road_id not in [33,779,775]:
                continue
            #print(world.get_map().get_waypoint(new_location.location).road_id)
            new_location = world.get_map().get_waypoint(new_location.location).transform
            new_location.location=new_location.location+carla.Vector3D(0,0,1)
            if len(ghosts)==0:
                #print("creating the ghost")
                #spawn_points = world.get_map().get_spawn_points()
                actor = world.try_spawn_actor(vehicle_bp, new_location)
                if actor : 
                    actor.set_autopilot(True)
                    actor.set_simulate_physics(enabled=False)  
                    ghosts.append(actor.id)
                    controlght = world.get_actor(actor_snapshot.id).get_control()
                    actor.apply_control(controlght)
                    print(actual_actor.id )
                    print(actual_actor.attributes )
                    print(actual_actor.semantic_tags )
            debug.draw_box(carla.BoundingBox(new_location.location+carla.Vector3D(0,0,15) ,carla.Vector3D(1,0.5,1)),actor_snapshot.get_transform().rotation, 3, carla.Color(255,255,0),0.11)
            
    # for ghost_id in ghosts :
    #     if not world.get_actor(ghost_id).is_alive :
    #         print('ghost died')
    #         world.get_actor(ghost_id).destroy()
    #         print(ghosts)
    #         ghosts=[]
    #         # actor = world.try_spawn_actor(vehicle_bp, new_location)
    #         # if actor : 
    #         #     print('new ghost created')
    #         #     ghosts[0] = actor.id
    #         #     actor.set_autopilot(True)
    #         #     actor.set_simulate_physics(enabled=False)  
    #     else :
    #         actor = world.get_actor(ghost_id)
            
    #         actor.set_location(new_location.location)

    #         ghost_move_delta_time =10
    return ghost_move_delta_time,old_location,ghosts



def attacker2(world,ghost_move_delta_time,ghosts,distance_multiplier,vehicle_bp,old_location):
    debug = world.debug

    world_snapshot = world.get_snapshot()

    for actor_snapshot in world_snapshot:
        
        actual_actor = world.get_actor(actor_snapshot.id)
        
        if actual_actor.id in ghosts :
            continue

        if actual_actor.type_id.startswith('vehicle.mercedes.coup') :
            if not world.get_actor(actual_actor.id).is_alive:
                print("Attacker Died !!!!") 
            if old_location == []:
                old_location = actor_snapshot.get_transform().location
                print("setting up the first ever location")
            else :
                debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(1,0.5,1)),actor_snapshot.get_transform().rotation, 3, carla.Color(255,0,0),0.11)
                
                diff_location =  actor_snapshot.get_transform().location - old_location
                
                new_location= actor_snapshot.get_transform()
                new_location.location += diff_location*distance_multiplier
                new_location = world.get_map().get_waypoint(new_location.location).transform

                if len(ghosts)==0:
                    print("creating the ghost")
                    spawn_points = world.get_map().get_spawn_points()
                    actor = world.spawn_actor(vehicle_bp, new_location)
                    if actor : 
                        print(actor)
                        actor.set_autopilot(True)  
                        ghosts.append(actor.id)
                debug.draw_box(carla.BoundingBox(new_location.location+carla.Vector3D(0,0,15) ,carla.Vector3D(1,0.5,1)),actor_snapshot.get_transform().rotation, 3, carla.Color(255,255,0),0.11)
                old_location = actor_snapshot.get_transform().location
                controlght = world.get_actor(actor_snapshot.id).get_control()

    for ghost_id in ghosts :
        if not world.get_actor(ghost_id).is_alive :
            print('ghost died')
            actor = world.spawn_actor(vehicle_bp, new_location)
            if actor : 
                ghosts[0] = actor.id
                actor.set_autopilot(True)  

        actor = world.get_actor(ghost_id)
        
        actor.set_location(new_location.location)
        actor.apply_control(controlght)
        ghost_move_delta_time =10
    return ghost_move_delta_time,old_location,ghosts


def attacker(world,ghost_move_delta_time,ghosts,distance_multiplier,vehicle_bp,old_location):
    actor_list = world.get_actors()
    debug = world.debug
    world_snapshot = world.get_snapshot()
    for actor_snapshot in world_snapshot:
        
        actual_actor = world.get_actor(actor_snapshot.id)
        if actual_actor.id in ghosts :
            continue

        if actual_actor.type_id.startswith('vehicle.mercedes.coup') :
            
            if old_location == []:
                old_location = actor_snapshot.get_transform().location
                
            else :
                ghost_move_delta_time-=1
                debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(1,0.5,1)),actor_snapshot.get_transform().rotation, 3, carla.Color(255,0,0),0.11)
                if ghost_move_delta_time <= 0:
                    diff_location =  actor_snapshot.get_transform().location - old_location
                    if diff_location.x+diff_location.y+diff_location.z == 0.0 :
                        continue
                    
                    new_location= actor_snapshot.get_transform()
                    new_location.location += diff_location*distance_multiplier+carla.Vector3D(0,0,0)
                    if len(ghosts)==0:
                        actor = world.try_spawn_actor(vehicle_bp, new_location)
                        if actor : 
                            actor.set_autopilot(True)  
                            ghosts.append(actor.id)
                    debug.draw_box(carla.BoundingBox(new_location.location ,carla.Vector3D(1,0.5,1)),actor_snapshot.get_transform().rotation, 3, carla.Color(255,255,0),0.11)
                    old_location = actor_snapshot.get_transform().location
                    controlght = world.get_actor(actor_snapshot.id).get_control()
    for ghost_id in ghosts :
        if not world.get_actor(ghost_id).is_alive :
            actor = world.try_spawn_actor(vehicle_bp, new_location)
            if actor : 
                ghosts[0] = actor.id
                actor.set_autopilot(True)  
        if ghost_move_delta_time <= 0:

            actor = world.get_actor(ghost_id)
            
            actor.set_location(new_location.location)
            actor.apply_control(controlght)
        else :
            ghost_move_delta_time =10
    return ghost_move_delta_time,old_location,ghosts


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    sumo_simulation = SumoSimulation(args.sumo_cfg_file,args.step_length , args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)


    distance_multiplier=150
    ghosts=[]
    old_location = []
    world = carla_simulation.world

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.*'))
    controlght=None
    ghost_move_delta_time= 10
    victimes=['8']
    attackers=['56','21','1']
    conn = None
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    s.bind((HOST, PORT))
    s.setblocking(0)
    s.listen(5)
    detroyed_one_sumo=False
    try:
        while True:

            start = time.time()
            # print(" before tick")

            synchronization.tick()
            # print("synchronization.tick()")
            # print("synchronization.sumo2carla_ids",synchronization.sumo2carla_ids)
            # print("synchronization.carla2sumo_ids",synchronization.carla2sumo_ids)
            # print("synchronization.sumo....",synchronization.sumo.destroyed_actors,synchronization.sumo.spawned_actors)
            # print("synchronization.carla....",synchronization.carla.destroyed_actors,synchronization.carla.spawned_actors)
            # print(" after tick")

            # if len(synchronization.sumo2carla_ids.keys()) >0 and not detroyed_one_sumo :
            #     synchronization.sumo.destroy_actor(list(synchronization.sumo2carla_ids.keys())[0])
            #     synchronization.sumo.unsubscribe(list(synchronization.sumo2carla_ids.keys())[0])
            #     detroyed_one_sumo=True
            color_agents(world,synchronization.sumo2carla_ids,victimes,attackers)
            if ghost_move_delta_time <= 0:
                ghosts= attacker4(world,synchronization.sumo2carla_ids,ghosts,distance_multiplier,vehicle_bp,attackers,synchronization)
                ghost_move_delta_time=10
            else:
                ghost_move_delta_time=ghost_move_delta_time-1 


            # if ghosts != []:
            #         print(ghosts)
            #         print([synchronization.carla2sumo_ids[x]  if x in synchronization.carla2sumo_ids.keys() else "not yet synchonized" for x in ghosts])


            readable, writable, errored = select.select([s], [], [],0.01)


            if readable and not conn:
                print("connect first time")
                conn, addr = s.accept() 
                conn.setblocking(0)

            if  not conn is None :
                recieve_cam_messages(s,conn, addr)



            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

            

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')
        pd.DataFrame(cams).to_csv('cams.csv')   

        synchronization.close()

        for actor in world.get_actors().filter('vehicle.*.*'):
            actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None     
        world.apply_settings(settings)
if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--sumo_cfg_file', type=str, help='sumo configuration file',default="/home/med/carla/Co-Simulation/Sumo/examples/Town04.sumocfg")
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=8813,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo', default=True)
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=6,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='carla')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
