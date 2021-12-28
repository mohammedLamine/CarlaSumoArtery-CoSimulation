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
import numpy as np
import errno

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys
from numpy import  random, sqrt

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
import sumolib
net = sumolib.net.readNet('./carla/Co-Simulation/Sumo/examples/net/Town04.net.xml')

sys.path.append("./carla/Co-Simulation/Sumo")

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position
import run_synchronization
from run_synchronization import SimulationSynchronization  
from carla_artery_connection import ArterySynchronization
from attacker_module import GhostAheadAttacker
from painting_module import Painter
from detection_module import SSC
# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================
cams=[]


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    sumo_simulation = SumoSimulation(args.sumo_cfg_file,args.step_length , args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)

    synchronization.artery2sumo_ids={}
    synchronization.net = sumolib.net.readNet(args.sumo_net_file)

    world = carla_simulation.world

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.*'))
    victimes=[]
    attackers=[ GhostAheadAttacker('56'),
                GhostAheadAttacker('21'),
                GhostAheadAttacker('1')]
    artery_conn = ArterySynchronization()
    carla_painter = Painter(freq=carla_simulation.step_length*5)
    glonal_detector = SSC(synchronization.net,200)
    try:
        while True:

            start = time.time()
            # synchronize carla with sumo
            synchronization.tick()



            # Synchronize artery data with carla
            artery_conn.checkAndConnectclient()

            # perform all attacks
            for attacker in attackers:
                attacker.perform_attack(synchronization,vehicle_bp)

            # apply all detection mechanisms
            detections = set([])
            if  artery_conn.is_connected() :
                current_step_cams=artery_conn.recieve_cam_messages(synchronization)
                detections = glonal_detector.check(synchronization.artery2sumo_ids,current_step_cams)
                cams.extend(current_step_cams)
                # [carla_painter.color_communication(synchronization,cam) for cam in current_step_cams]

            # # color agents accordingly
            carla_painter.color_agents(synchronization,victimes,attackers,detections)

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
    argparser.add_argument('--sumo_cfg_file', type=str, help='sumo configuration file',default="./carla/Co-Simulation/Sumo/examples/Town04.sumocfg")
    argparser.add_argument('--sumo_net_file', type=str, help='sumo network file',default="./carla/Co-Simulation/Sumo/examples/net/Town04.net.xml")

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