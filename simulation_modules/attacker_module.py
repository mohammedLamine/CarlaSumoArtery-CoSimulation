"""
Script to inject attacks in recieved artery messages in python
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================
import carla
from matplotlib import transforms
import numpy as np
class Attacker(object):

    def __init__(self,sumo_id, distance_multiplier=150):
        self.sumo_id = sumo_id
        self.carla_id = None
        self.ghosts=[]
        self.distance_multiplier = distance_multiplier
    def perform_attack(self):
        pass
    
    def is_ready(self,sumo2carla_ids):
        self.carla_id= sumo2carla_ids.get(self.sumo_id)
        return self.carla_id


    def kill_ghosts(self,synchronization):
        for ghost_id in self.ghosts :
            synchronization.sumo.unsubscribe(synchronization.carla2sumo_ids[ghost_id])
            synchronization.sumo.destroy_actor(synchronization.carla2sumo_ids.pop(ghost_id))
            synchronization.carla.destroy_actor(ghost_id)
        self.ghosts=[]

    def createGhostVehicle(self,carla_client,ghost_vehicle_bp,new_location,new_control):
        if len(self.ghosts)==0:
            self.spawnGhost(carla_client,ghost_vehicle_bp,new_location,new_control)
        else :
            carla_client.synchronize_vehicle(self.ghosts[0],new_location)


    def spawnGhost(self,carla_client,ghost_vehicle_bp,location,ghost_control):
        spawned_actor_id = carla_client.spawn_actor(ghost_vehicle_bp, location)
        if spawned_actor_id<0 :
            return
        actor=carla_client.world.get_actor(spawned_actor_id)
        if actor : 
            actor.set_autopilot(True)
            actor.set_simulate_physics(enabled=False)  
            self.ghosts.append(actor.id)
            actor.apply_control(ghost_control)

class GhostAheadAttacker(Attacker):

    def perform_attack(self,synchronization,ghost_vehicle_bp):

        if self.is_ready(synchronization.sumo2carla_ids):
            new_location,ghost_control = self.computeGhostPositionAndControl(synchronization.carla)
            self.createGhostVehicle(synchronization.carla,ghost_vehicle_bp,new_location,ghost_control)
       
    def computeGhostPositionAndControl(self,carla_client):

            actor=carla_client.world.get_actor(self.carla_id)
            ghost_control = actor.get_control()
            
            new_location= actor.get_transform()
            new_location.location += new_location.get_forward_vector()*self.distance_multiplier
            new_location = carla_client.world.get_map().get_waypoint(new_location.location).transform
            new_location.location=new_location.location+carla.Vector3D(0,0,1)

            return new_location,ghost_control


class PositionAttacker(Attacker):
    def perform_attack(self,synchronization,ghost_vehicle_bp):
        if self.is_ready(synchronization.sumo2carla_ids):
            new_location,ghost_control = self.computeNewPositionAndControl(synchronization.carla)
            self.createGhostVehicle(synchronization.carla,ghost_vehicle_bp,new_location,ghost_control)
       

    def computeNewPositionAndControl(self,carla_client):
        pass

class RandomOnRoadPositionAttacker(PositionAttacker):
    
    def computeNewPositionAndControl(self,carla_client):
        actor=carla_client.world.get_actor(self.carla_id)
        control = actor.get_control()

        spawn_points = carla_client.world.get_map().get_spawn_points()
        new_position =spawn_points[np.random.randint(0,len(spawn_points))]
        return new_position , control

class RandomPositionAttacker(PositionAttacker):
    
    def computeNewPositionAndControl(self,carla_client):
        pass


class RandomOffsetPositionAttacker(PositionAttacker):
    
    def computeNewPositionAndControl(self,carla_client,offset_std= 50):
        actor=carla_client.world.get_actor(self.carla_id)
        control = actor.get_control()
        new_position = actor.get_transform()

        new_position.location.x +=  np.random.uniform(-offset_std,+offset_std)
        new_position.location.y +=  np.random.uniform(-offset_std,+offset_std)

        return new_position , control


class ConstantOffsetPositionAttacker(PositionAttacker):
    
    def computeNewPositionAndControl(self,carla_client,offset_x= 50,offset_y= 50):
        actor=carla_client.world.get_actor(self.carla_id)
        control = actor.get_control()
        new_position = actor.get_transform()

        new_position.location.x += offset_x
        new_position.location.y += offset_y

        return new_position , control