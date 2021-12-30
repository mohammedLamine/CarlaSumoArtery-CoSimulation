# CarlaSumoArtery-CoSimulation

## Download Requirements
- CARLA v.0.9.11
- Artery
- Sumo

## Installation
### Pre-requisites
- Ensure CARLA and Artery and Sumo are installed and working

### Add the following folders
- In Artery, add the folder named "artery/src"
- In CARLA, add the folder named "carla/Co-Simulation"
- Add the folder named "simulation_modules" next to the carla folder

### Testing
- Run server mode carla: ```/Path/To/Carla/CarlaUE4.sh -ResX=600 -ResY=400 -carla-server```
- Run Sumo with two clients (cars) and using Town04 config: ```sumo-gui --remote-port 8813 --num-clients 2 -c Path/To/Carla/Co-Simulation/Sumo/examples/Town04.sumocfg  --step-length 0.05```
- Connect as a Carla client:
  - ```cd to parent folder of carla and simulation_modules``` 
  - ```python3.7 synchro_client.py```

- Connect an Artery client:
  - ```cd Path/To/Carla/Co-Simulation/arterysim```
  - ```Path/To/Artery/build/run_artery.sh sumo-omnetpp-med.ini``` 


## Adding new security features

## How it works
- The synchro_client.py script uses carlas's "run_synchronization.py" to synchronize carla and sumo and then in the simulation loop looks for a connection from Artery simulation to start recieving messages. The script also looks for defined attackers and detection mechanisms and runs them in the same loop and plots corresponding visualization in carla.

### new detector
- Adding a new detector should be done as follows :
  - Create a class for the detector (subclass of Detector)
  - Implements the function check().
     - check() takes as argument the current step cam messages ( which defines wether the detector is a global one or a local one depending if it receives all messages or just specific messages received by one vehicle )
     - check() then retuns the ids of detected misbehaving vehicles. 
### new attack
- Adding a new attacker should be done as follows : 
  - Create a class for the attacker (subclass of Attacker)
  - Reimplement the is_ready() function if necessary (if not, the attacker will perform an attack every simulation step).
  - implement the perform_attack() function.
    - perform_attack() function must create a carla object in the position where the attack will happen.

## Misc
### Reference
If you use our work, please cite us!
```
@inproceedings{bouchouia2022simulator,
  title={A simulator for cooperative and automated driving security},
  author={Bouchouia, Mohammed and Monteuuis, Jean-Philippe and Labiod, Houda and Jaballah, Wafa Ben and Petit, Jonathan},
  year={2022}
}

```
### Authors
- ``` Mohammed Lamine Bouchouia ```
- ``` Jean-Philippe Monteuuis ```
- ``` Houda Labiod ```
- ``` Ons Jelassi ```
- ``` Wafa Ben Jaballah ```
- ``` Jonathan Petit ```

### Contact:
A question? Send us an email: ``` mohammed.bouchouia@telecom-paris.fr ``` 
