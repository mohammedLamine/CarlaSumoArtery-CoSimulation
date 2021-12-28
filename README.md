# CarlaSumoArtery-CoSimulation

## Download Requirements
- CARLA v.0.9.11
- Artery

## Installation
### Pre-requisites
- Ensure CARLA and Artery are installed and working

### Add the following folders
- In Artery, add the folder named "artery/src"
- In CARLA, add the folder named "carla/Co-Simulation"
- Add the folder named "simulation_modules"

### Testing
- Run server mode carla: ```/Path/To/Carla/CarlaUE4.sh -ResX=600 -ResY=400 -carla-server```
- Run Sumo with two clients (cars) and using Town04 config: ```sumo-gui --remote-port 8813 --num-clients 2 -c Path/To/Carla/Co-Simulation/Sumo/examples/Town04.sumocfg  --step-length 1```
- Connect as a Carla client:
  - ```cd Path/To/Carla/Co-Simulation/Sumo/``` 
  - ```python3.7 Path/To/Carla/Co-Simulation/Sumo/run_synchronization.py Path/To/Carla/Co-Simulation/Sumo/examples/Town04.sumocfg --sumo-port 8813 --sumo-host 127.0.0.1 --tls-manager sumo --client-order 4 --step-length 0.01```

- Connect an Artery client:
  - ```cd Path/To/Carla/Co-Simulation/arterysim```
  - ```Path/To/Artery/build/run_artery.sh sumo-omnetpp-med.ini``` 


## Adding new security features
### new detector
- ``` Add description of where and how to add a new detector ``` 
### new attack
- ``` Add description of where and how to add a new attack ``` 


## Misc
### Reference
If you use our work, please cite us!
``` Add bibtex ```
### Authors
- ``` Author 1 ```
- ``` Author 2 ```
- ``` Author 3 ```
### Contact:
A question? Send us an email: ``` Add an email ``` 
