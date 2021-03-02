import logging
import traci
from sumolib import checkBinary

def flatten(l):
    # A basic function to flatten a list
    return [item for sublist in l for item in sublist]

def setUpSimulation(configFile, trafficScale, validation):
    # Check SUMO has been set up properly
    

    # Set up logger
    logging.basicConfig(format='%(asctime)s %(message)s')
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # Start Simulation and step through
    if validation == 0:
            # Start Simulation and step through, --step-length param can be added if needed. --random ensures random scenarios
        #trafficScale = random.randint(1,2)
        sumoBinary = checkBinary("sumo")
        traci.start([sumoBinary, "-c", configFile,  "--collision.action", "warn", "--start","--quit-on-end","--seed", str(50), "--step-length",str(0.25),"--collision.mingap-factor",str(0),
                     ])
    elif validation == 1:
        sumoBinary = checkBinary("sumo-gui")
        traci.start([sumoBinary, "-c", configFile,  "--collision.action", "warn", "--start","--quit-on-end","--seed",str(200),"--log","logfile.txt","--step-length",str(0.25),"--collision.mingap-factor",str(0),
                    ])  #  '--lateral-resolution', '0.4'
    else:
        sumoBinary = checkBinary("sumo-gui")
        traci.start([sumoBinary, "-c", configFile,  "--collision.action", "warn", "--start","--quit-on-end","--random", "--step-length",str(0.25),"--collision.mingap-factor",str(0),
                     ])