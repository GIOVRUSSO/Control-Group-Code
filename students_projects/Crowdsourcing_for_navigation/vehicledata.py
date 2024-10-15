# VehicleData class to gather all useful data related to a vehicle spawned in the simulation

class VehicleData:
    
    def __init__(self,id,alg,depart,dest='',route=None):
        self.id = id # id of the vehicle
        self.alg = alg # id of the assigned route
        self.speeds = [] # array of speeds for keeping track of the vehicle's speeds
        self.selected_id = 0 # id of the selected behaviour (useful for agents)
        self.dist = 0 # travelled distance
        self.dest = dest # destination edge
        self.depart = int(depart) # integer with the instant of departure
        self.fuelconsumption = [] # array of fuel consumptions for keeping track of the vehicle's fuel use
        self.waitingtime = [0] # array of waiting times for keeping track of the vehicle's waiting time situations
        self.noiseemission = [] # array of noise emissions for keeping track of the vehicle's noise emissions
        self.co2emission = [] # array of CO2 emissions for keeping track of the vehicle's emissions
        self.coemission = [] # array of CO emissions for keeping track of the vehicle's emissions
        self.noxemission = [] # array of NOx emissions for keeping track of the vehicle's emissions
        self.pmxemission = [] # array of PMx emissions for keeping track of the vehicle's emissions
        self.hcemission = [] # array of HC emissions for keeping track of the vehicle's emissions
        self.traveltime = 0 # travel time
        self.influence = 0.5 # vehicle influence in signalling wip areas
        self.currentroad = '' # non-crossing edge on which the vehicle is
        self.weightened = {} # list of edges weightened by a nearby wip area
        self.arrived = False # flag to keep track of whether the vehicle has reached its destination or not
        self.route = route # list of edges the vehicle is initially supposed to follow