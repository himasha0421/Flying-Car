import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

#my work
class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        #contain the waypoints on the path
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.target_alititude = 3.0
        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if(self.flight_state == States.TAKEOFF):
            #check on the current alltitude and retain in takeoff state while altuide error become small
            drone_position = self.local_position
            if(np.abs(self.target_alititude - (-drone_position[2])) < 0.01):
                #start the transition throught waypoints
                self.waypoint_transition()
                #self.cmd_velocity(0,0,0,0)
        if(self.flight_state == States.WAYPOINT):
            #takecare of the waypoint transition
            target_xy = self.target_position[0:2]
            local_xy = self.local_position[0:2]
            print("local positions ", target_xy , local_xy)
            #take the euclidian distnace between two positions
            distance = np.abs(np.linalg.norm(target_xy-local_xy))
            
            #if diatnace less than a threshold then do waypoint transition
            if(distance <0.1):
                if(len(self.all_waypoints) >0 ):
                    self.waypoint_transition()
                else:
                    #then all the waypoints have reached and do the landing
                    #check on the velocity
                    vel_xyz = self.local_velocity[0:3]
                    if(np.linalg.norm(vel_xyz) < 0.5):
                        self.landing_transition()
        if(self.flight_state == States.DISARMING):
            #takeover the control onto manual states
            self.manual_transition()
            print("Drone flight sucessful !")


    def velocity_callback(self):
        if(self.flight_state == States.LANDING):
            local_vel = self.local_velocity[0:3]
            local_xyz = self.local_position[0:3]
            print("local velocity ",local_vel)
            print(self.global_home[0:2])
            if( (np.linalg.norm(local_xyz[0:2] - self.target_position[0:2])  < 0.1 ) and ( np.linalg.norm(local_xyz[2]-self.global_home[2]) < 0.01) ):
                print("Working")
                if(np.linalg.norm(local_vel) < 0.1):
                    #unarm the drone
                    self.disarming_transition()
                else:
                    #if the landing postion is ok then setup the velocity
                    self.cmd_velocity(0.0 , 0.0 , 0.0 ,0.0 )

    def state_callback(self):
        #initlially in maual state and need to start the motion with arming
        if not self.in_mission:
            return 
        if(self.flight_state == States.MANUAL):
            #arm the drone
            self.arming_transition()
        if(self.flight_state==States.ARMING):
            self.calculate_box()
            self.takeoff_transition()

    def calculate_box(self):
        #define the box waypoints
        self.all_waypoints=[
            [10.0 , 0.0 , 3.0],
            [10.0 , 10.0 , 3.0],
            [0.0 , 10.0  , 3.0],
            [0.0 , 0.0 , 3.0]
        ]


    def arming_transition(self):

        #take the control of the drone to autonomus mode
        print("arming transition")
        self.take_control()
        #arm the drone
        self.arm()
        #set the global position
        self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2])
        #set the state to current state
        self.flight_state= States.ARMING

    def takeoff_transition(self):

        print("takeoff transition")
        self.target_position[2] = self.target_alititude 
        self.takeoff(self.target_alititude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):

        print("waypoint transition")
        #take the transition target position
        target_waypoint = self.all_waypoints.pop(0)
        #take north east altitude heading
        north, east = target_waypoint[0] , target_waypoint[1]
        altitude , heading = target_waypoint[2] , 0.0 
        print(north , east , altitude ,heading)
        self.cmd_position(north , east , altitude , heading)
        #set the target position for after processing
        self.target_position = np.array([north , east , altitude])
        self.flight_state = States.WAYPOINT


    def landing_transition(self):
        self.land()
        #change the state
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        #switch to manual mode
        self.release_control()
        #stop the connection
        self.connection.stop()
        self.in_mission = False 
        self.flight_state=States.MANUAL

    def start(self):

        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
