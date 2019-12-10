import argparse
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

# Created by Yamnel S.

parser = argparse.ArgumentParser()
parser.add_argument("lat", help="Latitude where the drone will be sent to.")
parser.add_argument("lon", help="Longitude where the drone will be sent to.")

# still in development
parser.add_argument("--connect", help="Vehicle connection target. If not specified will be the SITL.")

args = parser.parse_args()

connection_string = args.connect
a_location = [args.lat, args.lon] # Destination Location


class Drone(object):
    def __init__(self, vehicle):
        self.gps_lock = False
        self.altitude = 30.0 # safe altitude

        # Connect to the Vehicle
        self._log('Connected to vehicle.')
        self.vehicle = vehicle
        self.commands = self.vehicle.commands
        self.current_coords = []

        self._log("******  Let's begin the delivery  ******")

        # Register observers
        self.vehicle.add_attribute_listener('location', self.location_callback)

    def launch(self):
        self._log("Waiting for location...")
        while self.vehicle.location.global_frame.lat == 0:
            time.sleep(0.1)
        self.home_coords = [self.vehicle.location.global_frame.lat,
                            self.vehicle.location.global_frame.lon]

        self._log("Waiting for ability to arm...")
        while not self.vehicle.is_armable:
            time.sleep(.1)

        self._log('Running initial boot sequence')
        self.change_mode('GUIDED')
        self.arm()
        self.takeoff()

    def takeoff(self):
        aTargetAltitude = 30 #  self.altitude
        self._log("Taking off")
        self.vehicle.simple_takeoff(aTargetAltitude)


        # Wait until the vehicle reaches a safe height before
        # processing the goto
        # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)


    def arm(self, value=True):
        if value:
            self._log('Waiting for arming...')
            self.vehicle.armed = True
            while not self.vehicle.armed:
                time.sleep(.1)
        else:
            self._log("Disarming!")
            self.vehicle.armed = False

    def change_mode(self, mode):
        self._log("Changing to mode: {0}".format(mode))

        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            self._log('  ... polled mode: {0}'.format(mode))
            time.sleep(1)

    # to change to mode RTL
    def return_home(self):
        self._log("Returning Home...")

        self.vehicle.mode = VehicleMode('RTL')

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt, " Location: ", self.vehicle.location.global_relative_frame.lat, ", ", self.vehicle.location.global_relative_frame.lon)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt <= 0.5: # .5 and not 0 to account for sensor error.
                print("Reached home")
                break
            time.sleep(1)


    def goto(self, location, relative=None):
        wait = 30 #seconds
        self._log("Goto: {0}, {1}".format(location, self.altitude))

        # Altitude is relative to Home position
        if relative:
            self.vehicle.simple_goto(
                LocationGlobalRelative(
                    float(location[0][:-9]), float(location[1][:-9]),
                    float(self.altitude)
                )
            )
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt, " Location: ",
                  self.vehicle.location.global_relative_frame.lat, ", ",
                  self.vehicle.location.global_relative_frame.lon)

            time.sleep(wait) #give it time to fy

            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt, " Location: ",
                  self.vehicle.location.global_relative_frame.lat, ", ",
                  self.vehicle.location.global_relative_frame.lon)
        else:
            self.vehicle.simple_goto(
                LocationGlobal(
                    float(location[0][:-9]), float(location[1][:-9]),
                    float(self.altitude)
                )
            )
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt, " Location: ",
                  self.vehicle.location.global_relative_frame.lat, ", ",
                  self.vehicle.location.global_relative_frame.lon)

            time.sleep(wait) #give it time to fy

            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt, " Location: ",
                  self.vehicle.location.global_relative_frame.lat, ", ",
                  self.vehicle.location.global_relative_frame.lon)

        # time.sleep(20)

    def get_location(self):
        return [self.current_location.lat, self.current_location.lon]


    def location_callback(self, vehicle, name, location):
        if location.global_relative_frame.alt is not None:
            self.altitude = location.global_relative_frame.alt

        self.current_location = location.global_relative_frame

    def _log(self, message):
        print("[DEBUG]: {0}".format(message))

    def flush(self):
        self.vehicle.flush() # closes it all
        
          
 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    #vehicle.flush()
    
    
#-- Key event function
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")	
		    
    else: #-- non standard keys
        if event.keysym == 'W':
			#print("UP pressed >> moving forward")
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'S':
			#print("DOWN pressed >> moving backward")
            set_velocity_body(vehicle,-gnd_speed, 0, 0)
        elif event.keysym == 'A':
			#print("LEFT pressed >> moving left")
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'D':
			#print("RIGHT pressed >> moving right")
            set_velocity_body(vehicle, 0, gnd_speed, 0)
        elif event.keysym == 'I':
			#print("UP pressed >> moving upward")
            set_velocity_body(vehicle, 0, 0, gnd_speed)
        elif event.keysym == 'J':
			#print("DOWN pressed >> moving upward")
            set_velocity_body(vehicle, 0, 0, -gnd_speed)
    
#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)
 
#- Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()


def main():
    print('Connecting to Solo ...')
    print('Connecting to vehicle on: %s' % connection_string)

    vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True) # for solo drone

    print('Launching Drone...')
    drone = Drone(vehicle)
    drone.launch()
    time.sleep(3)
    drone.goto(a_location, True)
    print("Got to location: {0}".format(a_location))

    drone.return_home()
    drone.flush()
    time.sleep(5) # giving time for disarming to take place


    return True # might be needed for future use

if __name__ == '__main__':
    main()


