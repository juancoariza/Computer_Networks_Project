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
		
		self.local_location_north = 0
		self.local_location_east = 0
		self.local_location_down = 0
		self.altitude_yaw = 0

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
		# changed by user input on HTML controller
		self.change_mode('GUIDED')
		# ////////////////////////////////////////
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
			
	def manual_flight(self):
		self.vehicle.mode = VehicleMode('AltHold')
		
		# collects the keyboard commands from user
		inputKey = raw_input("Enter a control key then press enter")
		
		if inputKey == 'w':
			self.local_location_down += 0.2
			set_velocity_body(vehicle, local_location_down, 0, 0)
		elif inputKey == 's':
			self.local_location_down -= 0.2
			set_velocity_body(vehicle, local_location_down, 0, 0)
		elif inputKey == 'a':
			self.altitude_yaw -= 0.174533
			if self.altitude_yaw > 6.28:
				self.altitude_yaw = self.altitude_yaw - 6.28
			if self.altitude_yaw < 0:
				self.altitude_yaw = self.altitude_yaw + 6.28s
		elif inputKey == 'd':
			self.altitude_yaw += 0.174533
			if self.altitude_yaw > 6.28:
				self.altitude_yaw = self.altitude_yaw - 6.28
			if self.altitude_yaw < 0:
				self.altitude_yaw = self.altitude_yaw + 6.28
		elif inputKey = 'i':
			self.local_location_north += 0.3
		elif inputKey = 'k':
			self.local_location_north -= 0.3
		elif inputKey = 'j':
			self.local_location_east -= 0.3
		elif inputKey = 'l':
			self.local_location_east += 0.3
			
		else: 
			continue
			
	def set_velocity_body(vehicle, vx, vy, vz):

		# Standard MAVLink 
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,
			0, 0,
			mavutil.mavlink.MAV_FRAME_BODY_NED,
			0b0000111111000111, #-- Used to bitmask the veolcity inputs 
			0, 0, 0,			# position setupts
			vx, vy, vz,			# velocity setups (from user)
			0, 0, 0,			# acceleration setups (automatic)
			0, 0)
		vehicle.send_mavlink(msg)
		vehicle.flush()

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
