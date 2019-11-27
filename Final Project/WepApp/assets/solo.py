import time
from dronekit import connect, VehicleMode, LocationLocal from pymavlink import mavutil
import math
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped from std_msgs.msg import String

# The Solo class is used to apply the dronekit package and send the command to drone
class Solo:
# Initial the vehicle object
    vehicle = None
# If the IsConnected == True, the class will skip the connect command # Otherwise, it will run the connect command first
IsConnected = False
    # initial the class value
def __init__(self):
__msgVehicleGlobalHomePosition = PointStamped()
        __msgVehicleGlobalHomePosition.header.stamp = 0
        __msgVehicleGlobalHomePosition.point.x = 0
        __msgVehicleGlobalHomePosition.point.y = 0
        __msgVehicleGlobalHomePosition.point.z = 0
        __msgVehicleLocalHomePosition = PointStamped()
        __msgVehicleLocalHomePosition.header.stamp = 0
        __msgVehicleLocalHomePosition.point.x = 0
        __msgVehicleLocalHomePosition.point.y = 0
        __msgVehicleLocalHomePosition.point.z = 0
"""
Function: IsArmed
Output: return the Boolean value """
def IsArmed(self):
return Solo.vehicle.armed
    """
    Getters and Setters
    """
def GetAltitude(self):
return Solo.vehicle.location.global_relative_frame.alt
def GetGlobalPosition(self):
        # vehicle GPS information x=lat, y=lon, z=alt
msgVehicleGPS = PointStamped()
msgVehicleGPS.header.stamp = rospy.Time.now()
msgVehicleGPS.point.x = Solo.vehicle.location.global_relative_frame.lat msgVehicleGPS.point.y = Solo.vehicle.location.global_relative_frame.lon msgVehicleGPS.point.z = Solo.vehicle.location.global_relative_frame.alt #rospy.loginfo(msgVehicleGPS)
return msgVehicleGPS
def GetLocalPosition(self):
# vehicle velocity information x=North, y=East, z=Down in m, z is negative altitude msgVehicleLocalLocation = PointStamped()
msgVehicleLocalLocation.header.stamp = rospy.Time.now() msgVehicleLocalLocation.point.x = Solo.vehicle.location.local_frame.north msgVehicleLocalLocation.point.y = Solo.vehicle.location.local_frame.east msgVehicleLocalLocation.point.z = Solo.vehicle.location.local_frame.down
# rospy.loginfo(msgvehicleLocalLocation)
return msgVehicleLocalLocation
def GetAttitude(self): msgVehicleAttitude = PointStamped()
 
msgVehicleAttitude.header.stamp = rospy.Time.now() msgVehicleAttitude.point.x = Solo.vehicle.attitude.roll msgVehicleAttitude.point.y = Solo.vehicle.attitude.pitch msgVehicleAttitude.point.z = Solo.vehicle.attitude.yaw # rospy.loginfo(msgvehicleAttitude)
return msgVehicleAttitude
def GetVelocity(self):
# vehicle velocity information x=North, y=East, z=Alt in m/s msgVehicleVolecity = PointStamped() msgVehicleVolecity.header.stamp = rospy.Time.now() msgVehicleVolecity.point.x = Solo.vehicle.velocity[0] msgVehicleVolecity.point.y = Solo.vehicle.velocity[1] msgVehicleVolecity.point.z = Solo.vehicle.velocity[2]
# rospy.loginfo(msgvehicleVolecity)
return msgVehicleVolecity
def GetGroundSpeed(self):
# vehicle groundspeed information x=groundspeed, y=0, z=0 in m/s msgVehicleGroundSpeed = PointStamped() msgVehicleGroundSpeed.header.stamp = rospy.Time.now() msgVehicleGroundSpeed.point.x = Solo.vehicle.groundspeed msgVehicleGroundSpeed.point.y = 0
msgVehicleGroundSpeed.point.z = 0
# rospy.loginfo(msgvehicleGroundspeed)
return msgVehicleGroundSpeed
def GetAirSpeed(self):
# vehicle velocity information x=airspeed, y=0, z=0 in m/s msgVehicleAirSpeed = PointStamped() msgVehicleAirSpeed.header.stamp = rospy.Time.now() msgVehicleAirSpeed.point.x = Solo.vehicle.airspeed msgVehicleAirSpeed.point.y = 0
msgVehicleAirSpeed.point.z = 0
# rospy.loginfo(msgvehicleAirspeed)
return msgVehicleAirSpeed
def SetGroundSpeed(self, speed):
ifspeed<=0 orspeed>10:
print("ground speed is out of allowed range") return
Solo.vehicle.groundspeed = speed
def __SetGlobalHomePosition(self, lat, lon, alt):
    self.__msgVehicleGlobalPosition.header.stamp = rospy.Time.now()
    self.__msgVehicleGlobalPosition.point.x = lat
    self.__msgVehicleGlobalPosition.point.y = lon
    self.__msgVehicleGlobalPosition.point.z = alt
def __SetLocalHomePosition(self, north, east, down):
    self.__msgVehicleLocalHomePosition.header.stamp = rospy.Time.now()
    self.__msgVehicleLocalHomePosition.point.x = north
    self.__msgVehicleLocalHomePosition.point.y = east
    self.__msgVehicleLocalHomePosition.poivehiclent.z = down
"""
Function: Connect
Input: rate, mode
Returns a Vehicle object connected to the address specified by string parameter ip. Set rate (Data stream refresh rate. The default is 4Hz (4 updates per second).) as 12. Choose the mode == 'Simulation', for simulation test
Choose the mode == 'Serial', for usb connect
Choose the mode == 'WIFI', for wifi test
"""
def Connect(self, rate = 12, mode = 'Simulation'):
if Solo.IsConnected:
print "The drone is already connected." return
if mode == 'Simulation':
connection_string = '127.0.0.1:14550' # For simulation test
elif mode == 'Serial':
connection_string = '/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00'

        # connection_string = '/dev/ttyACM0'
else:
connection_string = 'udpin:0.0.0.0:14550' # For simulation test
    # Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
Solo.vehicle = connect(connection_string, wait_ready = False, rate = rate) Solo.vehicle.mode = VehicleMode("GUIDED")
    Solo.IsConnected = True
"""
Function: Arm
Doing the pre-arm test and finish arm the drone """
def Arm(self):
print("Basic pre-arm checks")
# Don't try to arm until autopilot is ready while not Solo.vehicle.is_armable:
print(" Waiting for vehicle to initialize...") time.sleep(1)
print("Arming motors")
# Copter should arm in GUIDED mode
    Solo.vehicle.mode = VehicleMode("GUIDED")
    Solo.vehicle.armed = True
    # Confirm vehicle armed before attempting to take off
while not Solo.vehicle.armed: print(" Waiting for arming...") time.sleep(1)
print("Armed Successfully.")
"""
Function: TakeOff
TakeOff the drone to desired altitude 
"""
def TakeOff(self, aTargetAltitude):
    # Confirm vehicle armed before attempting to take off
if not Solo.vehicle.armed:
print("Error: Vehicle is not armed...") return
print("Taking off!")
Solo.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
def FlyToLocalPositionNed(self, north, east, down): self.goto_position_target_local_ned(north, east, down)
def SetVelocityNed(self, velocity_x, velocity_y, velocity_z): self.send_ned_velocity(velocity_x, velocity_y, velocity_z)
def SetYawDeg(self, heading, relative = False): # direction -1 ccw, 1 cw
if heading <180 and heading > 0:
self.condition_yaw(heading, relative = False, direction = 1)
else:
self.condition_yaw(heading, relative = False, direction = -1)
def SetYawWithDirectionDeg(self, heading, relative = False, direction = 1): # direction -1 ccw, 1 cw
self.condition_yaw(heading, relative = False, direction = 1)

def SetGimbalPitchDeg(self, pitch, roll, yaw):
# The solo gimbal can only control the pitch angle, the effective range is from 0 to -90 degrees
    Solo.vehicle.gimbal.rotate(pitch, roll, yaw)
def SetRoi(self,targetLocation): """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.
"""
    self.set_roi(targetLocation)
def goto_position_target_local_ned(self, north, east, down): """
Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified location in the North, East, Down frame.
"""
msg = Solo.vehicle.message_factory.set_position_target_local_ned_encode(
0, # time_boot_ms (not used)
0, 0, # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000111111111000, # type_mask (only positions enabled)
north, east, down,
0, 0, 0, # x, y, z velocity in m/s (not used)
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink) 0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    Solo.vehicle.send_mavlink(msg)
def send_ned_velocity(self, velocity_x, velocity_y, velocity_z): """
Move vehicle in direction based on specified velocity vectors.
"""
msg = Solo.vehicle.message_factory.set_position_target_local_ned_encode(
0, # time_boot_ms (not used)
0, 0, # target system, target component mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
0b0000111111000111, # type_mask (only speeds enabled)
0,0,0, # x, y, z positions (not used)
velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink) 0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    Solo.vehicle.send_mavlink(msg)
def set_roi(self,location):
# create the MAV_CMD_DO_SET_ROI command
msg = Solo.vehicle.message_factory.command_long_encode(
0, 0, # target system, target component mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command 0, #confirmation
0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    Solo.vehicle.send_mavlink(msg)
def condition_yaw(self, heading, relative = False, direction = 1): if relative:
is_relative=1 #yaw relative to direction of travel else:
is_relative=0 #yaw is an absolute angle
# create the CONDITION_YAW command using command_long_encode() msg = Solo.vehicle.message_factory.command_long_encode(
0, 0, # target system, target component mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command 0, #confirmation
heading,
0,
direction,
is_relative, # param 4, relative offset 1, absolute angle 0 0,0,0) #param5~7notused
# send command to vehicle
Solo.vehicle.send_mavlink(msg)
# param 1, yaw in degrees
# param 2, yaw speed deg/s
# param 3, direction -1 ccw, 1 cw

# ***************************************************************************************** # TargetTracking.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-
