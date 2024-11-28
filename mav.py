import time
from pymavlink import mavutil

connection = mavutil.mavlink_connection('udp:0.0.0.0:14553')

connection.wait_heartbeat()
print("Heart beat from system: (system %u component %u)" % 
      (connection.target_system, connection.target_component))

# Display heart beat
print("type: ", connection.messages['HEARTBEAT'].type) # MAV_TYPE 1:Fixed wing aircraft.
print("autopilot: ",connection.messages['HEARTBEAT'].autopilot) # MAV_AUTOPILOT 3 MAV_AUTOPILOT_ARDUPILOTMEGA-ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org
print("base_mode: ",connection.messages['HEARTBEAT'].base_mode) # MAV_MODE_FLAG 81
print("custom_mode: ",connection.messages['HEARTBEAT'].custom_mode) #
print("system_status: ",connection.messages['HEARTBEAT'].system_status) #MAV_STATE 3 MAV_STATE_STANDBY-System is grounded and on standby. It can be launched any time.
print("mavlink_version: ",connection.messages['HEARTBEAT'].mavlink_version)


# Main Loop to get some message such as GLOBAL_POSITION_INT and ATTITUDE
while True:
    msg = connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True)

    if msg:
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat/1e7
            lon = msg.lon/1e7
            # print("lat %.8f long %.8f" %(lat, lon))
            print(f"lat: {lat}, lon: {lon}")

        elif msg.get_type() == 'ATTITUDE':
            roll = msg.roll/3.1415*180
            pitch = msg.pitch/3.1415*180
            hdg = msg.yaw/3.1415*180
            print("roll %.2f, pitch %.2f, heading %.2f \n"%(roll, pitch, hdg))
            