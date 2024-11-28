import time
from pymavlink import mavutil

# Establish a connection to the MAVLink stream on the GCS computer.
# Replace '127.0.0.1:14550' with the correct IP and port if needed.
connection = mavutil.mavlink_connection('udp:127.0.0.1:14555')

# Wait for a heartbeat from the system to ensure connection is established
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received from system: (system %u component %u)" % 
      (connection.target_system, connection.target_component))

# Display heartbeat information if available
if 'HEARTBEAT' in connection.messages:
    print("Type: ", connection.messages['HEARTBEAT'].type)  # MAV_TYPE e.g., 1 for fixed wing
    print("Autopilot: ", connection.messages['HEARTBEAT'].autopilot)  # MAV_AUTOPILOT type
    print("Base mode: ", connection.messages['HEARTBEAT'].base_mode)  # Base mode flags
    print("Custom mode: ", connection.messages['HEARTBEAT'].custom_mode)  # Custom mode
    print("System status: ", connection.messages['HEARTBEAT'].system_status)  # System status
    print("MAVLink version: ", connection.messages['HEARTBEAT'].mavlink_version)  # MAVLink version

# Main loop to continuously receive and display MAVLink messages like position and attitude
while True:
    # Block until a message of type GLOBAL_POSITION_INT or ATTITUDE is received
    msg = connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True)

    if msg:
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7  # Convert from integer (degrees * 10^7) to float degrees
            lon = msg.lon / 1e7
            print(f"Latitude: {lat}, Longitude: {lon}")

        elif msg.get_type() == 'ATTITUDE':
            # Convert roll, pitch, and heading from radians to degrees
            roll = msg.roll * 180 / 3.14159
            pitch = msg.pitch * 180 / 3.14159
            hdg = msg.yaw * 180 / 3.14159
            print("Roll: %.2f, Pitch: %.2f, Heading: %.2f" % (roll, pitch, hdg))
            
    # Add a small delay to avoid overloading the console
    time.sleep(0.1)
