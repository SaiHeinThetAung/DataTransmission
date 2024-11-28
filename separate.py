import asyncio
import websockets
import json
import gzip
from pymavlink import mavutil
import math
import time

# Connect to Ground Control Station (GCS)
# connection = mavutil.mavlink_connection('tcp:0.0.0.0:14555') # udp
connection = mavutil.mavlink_connection('tcp:127.0.0.1:14550') # self
# connection = mavutil.mavlink_connection('tcp:172.24.160.106:14550') # park
# connection = mavutil.mavlink_connection('tcp:172.24.253.192:14550') # toe
# connection = mavutil.mavlink_connection('tcp:172.24.74.230:14550') # hein
# connection = mavutil.mavlink_connection('tcp:172.24.54.249:14550') # htet

connection.wait_heartbeat()
print("Heartbeat received")

# Home location (set manually or fetched from the autopilot system)
home_location = {'lat': 16.7745, 'lon': 96.1552}  # Example: Yangon, Myanmar

start_time = time.time()  # To calculate the time in air
# Store previous latitude and longitude for distance calculation
prev_lat = None
prev_lon = None
total_distance = 0  # Total distance traveled
# Cache for storing waypoints to avoid fetching repeatedly
waypoints_cache = None

# Dictionary to store telemetry data
telemetry_data = {
    'lat': None,                   # Latitude in decimal degrees (WGS84)
    'lon': None,                   # Longitude in decimal degrees (WGS84)
    'alt': None,                   # Altitude above mean sea level in meters
    'dist_traveled': 0,            # Total distance traveled since the start of the mission in meters
    'wp_dist': None,               # Distance to the next waypoint in meters
    'dist_to_home': None,          # Distance to the home location in meters
    'vertical_speed': None,        # Vertical speed in meters per second (climbing or descending rate)
    'wind_vel': None,              # Wind velocity in meters per second
    'airspeed': None,              # Current airspeed in meters per second (relative to air)
    'groundspeed': None,           # Current groundspeed in meters per second (relative to the ground)
    'roll': None,                  # Roll angle in degrees (tilt around the longitudinal axis)
    'pitch': None,                 # Pitch angle in degrees (tilt around the lateral axis)
    'yaw': None,                   # Yaw angle in degrees (rotation around the vertical axis, heading)
    'toh': None,                   # Estimated time to return to home location in seconds
    'tot': None,                   # Estimated time to reach the next waypoint in seconds
    'time_in_air': None,           # Total flight time in seconds
    'time_in_air_min_sec': None,   # Total flight time in minutes and fractional seconds (e.g., 2.45 for 2 minutes 27 seconds)
    'gps_hdop': None,              # GPS Horizontal Dilution of Precision (lower is better; typically < 2 is good)
    'battery_voltage': None,       # Current battery voltage in volts
    'battery_current': None,       # Current battery consumption in amperes
    'ch3percent': None,            # Throttle percentage output on channel 3
    'ch3out': None,                # PWM output value for channel 3 (typically used for throttle)
    'ch9out': None,                # PWM output value for channel 9
    'ch10out': None,               # PWM output value for channel 10
    'ch11out': None,               # PWM output value for channel 11
    'ch12out': None,               # PWM output value for channel 12
    'waypoints': [],               # List of mission waypoints with metadata (e.g., latitude, longitude, altitude)
}


# Function to calculate the distance between two GPS coordinates
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# Function to classify update rate based on adverse weather conditions
def classify_update_rate(telemetry_data):
    """
    Adjusts medium-frequency update rate based on weather conditions.
    Returns the update rate (in seconds).
    """
    wind_vel = telemetry_data.get('wind_vel', 0)
    vertical_speed = telemetry_data.get('vertical_speed', 0)
    roll = telemetry_data.get('roll', 0)
    pitch = telemetry_data.get('pitch', 0)
    yaw = telemetry_data.get('yaw', 0)
    gps_hdop = telemetry_data.get('gps_hdop', 0)
    groundspeed = telemetry_data.get('groundspeed', 0)
    airspeed = telemetry_data.get('airspeed', 0)
    speed_discrepancy = abs(groundspeed - airspeed)

    if wind_vel < 5 and abs(vertical_speed) <= 2 and abs(roll) <= 15 and abs(pitch) <= 15 and abs(yaw) <= 20 and gps_hdop <= 1.5 and speed_discrepancy < 5:
        return 0.5  # 500ms - Normal Conditions
    if 5 <= wind_vel <= 10 or 2 < abs(vertical_speed) <= 4 or 15 < abs(roll) <= 20 or 15 < abs(pitch) <= 20 or 20 < abs(yaw) <= 30 or 1.5 < gps_hdop <= 3.0 or 5 <= speed_discrepancy <= 10:
        return 0.25  # 250ms - Moderate Adverse Conditions
    if 10 < wind_vel <= 15 or 4 < abs(vertical_speed) <= 6 or 20 < abs(roll) <= 25 or 20 < abs(pitch) <= 25 or 30 < abs(yaw) <= 35 or 3.0 < gps_hdop <= 5.0 or 10 < speed_discrepancy <= 15:
        return 0.1  # 100ms - Severe Adverse Conditions
    if wind_vel > 15 or abs(vertical_speed) > 6 or abs(roll) > 25 or abs(pitch) > 25 or abs(yaw) > 35 or gps_hdop > 5.0 or speed_discrepancy > 15:
        return 0.05  # 50ms - Critical Conditions
    return 0.5  # Default

# Function to fetch waypoints
async def fetch_waypoints():
    global waypoints_cache
    if waypoints_cache is not None:
        return waypoints_cache  # Return cached waypoints if available

    print("Requesting waypoint list...")
    connection.mav.mission_request_list_send(connection.target_system, connection.target_component)
    msg = connection.recv_match(type='MISSION_COUNT', blocking=True, timeout=10)
    
    if not msg:
        print("No waypoints found or timeout occurred.")
        return []

    waypoint_count = msg.count
    print(f"Number of waypoints: {waypoint_count}")

    waypoints = []
    for seq in range(waypoint_count):
        connection.mav.mission_request_int_send(connection.target_system, connection.target_component, seq)
        msg = connection.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if msg:
            waypoint = {
                "seq": msg.seq,
                "frame": msg.frame,
                "command": msg.command,
                "current": msg.current,
                "autocontinue": msg.autocontinue,
                "param1": msg.param1,
                "param2": msg.param2,
                "param3": msg.param3,
                "param4": msg.param4,
                "x": msg.x / 1e7,  # Latitude in degrees
                "y": msg.y / 1e7,  # Longitude in degrees
                "z": msg.z,        # Altitude in meters
            }
            waypoints.append(waypoint)
            print(f"Received waypoint {waypoint['seq']}: {waypoint}")
        else:
            print(f"Failed to fetch waypoint {seq}")

    waypoints_cache = waypoints  # Cache the fetched waypoints
    return waypoints_cache

# High-frequency task (lat, lon, alt, etc.)
async def send_high_frequency_data(websocket):
    try:
        while True:
        # Receive telemetry data from the GCS connection
            msg = connection.recv_match(
                type=[
                    'GLOBAL_POSITION_INT', 'VFR_HUD',
                ],
                blocking=True, timeout=1
            )
            if not msg:
                await asyncio.sleep(0.1)  # Sleep if no message received
                continue

            # Process the 'GLOBAL_POSITION_INT' message
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                telemetry_data['lat'] = msg.lat / 1e7  # Convert to decimal degrees
                telemetry_data['lon'] = msg.lon / 1e7  # Convert to decimal degrees
                telemetry_data['alt'] = msg.alt / 1000.0  # Convert to meters
                telemetry_data['vertical_speed'] = msg.vz / 100.0  # Vertical speed in m/s

            # Process 'VFR_HUD' message for airspeed and groundspeed
            elif msg.get_type() == 'VFR_HUD':
                telemetry_data['airspeed'] = msg.airspeed  # Airspeed in m/s
                telemetry_data['groundspeed'] = msg.groundspeed  # Groundspeed in m/s

            # Prepare data for sending to WebSocket client
            high_frequency_data = {
                'lat': telemetry_data['lat'],
                'lon': telemetry_data['lon'],
                'alt': telemetry_data['alt'],
                'vertical_speed': telemetry_data['vertical_speed'],
                'airspeed': telemetry_data['airspeed'],
                'groundspeed': telemetry_data['groundspeed']
            }

            # Send telemetry data to WebSocket client
            print(f"Sending High Frequency Telemetry Data: {high_frequency_data}")
            # Convert the telemetry data to JSON string
            json_data = json.dumps(high_frequency_data)
            # Compress the JSON string with Gzip
            compressed_data = gzip.compress(json_data.encode('utf-8'))

            await websocket.send(compressed_data)
            await asyncio.sleep(0.05)  # High-frequency: 50ms
    except websockets.exceptions.ConnectionClosedError:
        print("WebSocket connection closed.")
        
    except Exception as e:
        print(f"Error in telemetry server: {e}")

# Medium-frequency task with dynamic update rate
async def send_medium_frequency_data(websocket):
    try:
        while True:
            # Receive telemetry data from the GCS connection
            msg = connection.recv_match(
                type=[
                    'GLOBAL_POSITION_INT', 'ATTITUDE', 'VFR_HUD', 'SYS_STATUS', 'GPS_RAW_INT', 'WIND'
                ],
                blocking=True, timeout=1
            )
            if not msg:
                await asyncio.sleep(0.1)  # Sleep if no message received
                continue

            # Process 'GLOBAL_POSITION_INT' message for distance and location info
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                # Calculate distance to home and distance traveled
                telemetry_data['dist_to_home'] = calculate_distance(
                    telemetry_data['lat'], telemetry_data['lon'], home_location['lat'], home_location['lon']
                )

                if prev_lat and prev_lon:
                    distance = calculate_distance(prev_lat, prev_lon, telemetry_data['lat'], telemetry_data['lon'])
                    total_distance += distance
                    telemetry_data['dist_traveled'] = round(total_distance, 2)
                prev_lat, prev_lon = telemetry_data['lat'], telemetry_data['lon']

                if telemetry_data['waypoints']:
                    next_wp = telemetry_data['waypoints'][0]
                    telemetry_data['wp_dist'] = calculate_distance(
                        telemetry_data['lat'], telemetry_data['lon'], next_wp['x'], next_wp['y']
                    )
                else:
                    telemetry_data['wp_dist'] = None

            # Process 'ATTITUDE' message for roll, pitch, and yaw angles
            elif msg.get_type() == 'ATTITUDE':
                telemetry_data['roll'] = round(msg.roll * (180 / math.pi), 2)  # Convert from radians to degrees
                telemetry_data['pitch'] = round(msg.pitch * (180 / math.pi), 2)  # Convert from radians to degrees
                telemetry_data['yaw'] = round(msg.yaw * (180 / math.pi), 2)  # Convert from radians to degrees

            # Calculate 'toh' (time over home) and 'tot' (time over target)
            if telemetry_data['dist_to_home'] and telemetry_data['groundspeed']:
                telemetry_data['toh'] = round(telemetry_data['dist_to_home'] / telemetry_data['groundspeed'], 2)
            else:
                telemetry_data['toh'] = None

            if telemetry_data['wp_dist'] and telemetry_data['groundspeed']:
                telemetry_data['tot'] = round(telemetry_data['wp_dist'] / telemetry_data['groundspeed'], 2)
            else:
                telemetry_data['tot'] = None

            # Calculate time in air
            telemetry_data['time_in_air'] = round(time.time() - start_time, 2)
            telemetry_data['time_in_air_min_sec'] = round(
                telemetry_data['time_in_air'] // 60 + (telemetry_data['time_in_air'] % 60) / 100, 2
            )

            # Prepare the medium-frequency data
            medium_frequency_data = {
                'dist_traveled': telemetry_data['dist_traveled'],
                'wp_dist': telemetry_data['wp_dist'],
                'dist_to_home': telemetry_data['dist_to_home'],
                'roll': telemetry_data['roll'],
                'pitch': telemetry_data['pitch'],
                'yaw': telemetry_data['yaw'],
                'toh': telemetry_data['toh'],
                'tot': telemetry_data['tot'],
                'time_in_air': telemetry_data['time_in_air'],
                'time_in_air_min_sec': telemetry_data['time_in_air_min_sec']
            }
            # Send telemetry data to WebSocket client
            print(f"Sending Medium Telemetry Data: {medium_frequency_data}")
            # Convert the telemetry data to JSON string
            json_data = json.dumps(medium_frequency_data)
            # Compress the JSON string with Gzip
            compressed_data = gzip.compress(json_data.encode('utf-8'))

            await websocket.send(compressed_data)
            print(f"Update rate : {classify_update_rate(telemetry_data)} ms")

            # Sleep to control the update rate
            await asyncio.sleep(classify_update_rate)  # Convert ms to seconds

    except websockets.exceptions.ConnectionClosedError:
        print("WebSocket connection closed.")
        
    except Exception as e:
        print(f"Error in telemetry server: {e}")

# Low-Frequency data task (waypoints, etc.)
async def send_low_frequency_data(websocket):
    try:
        while True:
            # Receive telemetry data from the GCS connection
            msg = connection.recv_match(
                type=['SYS_STATUS', 'GPS_RAW_INT', 'SERVO_OUTPUT_RAW'],
                blocking=True, timeout=1
            )

            if not msg:
                await asyncio.sleep(0.1)  # Sleep if no message received
                continue

            # Process 'SYS_STATUS' message for battery voltage and current
            if msg.get_type() == 'SYS_STATUS':
                telemetry_data['battery_voltage'] = (
                    msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
                )
                telemetry_data['battery_current'] = (
                    msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                )  # Default to 0.0 if current_battery is -1

            # Process 'GPS_RAW_INT' message for GPS Horizontal Dilution of Precision (HDOP)
            elif msg.get_type() == 'GPS_RAW_INT':
                telemetry_data['gps_hdop'] = msg.eph / 100.0  # Convert from cm to meters

            # Process 'SERVO_OUTPUT_RAW' message for channel output data
            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                telemetry_data['ch3out'] = msg.servo3_raw if hasattr(msg, 'servo3_raw') else None
                telemetry_data['ch9out'] = msg.servo9_raw if hasattr(msg, 'servo9_raw') else None
                telemetry_data['ch10out'] = msg.servo10_raw if hasattr(msg, 'servo10_raw') else None
                telemetry_data['ch11out'] = msg.servo11_raw if hasattr(msg, 'servo11_raw') else None
                telemetry_data['ch12out'] = msg.servo12_raw if hasattr(msg, 'servo12_raw') else None
                
                # Calculate throttle percentage (assuming servo3 is throttle)
                if telemetry_data['ch3out'] is not None:
                    telemetry_data['ch3percent'] = round(((telemetry_data['ch3out'] - 1000) / 1000) * 100, 2)

            # Prepare the one-time-frequency data
            low_frequency_data = {
                'gps_hdop': telemetry_data['gps_hdop'],
                'battery_voltage': telemetry_data['battery_voltage'],
                'battery_current': telemetry_data['battery_current'],
                'ch3percent': telemetry_data['ch3percent'],
                'ch3out': telemetry_data['ch3out'],
                'ch9out': telemetry_data['ch9out'],
                'ch10out': telemetry_data['ch10out'],
                'ch11out': telemetry_data['ch11out'],
                'ch12out': telemetry_data['ch12out']
            }

            # Send telemetry data to WebSocket client
            print(f"Sending Low Frequency Telemetry Data: {low_frequency_data}")
            # Convert the telemetry data to JSON string
            json_data = json.dumps(low_frequency_data)
            # Compress the JSON string with Gzip
            compressed_data = gzip.compress(json_data.encode('utf-8'))

            await websocket.send(compressed_data)

            # Sleep briefly before checking for new messages
            await asyncio.sleep(5)  # Adjust sleep time as needed

    except websockets.exceptions.ConnectionClosedError:
        print("WebSocket connection closed.")
        
    except Exception as e:
        print(f"Error in telemetry server: {e}")

# Send one-time data (initial fetch of waypoints)
async def send_one_time_data(websocket):
    waypoints = await fetch_waypoints()
    telemetry_data['waypoints'] = waypoints
    one_time_data = {"waypoints": waypoints}

    json_data = json.dumps(one_time_data)
    compressed_data = gzip.compress(json_data.encode('utf-8'))

    await websocket.send(compressed_data)


# Monitor waypoint changes and send updates
async def monitor_waypoint_changes(websocket):
    global waypoints_cache
    while True:
        new_waypoints = await fetch_waypoints()

        # Compare current waypoints with cached waypoints
        if new_waypoints != waypoints_cache:  # Send only if there is a change
            print("Waypoints have changed. Sending updated waypoints to WebSocket.")
            telemetry_data['waypoints'] = new_waypoints
            waypoints_cache = new_waypoints

            updated_data = {"waypoints": new_waypoints}
            json_data = json.dumps(updated_data)
            compressed_data = gzip.compress(json_data.encode('utf-8'))

            await websocket.send(compressed_data)
        else:
            print("Waypoints have not changed. No update sent.")

        await asyncio.sleep(5)  # Wait before checking again

# Telemetry Server
async def telemetry_server(websocket):
    # Send one-time data initially
    await send_one_time_data(websocket)

    # Start monitoring for waypoint changes
    await monitor_waypoint_changes(websocket)

    # Start high- and medium-frequency updates
    await asyncio.gather(
        send_high_frequency_data(websocket),
        # send_medium_frequency_data(websocket),
        send_low_frequency_data(websocket)
    )

# Main Server
async def main():
    async with websockets.serve(telemetry_server, "0.0.0.0", 6789):
        print("WebSocket server started on ws://0.0.0.0:6789")
        await asyncio.Future()  # Run indefinitely

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server shut down gracefully.")