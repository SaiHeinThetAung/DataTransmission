from pymavlink import mavutil
import time

# Connect to the drone or simulation
connection = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Connected to the vehicle!")

# Define global variables for TOH and TOT
toh = None
tot = None

# Placeholder for waypoint IDs
home_waypoint_id = None
target_waypoint_id = None

# Function to request waypoints
def fetch_waypoints():
    print("Requesting waypoints from the drone...")
    connection.mav.mission_request_list_send(connection.target_system, connection.target_component)
    waypoints = []

    while True:
        msg = connection.recv_match(type=['MISSION_ITEM_INT', 'MISSION_COUNT'], blocking=True)
        if msg.get_type() == 'MISSION_COUNT':
            print(f"Total waypoints: {msg.count}")
            continue
        elif msg.get_type() == 'MISSION_ITEM_INT':
            waypoints.append(msg)
            print(f"Waypoint {msg.seq} received: Lat={msg.x / 1e7}, Lon={msg.y / 1e7}")
            if len(waypoints) == msg.seq + 1:  # All waypoints received
                break

    return waypoints

# Function to monitor current waypoint and calculate TOH and TOT
def monitor_waypoints(home_id, target_id):
    global toh, tot

    while True:
        msg = connection.recv_match(blocking=True)
        if not msg:
            continue

        # Monitor MISSION_CURRENT for active waypoint
        if msg.get_type() == 'MISSION_CURRENT':
            current_wp = msg.seq
            current_time = time.time()

            if current_wp == home_id and toh is None:
                toh = current_time
                print(f"Time Over Home (TOH): {toh}")

            elif current_wp == target_id and tot is None:
                tot = current_time
                print(f"Time Over Target (TOT): {tot}")

        # Exit if both TOH and TOT are logged
        if toh is not None and tot is not None:
            print("TOH and TOT have been logged.")
            break

# Fetch waypoints
waypoints = fetch_waypoints()

# Identify Home and Target waypoints based on known information
# Modify this section to match the expected waypoints for your mission
for wp in waypoints:
    if wp.seq == 0:  # Assume Home is the first waypoint
        home_waypoint_id = wp.seq
    elif wp.seq == 5:  # Assume Target is the 6th waypoint
        target_waypoint_id = wp.seq

print(f"Home Waypoint ID: {home_waypoint_id}")
print(f"Target Waypoint ID: {target_waypoint_id}")

# Monitor waypoints to calculate TOH and TOT
if home_waypoint_id is not None and target_waypoint_id is not None:
    monitor_waypoints(home_waypoint_id, target_waypoint_id)
else:
    print("Home or Target waypoint not found. Check your mission plan.")

# Final output
if toh is not None:
    print(f"Final TOH: {toh}")
else:
    print("Final TOH: None")

if tot is not None:
    print(f"Final TOT: {tot}")
else:
    print("Final TOT: None")
