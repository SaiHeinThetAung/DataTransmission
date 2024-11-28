from pymavlink import mavutil

# Constants for PWM range
PWM_MIN = 1000  # Minimum PWM value
PWM_MAX = 2000  # Maximum PWM value

def pwm_to_percent(pwm):
    """
    Convert PWM value to percentage.
    Args:
        pwm (int): PWM value.
    Returns:
        float: Percentage value (0-100%).
    """
    if pwm < PWM_MIN:
        return 0.0
    elif pwm > PWM_MAX:
        return 100.0
    return ((pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)) * 100.0

# Connect to the SITL simulation via MAVLink
connection = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

# Wait for a heartbeat to confirm connection
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Connected to the vehicle!")

def get_servo_outputs():
    while True:
        # Receive the SERVO_OUTPUT_RAW message
        msg = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if not msg:
            continue

        # Extract PWM values for the desired channels
        ch3out = msg.servo3_raw
        ch9out = msg.servo9_raw
        ch10out = msg.servo10_raw
        ch11out = msg.servo11_raw
        ch12out = msg.servo12_raw

        # Calculate percentage for CH3
        ch3percent = pwm_to_percent(ch3out)

        # Print the outputs
        print(f"CH3OUT: {ch3out}")
        print(f"CH3Percent: {ch3percent:.2f}%")
        print(f"CH9OUT: {ch9out}, CH10OUT: {ch10out}, CH11OUT: {ch11out}, CH12OUT: {ch12out}")
        print("-------------------------------------------------------")

try:
    get_servo_outputs()
except KeyboardInterrupt:
    print("Exiting...")
