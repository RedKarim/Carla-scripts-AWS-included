import paho.mqtt.client as mqtt
import json
import time
import math

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
CLIENT_ID = "virtual_controller"

# Car following parameters
TARGET_DISTANCE = 5.0  # Reduced target distance for closer following
Kp_distance = 2.0  # Increased proportional gain for more responsive distance control
Kp_steer = 0.5    # Reduced steering gain for smoother turns
MAX_SPEED = 30.0  # Increased maximum speed
MIN_SPEED = 5.0   # Increased minimum speed
ACCELERATION_GAIN = 1.0  # Increased acceleration gain
STEERING_GAIN = 0.3  # Reduced steering gain

# Vehicle state storage
lead_vehicle_data = None
follow_vehicle_data = None

def calculate_control(lead_data, follow_data):
    """Calculate control commands based on lead and follow vehicle data"""
    try:
        if lead_data is None or follow_data is None:
            print("Missing vehicle data, cannot calculate control")
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
                "timestamp": time.time()
            }

        # Extract positions
        lead_pos = lead_data['transform']['location']
        follow_pos = follow_data['transform']['location']
        
        # Calculate distance
        dx = lead_pos['x'] - follow_pos['x']
        dy = lead_pos['y'] - follow_pos['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle to lead vehicle (in radians)
        angle = math.atan2(dy, dx)
        
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
            
        # Calculate desired speed based on distance error
        distance_error = distance - TARGET_DISTANCE
        target_speed = max(MIN_SPEED, min(MAX_SPEED, abs(distance_error) * Kp_distance))
        
        # Calculate steering based on angle (normalized to [-1, 1])
        steer = max(-1.0, min(1.0, angle * Kp_steer))
        
        # Calculate throttle and brake based on speed error
        current_speed = follow_data['velocity']['speed']
        speed_error = target_speed - current_speed
        
        # More aggressive acceleration control
        if speed_error > 0.5:  # Need acceleration
            throttle = min(1.0, speed_error * ACCELERATION_GAIN)
            brake = 0.0
        elif speed_error < -0.5:  # Need deceleration
            throttle = 0.0
            brake = min(1.0, -speed_error * ACCELERATION_GAIN)
        else:  # Fine-tune speed
            throttle = 0.3  # Maintain some throttle for better acceleration
            brake = 0.0
        
        # Add steering correction based on current yaw
        current_yaw = follow_data['transform']['rotation']['yaw']
        lead_yaw = lead_data['transform']['rotation']['yaw']
        yaw_error = (lead_yaw - current_yaw) % 360
        if yaw_error > 180:
            yaw_error -= 360
        steer += yaw_error * STEERING_GAIN / 180.0  # Normalize to [-1, 1]
        steer = max(-1.0, min(1.0, steer))
        
        print(f"Control calculation - Distance: {distance:.2f}m, Angle: {math.degrees(angle):.2f}°, Target Speed: {target_speed:.2f}m/s, Current Speed: {current_speed:.2f}m/s")
        
        return {
            "throttle": throttle,
            "steer": steer,
            "brake": brake,
            "timestamp": time.time()
        }
    except Exception as e:
        print(f"Error calculating control: {e}")
        return {
            "throttle": 0.0,
            "steer": 0.0,
            "brake": 1.0,
            "timestamp": time.time()
        }

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Successfully connected to MQTT broker")
        client.subscribe("carla/gps", qos=1)
        print("Subscribed to topic: carla/gps")
    else:
        print(f"Failed to connect to MQTT broker with result code {rc}")

def on_message(client, userdata, msg):
    global lead_vehicle_data, follow_vehicle_data
    try:
        data = json.loads(msg.payload.decode())
        print(f"Received vehicle data: {data}")
        
        # Update vehicle data
        lead_vehicle_data = data['lead_vehicle']
        follow_vehicle_data = data['follow_vehicle']
        
        # Calculate and send control commands
        control_commands = calculate_control(lead_vehicle_data, follow_vehicle_data)
        client.publish("carla/control/vehicle2", json.dumps(control_commands), qos=1)
        print(f"Published control commands: {control_commands}")
        
    except Exception as e:
        print(f"Error processing message: {e}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from MQTT broker with result code {rc}")
    if rc != 0:
        print("Unexpected disconnection. Attempting to reconnect...")
        client.reconnect()

# Set up MQTT client
client = mqtt.Client(client_id=CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

# Connect to MQTT broker
print("Connecting to MQTT broker...")
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_forever() 