import paho.mqtt.client as mqtt
import json
import time
import math

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
CLIENT_ID = "virtual_controller"

# Car following parameters
desired_speed = 20.0  # Increased base desired speed
safe_time_headway = 1.5  # Reduced time headway for quicker response
max_acceleration = 2.5  # Increased base acceleration
comfortable_deceleration = 4.0  # Increased deceleration for better safety
minimum_spacing = 6.0  # Reduced minimum spacing
delta = 4.0  # acceleration exponent

# Advanced Safety Parameters
emergency_braking_distance = 8.0  # Increased emergency braking distance
critical_distance = 4.0  # Increased critical distance
relative_speed_threshold = 4.0  # Reduced relative speed threshold for earlier intervention
prediction_horizon = 1.5  # Increased prediction horizon
curve_detection_threshold = math.radians(15)  # Minimum angle for curve detection

# Initial acceleration parameters
initial_acceleration_boost = 1.5  # Boost factor for initial acceleration
initial_distance_threshold = 15.0  # Distance threshold for initial acceleration

# Vehicle state storage
lead_vehicle_data = None
follow_vehicle_data = None

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_control(lead_data, follow_data):
    """Calculate control commands using an advanced car-following model with predictive elements."""
    try:
        if lead_data is None or follow_data is None:
            print("Missing vehicle data, cannot calculate control")
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
                "timestamp": time.time()
            }

        # Get current states
        lead_location = lead_data['transform']['location']
        follow_location = follow_data['transform']['location']
        lead_speed = lead_data['velocity']['speed']
        follow_speed = follow_data['velocity']['speed']
        lead_rotation = lead_data['transform']['rotation']
        follow_rotation = follow_data['transform']['rotation']
        
        # Calculate relative states
        relative_speed = follow_speed - lead_speed
        current_distance = calculate_distance(lead_location['x'], lead_location['y'], 
                                           follow_location['x'], follow_location['y'])
        
        # Calculate relative heading and path curvature
        lead_yaw = math.radians(lead_rotation.get('yaw', 0.0))
        follow_yaw = math.radians(follow_rotation.get('yaw', 0.0))
        heading_diff = abs(lead_yaw - follow_yaw)
        if heading_diff > math.pi:
            heading_diff = 2 * math.pi - heading_diff
        
        # Scenario Detection and Adaptive Parameters
        is_curve = heading_diff > curve_detection_threshold
        is_approaching = relative_speed > 0 and current_distance < emergency_braking_distance
        is_accelerating = lead_speed > follow_speed and current_distance > minimum_spacing
        is_initial_following = current_distance > initial_distance_threshold and follow_speed < 2.0
        
        # Adaptive Parameters based on Scenario
        current_desired_speed = desired_speed
        current_safe_time_headway = safe_time_headway
        current_minimum_spacing = minimum_spacing
        current_max_acceleration = max_acceleration
        
        if is_initial_following:
            current_max_acceleration *= initial_acceleration_boost
            current_desired_speed = max(desired_speed, lead_speed * 1.2)
            current_safe_time_headway *= 0.8
        
        if is_curve:
            current_desired_speed *= 0.7
            current_safe_time_headway *= 1.2
            current_minimum_spacing *= 1.2
            
        if is_approaching:
            current_desired_speed = min(current_desired_speed, lead_speed)
            current_safe_time_headway *= 1.3
            current_max_acceleration *= 0.7
            
        if is_accelerating:
            current_max_acceleration *= 1.2
            
        # Enhanced Predictive Safety Checks
        predicted_distance = current_distance + relative_speed * prediction_horizon
        predicted_relative_speed = relative_speed + (lead_speed - follow_speed) * prediction_horizon
        
        # More aggressive emergency conditions
        if current_distance < critical_distance or predicted_distance < critical_distance:
            print("Critical distance - full emergency stop")
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
                "timestamp": time.time()
            }
            
        if (current_distance < emergency_braking_distance and 
            (relative_speed > relative_speed_threshold or predicted_relative_speed > relative_speed_threshold)):
            print("Emergency braking - approaching too fast")
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
                "timestamp": time.time()
            }
        
        # Enhanced IDM with Predictive Elements
        base_gap = current_minimum_spacing + (follow_speed * current_safe_time_headway)
        relative_speed_gap = ((follow_speed * (follow_speed - lead_speed)) / 
                            (2 * math.sqrt(current_max_acceleration * comfortable_deceleration)))
        curve_gap = current_minimum_spacing * 0.2 if is_curve else 0.0
        desired_minimum_gap = base_gap + relative_speed_gap + curve_gap
        
        if predicted_relative_speed > 0:
            desired_minimum_gap *= (1 + predicted_relative_speed / 10.0)
        
        # Enhanced IDM Acceleration Calculation with initial boost
        free_road_term = 1 - (follow_speed / current_desired_speed)**delta
        interaction_term = (desired_minimum_gap / current_distance)**2
        relative_speed_term = 0.5 * (relative_speed / current_desired_speed)**2
        curve_term = 0.3 if is_curve else 0.0
        
        acceleration = current_max_acceleration * (free_road_term - interaction_term - relative_speed_term - curve_term)
        
        # Apply initial acceleration boost
        if is_initial_following:
            acceleration *= initial_acceleration_boost
        
        # Convert acceleration to control commands with enhanced safety
        if acceleration > 0:
            distance_factor = min(1.0, current_distance / (2 * current_minimum_spacing))
            speed_factor = 1.0 - (follow_speed / current_desired_speed) if follow_speed > current_desired_speed else 1.0
            curve_factor = 1.0 - (heading_diff / math.pi) if is_curve else 1.0
            
            # Enhanced throttle control
            throttle = min(acceleration / current_max_acceleration, 0.8) * distance_factor * speed_factor * curve_factor
            if is_initial_following:
                throttle = max(throttle, 0.4)  # Ensure minimum throttle during initial following
            brake = 0.0
        else:
            throttle = 0.0
            brake_factor = 1.0 + (abs(relative_speed) / relative_speed_threshold)
            brake = min(-acceleration / comfortable_deceleration, 0.9) * brake_factor
        
        # Advanced Steering Control with enhanced safety
        dx = lead_location['x'] - follow_location['x']
        dy = lead_location['y'] - follow_location['y']
        angle_to_lead = math.atan2(dy, dx)
        
        angle_diff = angle_to_lead - follow_yaw
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        # Dynamic steering gain with enhanced safety
        base_kp_steer = 0.2
        speed_factor = max(0.5, min(1.0, follow_speed / current_desired_speed))
        distance_factor = min(1.0, current_distance / current_minimum_spacing)
        curve_factor = 1.2 if is_curve else 1.0
        
        # Reduce steering gain when too close
        if current_distance < emergency_braking_distance:
            base_kp_steer *= 0.5
        
        kp_steer = base_kp_steer * speed_factor * distance_factor * curve_factor
        
        predicted_angle = angle_diff + (lead_yaw - follow_yaw) * prediction_horizon
        steer = kp_steer * (0.7 * angle_diff + 0.3 * predicted_angle)
        steer = max(-0.4, min(0.4, steer))
        
        print(f"Control calculation - Distance: {current_distance:.2f}m, Relative Speed: {relative_speed:.2f}m/s")
        print(f"Throttle: {throttle:.2f}, Steer: {steer:.2f}, Brake: {brake:.2f}")
        
        return {
            "throttle": float(throttle),
            "steer": float(steer),
            "brake": float(brake),
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