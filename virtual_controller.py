import paho.mqtt.client as mqtt
import json
import time
import math

# MQTT Configuration
MQTT_BROKER = "192.168.0.127"  # IP address of the device running the MQTT broker
MQTT_PORT = 1883
CLIENT_ID = "virtual_controller"

# Car following parameters
desired_speed = 30.0  # Increased desired speed
safe_time_headway = 1.0  # Reduced time headway for quicker response
max_acceleration = 3.0  # Increased acceleration
comfortable_deceleration = 3.0  # Reduced deceleration for smoother following
minimum_spacing = 5.0  # Reduced minimum spacing
delta = 2.0  # Reduced acceleration exponent for smoother response
Kp_steer = 0.8  # Increased steering gain for better tracking

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
following_vehicles_data = {}

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_control(lead_vehicle, follow_vehicle, vehicle_id):
    # Calculate distance and relative speed
    distance = calculate_distance(
        lead_vehicle['transform']['location']['x'],
        lead_vehicle['transform']['location']['y'],
        follow_vehicle['transform']['location']['x'],
        follow_vehicle['transform']['location']['y']
    )
    
    relative_speed = lead_vehicle['velocity']['speed'] - follow_vehicle['velocity']['speed']
    
    # Calculate desired speed based on lead vehicle, with minimum value
    desired_speed = max(lead_vehicle['velocity']['speed'], 2.0)  # Increased minimum desired speed
    
    # Calculate acceleration using IDM
    s_star = minimum_spacing + max(0, follow_vehicle['velocity']['speed'] * safe_time_headway + 
                                 (follow_vehicle['velocity']['speed'] * relative_speed) / 
                                 (2 * math.sqrt(max_acceleration * comfortable_deceleration)))
    
    # Calculate acceleration with safety checks
    current_speed = max(follow_vehicle['velocity']['speed'], 0.1)  # Ensure non-zero current speed
    
    # Enhanced acceleration calculation
    free_road_term = 1 - (current_speed / desired_speed)**delta
    interaction_term = (s_star / max(distance, 0.1))**2
    
    # Add initial acceleration boost when far from lead vehicle
    if distance > initial_distance_threshold:
        free_road_term *= initial_acceleration_boost
    
    acceleration = max_acceleration * (free_road_term - interaction_term)
    
    # Convert acceleration to throttle/brake with enhanced control
    if acceleration > 0:
        # Ensure minimum throttle when following
        throttle = max(0.4, min(1.0, acceleration / max_acceleration))
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(1.0, -acceleration / comfortable_deceleration)
    
    # Calculate steering based on relative position
    dx = lead_vehicle['transform']['location']['x'] - follow_vehicle['transform']['location']['x']
    dy = lead_vehicle['transform']['location']['y'] - follow_vehicle['transform']['location']['y']
    target_angle = math.atan2(dy, dx)
    current_angle = follow_vehicle['transform']['rotation']['yaw'] * math.pi / 180.0
    angle_diff = target_angle - current_angle
    
    # Normalize angle difference to [-pi, pi]
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    # Enhanced steering calculation
    distance_factor = min(1.0, distance / (2 * minimum_spacing))
    speed_factor = min(1.0, current_speed / desired_speed)
    
    # Calculate steering with dynamic gain
    steer = Kp_steer * angle_diff * distance_factor * speed_factor
    steer = max(-0.4, min(0.4, steer))  # Limit steering to reasonable range
    
    # Emergency braking
    if distance < emergency_braking_distance:
        brake = 1.0
        throttle = 0.0
        steer *= 0.5  # Reduce steering during emergency braking
    
    # Force minimum throttle for movement
    if distance > minimum_spacing and throttle < 0.4:
        throttle = 0.4
    
    # Debug logging
    print(f"Control calculation for vehicle {vehicle_id}:")
    print(f"  Distance: {distance:.2f}m, Relative speed: {relative_speed:.2f}m/s")
    print(f"  Desired speed: {desired_speed:.2f}m/s, Current speed: {follow_vehicle['velocity']['speed']:.2f}m/s")
    print(f"  Acceleration: {acceleration:.2f}m/s²")
    print(f"  Angle diff: {angle_diff*180/math.pi:.2f}°")
    print(f"  Control outputs - Throttle: {throttle:.2f}, Steer: {steer:.2f}, Brake: {brake:.2f}")
    
    return {
        'throttle': float(throttle),
        'steer': float(steer),
        'brake': float(brake)
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
    global lead_vehicle_data, following_vehicles_data
    try:
        data = json.loads(msg.payload.decode())
        print(f"\nReceived vehicle data - Lead speed: {data['lead_vehicle']['velocity']['speed']:.2f} m/s")
        
        # Update vehicle data
        lead_vehicle_data = data['lead_vehicle']
        following_vehicles_data = data['following_vehicles']
        
        # Calculate and send control commands for each following vehicle
        for vehicle_id, follow_data in following_vehicles_data.items():
            try:
                # Get the vehicle in front (either lead vehicle or previous following vehicle)
                front_vehicle_data = lead_vehicle_data
                if int(vehicle_id) > 2:  # If not the first following vehicle
                    prev_vehicle_id = str(int(vehicle_id) - 1)
                    if prev_vehicle_id in following_vehicles_data:
                        front_vehicle_data = following_vehicles_data[prev_vehicle_id]
                
                # Calculate control based on the vehicle in front
                control_commands = calculate_control(front_vehicle_data, follow_data, vehicle_id)
                
                # Debug logging
                distance = calculate_distance(
                    front_vehicle_data['transform']['location']['x'],
                    front_vehicle_data['transform']['location']['y'],
                    follow_data['transform']['location']['x'],
                    follow_data['transform']['location']['y']
                )
                print(f"Vehicle {vehicle_id} - Front speed: {front_vehicle_data['velocity']['speed']:.2f} m/s, "
                      f"Follow speed: {follow_data['velocity']['speed']:.2f} m/s, "
                      f"Distance: {distance:.2f} m")
                
                # Ensure minimum throttle for movement
                if distance > minimum_spacing and control_commands['throttle'] < 0.4:
                    control_commands['throttle'] = 0.4
                
                # Add timestamp to control commands
                control_commands['timestamp'] = time.time()
                
                # Publish control commands
                topic = f"carla/control/vehicle{vehicle_id}"
                payload = json.dumps(control_commands)
                print(f"Publishing to topic {topic}: {payload}")
                client.publish(topic, payload, qos=1)
                print(f"Published control commands for vehicle {vehicle_id} - "
                      f"throttle: {control_commands['throttle']:.2f}, "
                      f"steer: {control_commands['steer']:.2f}, "
                      f"brake: {control_commands['brake']:.2f}")
                
            except Exception as e:
                print(f"Error processing vehicle {vehicle_id}: {e}")
                import traceback
                traceback.print_exc()
        
    except Exception as e:
        print(f"Error processing message: {e}")
        import traceback
        traceback.print_exc()

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
client.loop_start()

# Wait for MQTT connection to be established
time.sleep(1)

# Keep the script running
print("Controller is running. Press Ctrl+C to exit.")
try:
    while True:
        time.sleep(0.1)  # Small sleep to prevent high CPU usage
except KeyboardInterrupt:
    print("\nShutting down controller...")
    client.loop_stop()
    client.disconnect()
    print("Controller stopped.") 