import glob
import os
import sys
import carla
import time
import json
import pygame
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pygame.locals import K_w, K_s, K_a, K_d, K_q, K_ESCAPE
import paho.mqtt.client as mqtt
import math
from collections import deque

# ========== CONFIGURATION ==========
NUM_FOLLOWING_VEHICLES = 3  # Number of following vehicles to spawn
VEHICLE_SPACING = 5.0  # Distance between vehicles in meters
TARGET_SPEED = 30.0  # Target speed in km/h
TRAFFIC_LIGHT_DISTANCE = 10.0  # Distance to start checking for traffic lights

# Lead vehicle spawn configuration
LEAD_VEHICLE_LOCATION = carla.Location(x=71.651, y=16.345, z=2.381)
LEAD_VEHICLE_ROTATION = carla.Rotation(pitch=1.50, yaw=180.0, roll=-0.000)

# Plot configuration
PLOT_HISTORY = 100  # Number of data points to show in the plot
TIME_WINDOW = 10.0  # Time window in seconds

# Data storage for plotting
class VehicleData:
    def __init__(self):
        self.times = deque(maxlen=PLOT_HISTORY)
        self.velocities = deque(maxlen=PLOT_HISTORY)
        self.accelerations = deque(maxlen=PLOT_HISTORY)
        self.displacements = deque(maxlen=PLOT_HISTORY)
        self.last_time = time.time()
        self.last_velocity = 0.0
        self.last_position = None
        self.total_displacement = 0.0

# Initialize data storage for all vehicles
vehicle_data = {}

# Set up the plot
plt.ion()  # Enable interactive mode
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('Vehicle Motion Data')

# Initialize plots
lines = {}
for i in range(1, NUM_FOLLOWING_VEHICLES + 2):  # +2 because we start from vehicle 1
    vehicle_data[i] = VehicleData()
    lines[i] = {
        'velocity': ax1.plot([], [], label=f'Vehicle {i}')[0],
        'acceleration': ax2.plot([], [], label=f'Vehicle {i}')[0],
        'displacement': ax3.plot([], [], label=f'Vehicle {i}')[0]
    }

# Configure subplots
ax1.set_ylabel('Velocity (km/h)')
ax1.set_title('Velocity over Time')
ax1.grid(True)
ax1.legend()

ax2.set_ylabel('Acceleration (m/s²)')
ax2.set_title('Acceleration over Time')
ax2.grid(True)
ax2.legend()

ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Displacement (m)')
ax3.set_title('Displacement over Time')
ax3.grid(True)
ax3.legend()

# Adjust layout
plt.tight_layout()

def update_vehicle_data(vehicle_id, vehicle):
    """Update the motion data for a vehicle."""
    current_time = time.time()
    data = vehicle_data[vehicle_id]
    
    # Get current velocity
    velocity = vehicle.get_velocity()
    current_velocity = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # Convert to km/h
    
    # Get current position
    current_position = vehicle.get_location()
    
    # Calculate acceleration
    if data.last_velocity is not None:
        dt = current_time - data.last_time
        if dt > 0:
            acceleration = (current_velocity - data.last_velocity) / dt
        else:
            acceleration = 0
    else:
        acceleration = 0
    
    # Calculate displacement
    if data.last_position is not None:
        displacement = current_position.distance(data.last_position)
        data.total_displacement += displacement
    else:
        displacement = 0
    
    # Update data
    data.times.append(current_time)
    data.velocities.append(current_velocity)
    data.accelerations.append(acceleration)
    data.displacements.append(data.total_displacement)
    
    # Update last values
    data.last_time = current_time
    data.last_velocity = current_velocity
    data.last_position = current_position

def update_plot():
    """Update the plot with new data."""
    for vehicle_id, data in vehicle_data.items():
        if len(data.times) > 0:
            # Convert times to relative time
            relative_times = [t - data.times[0] for t in data.times]
            
            # Update line data
            lines[vehicle_id]['velocity'].set_data(relative_times, data.velocities)
            lines[vehicle_id]['acceleration'].set_data(relative_times, data.accelerations)
            lines[vehicle_id]['displacement'].set_data(relative_times, data.displacements)
    
    # Adjust axis limits
    for ax in [ax1, ax2, ax3]:
        ax.relim()
        ax.autoscale_view()
    
    # Redraw the plot
    fig.canvas.draw()
    fig.canvas.flush_events()

# ========== MQTT SETUP ==========
MQTT_BROKER = "10.21.89.70"  # IP address of your Mac running the MQTT broker
MQTT_PORT = 1883
CLIENT_ID = "carla_client"

# Add global variables for latency tracking
last_send_time = 0
last_receive_time = 0
round_trip_start_time = 0

# Dictionary to store following vehicles and their controls
following_vehicles = {}
following_controls = {}

# Control state variables for smooth acceleration
current_throttle = 0.0
current_brake = 0.0
current_steer = 0.0
THROTTLE_INCREMENT = 0.05
BRAKE_INCREMENT = 0.1
STEER_INCREMENT = 0.05

def get_traffic_light_state(vehicle, world):
    """Get the state of the traffic light affecting the vehicle using CARLA's native methods."""
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        if traffic_light:
            return traffic_light.get_state()
    return None

def update_lead_vehicle_control(vehicle, world):
    """Update the control of the lead vehicle based on traffic conditions."""
    global current_throttle, current_brake, current_steer
    
    try:
        # Get current speed in km/h
        velocity = vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # Convert to km/h
        
        # Check traffic light state
        traffic_light_state = get_traffic_light_state(vehicle, world)
        
        # Control logic
        if traffic_light_state == carla.TrafficLightState.Red:
            # Stop at red light
            current_throttle = 0.0
            current_brake = min(current_brake + BRAKE_INCREMENT, 1.0)
            print("Red light detected - stopping")
        elif traffic_light_state == carla.TrafficLightState.Yellow:
            # Slow down for yellow light
            current_throttle = max(current_throttle - THROTTLE_INCREMENT, 0.0)
            current_brake = min(current_brake + BRAKE_INCREMENT * 0.5, 0.5)
            print("Yellow light detected - slowing down")
        else:
            # Normal driving
            if current_speed < TARGET_SPEED:
                current_throttle = min(current_throttle + THROTTLE_INCREMENT, 0.7)
                current_brake = max(current_brake - BRAKE_INCREMENT, 0.0)
            else:
                current_throttle = max(current_throttle - THROTTLE_INCREMENT, 0.0)
                current_brake = 0.0
        
        # Keep steering straight
        current_steer = 0.0
        
        # Apply control
        control = carla.VehicleControl()
        control.throttle = current_throttle
        control.brake = current_brake
        control.steer = current_steer
        vehicle.apply_control(control)
        
    except Exception as e:
        print(f"Error in update_lead_vehicle_control: {e}")
        import traceback
        traceback.print_exc()

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Successfully connected to MQTT broker")
        # Subscribe to control topics for all following vehicles
        for vehicle_id in range(2, NUM_FOLLOWING_VEHICLES + 2):  # Vehicle IDs start from 2
            topic = f"carla/control/vehicle{vehicle_id}"
            client.subscribe(topic, qos=1)
            print(f"Subscribed to topic: {topic}")
    else:
        print(f"Failed to connect to MQTT broker with result code {rc}")

def on_subscribe(client, userdata, mid, granted_qos):
    print(f"Successfully subscribed to topic with message ID {mid}")
    print(f"Granted QoS levels: {granted_qos}")

def on_message(client, userdata, msg):
    global last_receive_time, round_trip_start_time
    print(f"\nReceived message on topic: {msg.topic}")
    print(f"Raw message payload: {msg.payload.decode()}")
    try:
        control_data = json.loads(msg.payload.decode())
        print(f"Parsed control data: {control_data}")
        
        # Extract vehicle ID from topic
        vehicle_id = int(msg.topic.split('/')[-1].replace('vehicle', ''))
        print(f"Processing control for vehicle {vehicle_id}")
        
        if vehicle_id not in following_controls:
            print(f"Warning: Received control for unknown vehicle {vehicle_id}")
            return
            
        # Calculate processing latency
        if 'timestamp' in control_data:
            last_receive_time = time.time()
            processing_latency = (last_receive_time - control_data['timestamp']) * 1000
            print(f"Processing latency for vehicle {vehicle_id}: {processing_latency:.2f} ms")
            
            if round_trip_start_time > 0:
                round_trip_latency = (last_receive_time - round_trip_start_time) * 1000
                print(f"Round trip latency for vehicle {vehicle_id}: {round_trip_latency:.2f} ms")
        
        # Apply control values with validation and smoothing
        target_throttle = max(0.0, min(1.0, float(control_data.get("throttle", 0.0))))
        target_steer = max(-1.0, min(1.0, float(control_data.get("steer", 0.0))))
        target_brake = max(0.0, min(1.0, float(control_data.get("brake", 0.0))))
        
        print(f"Target control values - throttle: {target_throttle}, steer: {target_steer}, brake: {target_brake}")
        
        # Direct assignment of control values
        following_controls[vehicle_id].throttle = target_throttle
        following_controls[vehicle_id].steer = target_steer
        following_controls[vehicle_id].brake = target_brake
        
        print(f"Applied control values for vehicle {vehicle_id} - throttle: {following_controls[vehicle_id].throttle}, steer: {following_controls[vehicle_id].steer}, brake: {following_controls[vehicle_id].brake}")
    except Exception as e:
        print(f"Error processing message: {e}")
        import traceback
        traceback.print_exc()

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from MQTT broker with result code {rc}")
    if rc != 0:
        print("Unexpected disconnection. Attempting to reconnect...")
        client.reconnect()

# MQTT listener for Car 2 control
follow_control = carla.VehicleControl()

# Set up MQTT client
mqtt_client = mqtt.Client(client_id=CLIENT_ID)
mqtt_client.on_connect = on_connect
mqtt_client.on_subscribe = on_subscribe
mqtt_client.on_message = on_message
mqtt_client.on_disconnect = on_disconnect

# Connect to MQTT broker
print("Connecting to MQTT broker...")
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# Wait for MQTT connection to be established
time.sleep(1)

# ========== CARLA SETUP ==========
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major, sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Autonomous Lead Vehicle with Following Vehicles")

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# Get the map and spawn points
carla_map = world.get_map()
spawn_points = carla_map.get_spawn_points()

# Spawn lead vehicle at specified location
vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
lead_transform = carla.Transform(LEAD_VEHICLE_LOCATION, LEAD_VEHICLE_ROTATION)
lead_vehicle = world.spawn_actor(vehicle_bp, lead_transform)
print(f"Spawned lead vehicle at location: ({LEAD_VEHICLE_LOCATION.x}, {LEAD_VEHICLE_LOCATION.y}, {LEAD_VEHICLE_LOCATION.z})")

# Wait a moment to ensure the lead vehicle is properly spawned
time.sleep(1)

# Spawn following vehicles
prev_transform = lead_transform
for i in range(NUM_FOLLOWING_VEHICLES):
    # Calculate spawn position behind the previous vehicle
    # Use the vehicle's forward vector to determine the offset direction
    forward_vector = prev_transform.get_forward_vector()
    follow_location = carla.Location(
        x=prev_transform.location.x - VEHICLE_SPACING * forward_vector.x,
        y=prev_transform.location.y - VEHICLE_SPACING * forward_vector.y,
        z=prev_transform.location.z
    )
    follow_transform = carla.Transform(follow_location, prev_transform.rotation)
    
    # Try to spawn the vehicle
    try:
        vehicle_id = i + 2  # Vehicle IDs start from 2
        following_vehicles[vehicle_id] = world.spawn_actor(vehicle_bp, follow_transform)
        following_controls[vehicle_id] = carla.VehicleControl()
        following_controls[vehicle_id].throttle = 0.0
        following_controls[vehicle_id].steer = 0.0
        following_controls[vehicle_id].brake = 0.0
        print(f"Spawned following vehicle {vehicle_id} at location: ({follow_location.x}, {follow_location.y})")
        
        # Update previous transform for next vehicle
        prev_transform = follow_transform
        
        # Wait a moment to ensure the vehicle is properly spawned
        time.sleep(0.5)
    except Exception as e:
        print(f"Failed to spawn following vehicle {vehicle_id}: {e}")
        continue

# Camera on Car 1
cam_bp = bp_lib.find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", "800")
cam_bp.set_attribute("image_size_y", "600")
cam_bp.set_attribute("fov", "90")
cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(cam_bp, cam_transform, attach_to=lead_vehicle)

camera_surface = None
def process_image(image):
    global camera_surface
    img = np.frombuffer(image.raw_data, dtype=np.uint8)
    img = np.reshape(img, (image.height, image.width, 4))
    img = img[:, :, :3][:, :, ::-1]
    camera_surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))

camera.listen(lambda image: process_image(image))

# GNSS for Car 1 (publishes to MQTT)
gnss_bp = bp_lib.find("sensor.other.gnss")
gnss = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=lead_vehicle)

last_sent_time = 0
GPS_UPDATE_INTERVAL = 0.1  # 100ms update rate for smoother platooning

def gps_callback(event):
    global last_sent_time, round_trip_start_time
    now = time.time()
    if now - last_sent_time >= GPS_UPDATE_INTERVAL:
        try:
            # Get lead vehicle data
            lead_velocity = lead_vehicle.get_velocity()
            lead_speed = math.sqrt(lead_velocity.x**2 + lead_velocity.y**2 + lead_velocity.z**2)
            lead_transform = lead_vehicle.get_transform()
            lead_location = lead_transform.location
            lead_rotation = lead_transform.rotation
            lead_control = lead_vehicle.get_control()
            
            # Prepare data package with timestamp
            data = {
                "lead_vehicle": {
                    "latitude": event.latitude,
                    "longitude": event.longitude,
                    "altitude": event.altitude,
                    "velocity": {
                        "x": lead_velocity.x,
                        "y": lead_velocity.y,
                        "z": lead_velocity.z,
                        "speed": lead_speed
                    },
                    "transform": {
                        "location": {
                            "x": lead_location.x,
                            "y": lead_location.y,
                            "z": lead_location.z
                        },
                        "rotation": {
                            "pitch": lead_rotation.pitch,
                            "yaw": lead_rotation.yaw,
                            "roll": lead_rotation.roll
                        }
                    },
                    "control": {
                        "throttle": lead_control.throttle,
                        "steer": lead_control.steer,
                        "brake": lead_control.brake,
                        "hand_brake": lead_control.hand_brake,
                        "reverse": lead_control.reverse,
                        "manual_gear_shift": lead_control.manual_gear_shift,
                        "gear": lead_control.gear
                    }
                },
                "following_vehicles": {},
                "timestamp": now
            }
            
            # Add data for each following vehicle
            for vehicle_id, vehicle in following_vehicles.items():
                follow_velocity = vehicle.get_velocity()
                follow_speed = math.sqrt(follow_velocity.x**2 + follow_velocity.y**2 + follow_velocity.z**2)
                follow_transform = vehicle.get_transform()
                follow_location = follow_transform.location
                follow_rotation = follow_transform.rotation
                follow_control = vehicle.get_control()
                
                data["following_vehicles"][str(vehicle_id)] = {
                    "velocity": {
                        "x": follow_velocity.x,
                        "y": follow_velocity.y,
                        "z": follow_velocity.z,
                        "speed": follow_speed
                    },
                    "transform": {
                        "location": {
                            "x": follow_location.x,
                            "y": follow_location.y,
                            "z": follow_location.z
                        },
                        "rotation": {
                            "pitch": follow_rotation.pitch,
                            "yaw": follow_rotation.yaw,
                            "roll": follow_rotation.roll
                        }
                    },
                    "control": {
                        "throttle": follow_control.throttle,
                        "steer": follow_control.steer,
                        "brake": follow_control.brake,
                        "hand_brake": follow_control.hand_brake,
                        "reverse": follow_control.reverse,
                        "manual_gear_shift": follow_control.manual_gear_shift,
                        "gear": follow_control.gear
                    }
                }
            
            # Record start time for round trip measurement
            round_trip_start_time = time.time()
            
            # Publish data and measure latency
            payload = json.dumps(data)
            print(f"\nPublishing GPS data to topic carla/gps:")
            print(f"Lead vehicle speed: {lead_speed:.2f} m/s")
            print(f"Number of following vehicles: {len(following_vehicles)}")
            mqtt_client.publish("carla/gps", payload, qos=1)
            publish_latency = (time.time() - round_trip_start_time) * 1000
            print(f"Publish latency: {publish_latency:.2f} ms")
            
            print(f"Sent vehicle data to MQTT broker")
            last_sent_time = now
        except Exception as e:
            print(f"Error publishing vehicle data: {e}")
            import traceback
            traceback.print_exc()

# Set up GNSS sensor
gnss_bp = bp_lib.find("sensor.other.gnss")
gnss = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=lead_vehicle)
print("Spawned GNSS sensor")

# Start GNSS sensor
gnss.listen(lambda event: gps_callback(event))
print("Started GNSS sensor")

# Main loop
clock = pygame.time.Clock()
running = True
lead_control = carla.VehicleControl()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key in [K_ESCAPE, K_q]):
            running = False

    # Update lead vehicle control automatically
    update_lead_vehicle_control(lead_vehicle, world)
    
    # Update data for lead vehicle
    update_vehicle_data(1, lead_vehicle)

    # Apply control to all following vehicles
    for vehicle_id, vehicle in following_vehicles.items():
        control = following_controls[vehicle_id]
        vehicle.apply_control(control)
        # Update data for following vehicles
        update_vehicle_data(vehicle_id, vehicle)

    # Update the plot
    update_plot()

    if camera_surface:
        screen.blit(camera_surface, (0, 0))
    pygame.display.flip()
    clock.tick(30)

# Cleanup
plt.close('all')  # Close all matplotlib windows
camera.stop()
gnss.stop()
camera.destroy()
gnss.destroy()
lead_vehicle.destroy()
for vehicle in following_vehicles.values():
    vehicle.destroy()
mqtt_client.loop_stop()
mqtt_client.disconnect()
pygame.quit()