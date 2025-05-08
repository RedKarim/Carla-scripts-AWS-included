import glob
import os
import sys
import carla
import time
import json
import pygame
import numpy as np
from pygame.locals import K_w, K_s, K_a, K_d, K_q, K_ESCAPE
import paho.mqtt.client as mqtt
import math

# ========== CONFIGURATION ==========
NUM_FOLLOWING_VEHICLES = 3  # Number of following vehicles to spawn
VEHICLE_SPACING = 15.0  # Distance between vehicles in meters

# ========== MQTT SETUP ==========
MQTT_BROKER = "192.168.0.127"  # Your Mac's IP address
MQTT_PORT = 1883
CLIENT_ID = "carla_client"

# Add global variables for latency tracking
last_send_time = 0
last_receive_time = 0
round_trip_start_time = 0

# Dictionary to store following vehicles and their controls
following_vehicles = {}
following_controls = {}

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Successfully connected to MQTT broker")
        # Subscribe to the control topic
        client.subscribe("carla/control/vehicle2", qos=1)
        print("Subscribed to topic: carla/control/vehicle2")
    else:
        print(f"Failed to connect to MQTT broker with result code {rc}")

def on_subscribe(client, userdata, mid, granted_qos):
    print(f"Successfully subscribed to topic with message ID {mid}")
    print(f"Granted QoS levels: {granted_qos}")

def on_message(client, userdata, msg):
    global last_receive_time, round_trip_start_time
    print(f"Received message on topic: {msg.topic}")
    print(f"Message payload: {msg.payload.decode()}")
    try:
        control_data = json.loads(msg.payload.decode())
        print(f"Parsed control data: {control_data}")
        
        # Extract vehicle ID from topic
        vehicle_id = int(msg.topic.split('/')[-1].replace('vehicle', ''))
        
        if vehicle_id not in following_controls:
            following_controls[vehicle_id] = carla.VehicleControl()
        
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
        
        # Smooth transitions with reduced smoothing factor for more responsive control
        following_controls[vehicle_id].throttle = following_controls[vehicle_id].throttle + (target_throttle - following_controls[vehicle_id].throttle) * 0.7
        following_controls[vehicle_id].steer = following_controls[vehicle_id].steer + (target_steer - following_controls[vehicle_id].steer) * 0.5
        following_controls[vehicle_id].brake = following_controls[vehicle_id].brake + (target_brake - following_controls[vehicle_id].brake) * 0.7
        
        print(f"Applied control values for vehicle {vehicle_id} - throttle: {following_controls[vehicle_id].throttle}, steer: {following_controls[vehicle_id].steer}, brake: {following_controls[vehicle_id].brake}")
    except Exception as e:
        print(f"Error processing message: {e}")
        # Reset to safe values on error
        following_controls[vehicle_id].throttle = 0.0
        following_controls[vehicle_id].steer = 0.0
        following_controls[vehicle_id].brake = 1.0

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

# ========== CARLA SETUP ==========
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major, sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Car 1 Manual, Car 2 MQTT Control")

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# Spawn Car 1 (manual)
vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
spawn_points = world.get_map().get_spawn_points()
lead_spawn = spawn_points[0]
lead_vehicle = world.spawn_actor(vehicle_bp, lead_spawn)
print("Spawned lead vehicle")

# Spawn following vehicles
prev_spawn = lead_spawn
for i in range(NUM_FOLLOWING_VEHICLES):
    # Calculate spawn position based on previous vehicle
    follow_spawn = carla.Transform(
        carla.Location(
            x=prev_spawn.location.x - VEHICLE_SPACING * math.cos(math.radians(prev_spawn.rotation.yaw)),
            y=prev_spawn.location.y - VEHICLE_SPACING * math.sin(math.radians(prev_spawn.rotation.yaw)),
            z=prev_spawn.location.z
        ),
        prev_spawn.rotation
    )
    
    vehicle_id = i + 2  # Vehicle IDs start from 2
    following_vehicles[vehicle_id] = world.spawn_actor(vehicle_bp, follow_spawn)
    following_controls[vehicle_id] = carla.VehicleControl()
    print(f"Spawned following vehicle {vehicle_id}")
    
    # Update previous spawn point for next vehicle
    prev_spawn = follow_spawn

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
            mqtt_client.publish("carla/gps", json.dumps(data), qos=1)
            publish_latency = (time.time() - round_trip_start_time) * 1000
            print(f"Publish latency: {publish_latency:.2f} ms")
            
            print(f"Sent vehicle data to MQTT broker")
            last_sent_time = now
        except Exception as e:
            print(f"Error publishing vehicle data: {e}")

gnss.listen(lambda event: gps_callback(event))

# Main loop
clock = pygame.time.Clock()
running = True
lead_control = carla.VehicleControl()

# Control state variables for smooth acceleration
current_throttle = 0.0
current_brake = 0.0
current_steer = 0.0
THROTTLE_INCREMENT = 0.05  # Reduced for smoother acceleration
BRAKE_INCREMENT = 0.1
STEER_INCREMENT = 0.05  # Reduced for smoother steering

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key in [K_ESCAPE, K_q]):
            running = False

    keys = pygame.key.get_pressed()
    
    # Smooth throttle control
    if keys[K_w]:
        current_throttle = min(current_throttle + THROTTLE_INCREMENT, 0.7)
        current_brake = 0.0
    elif keys[K_s]:
        current_brake = min(current_brake + BRAKE_INCREMENT, 1.0)
        current_throttle = 0.0
    else:
        current_throttle = max(current_throttle - THROTTLE_INCREMENT, 0.0)
        current_brake = max(current_brake - BRAKE_INCREMENT, 0.0)
    
    # Smooth steering control
    if keys[K_a]:
        current_steer = max(current_steer - STEER_INCREMENT, -0.7)
    elif keys[K_d]:
        current_steer = min(current_steer + STEER_INCREMENT, 0.7)
    else:
        if current_steer > 0:
            current_steer = max(current_steer - STEER_INCREMENT, 0.0)
        else:
            current_steer = min(current_steer + STEER_INCREMENT, 0.0)
    
    # Apply lead vehicle control
    lead_control.throttle = current_throttle
    lead_control.brake = current_brake
    lead_control.steer = current_steer
    lead_vehicle.apply_control(lead_control)

    # Apply control to all following vehicles
    for vehicle_id, vehicle in following_vehicles.items():
        vehicle.apply_control(following_controls[vehicle_id])
        print(f"Applied control to vehicle {vehicle_id}: throttle={following_controls[vehicle_id].throttle}, steer={following_controls[vehicle_id].steer}, brake={following_controls[vehicle_id].brake}")

    if camera_surface:
        screen.blit(camera_surface, (0, 0))
    pygame.display.flip()
    clock.tick(30)

# Cleanup
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