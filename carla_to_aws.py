import glob
import os
import sys
import carla
import time
import json
import ssl
import pygame
import numpy as np
from pygame.locals import K_w, K_s, K_a, K_d, K_q, K_ESCAPE
from dotenv import load_dotenv
import paho.mqtt.client as mqtt
import math

# ========== ENV + MQTT SETUP ==========
load_dotenv()
AWS_ENDPOINT = os.getenv("AWS_ENDPOINT")
CLIENT_ID = os.getenv("CLIENT_ID") + "_car2"
CA_PATH = os.getenv("AWS_CA_PATH")
CERT_PATH = os.getenv("AWS_CERT_PATH")
KEY_PATH = os.getenv("AWS_KEY_PATH")

# Add global variables for latency tracking
last_send_time = 0
last_receive_time = 0
round_trip_start_time = 0

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Successfully connected to AWS IoT")
        # Subscribe to the control topic
        client.subscribe("carla/control/vehicle2", qos=1)
        print("Subscribed to topic: carla/control/vehicle2")
    else:
        print(f"Failed to connect to AWS IoT with result code {rc}")

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
        
        # Calculate AWS to CARLA latency
        if 'aws_timestamp' in control_data:
            last_receive_time = time.time()
            aws_to_carla_latency = (last_receive_time - control_data['aws_timestamp']) * 1000
            print(f"AWS to CARLA latency: {aws_to_carla_latency:.2f} ms")
            
            # Calculate round trip latency if we have the start time
            if round_trip_start_time > 0:
                round_trip_latency = (last_receive_time - round_trip_start_time) * 1000
                print(f"Round trip latency: {round_trip_latency:.2f} ms")
        
        # Apply control values with validation and smoothing
        target_throttle = max(0.0, min(1.0, float(control_data.get("throttle", 0.0))))
        target_steer = max(-1.0, min(1.0, float(control_data.get("steer", 0.0))))
        target_brake = max(0.0, min(1.0, float(control_data.get("brake", 0.0))))
        
        # Smooth transitions with reduced smoothing factor for more responsive control
        follow_control.throttle = follow_control.throttle + (target_throttle - follow_control.throttle) * 0.7
        follow_control.steer = follow_control.steer + (target_steer - follow_control.steer) * 0.5
        follow_control.brake = follow_control.brake + (target_brake - follow_control.brake) * 0.7
        
        # Update vehicle state in IoT shadow
        try:
            velocity = follow_vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            transform = follow_vehicle.get_transform()
            
            state_update = {
                "state": {
                    "reported": {
                        "location": {
                            "x": transform.location.x,
                            "y": transform.location.y,
                            "z": transform.location.z
                        },
                        "rotation": {
                            "yaw": transform.rotation.yaw,
                            "pitch": transform.rotation.pitch,
                            "roll": transform.rotation.roll
                        },
                        "velocity": {
                            "x": velocity.x,
                            "y": velocity.y,
                            "z": velocity.z,
                            "speed": speed
                        }
                    }
                }
            }
            
            mqtt_client.publish(
                "$aws/things/vehicle2/shadow/update",
                json.dumps(state_update),
                qos=1
            )
        except Exception as e:
            print(f"Error updating vehicle state: {e}")
        
        print(f"Applied control values - throttle: {follow_control.throttle}, steer: {follow_control.steer}, brake: {follow_control.brake}")
    except Exception as e:
        print(f"Error processing message: {e}")
        # Reset to safe values on error
        follow_control.throttle = 0.0
        follow_control.steer = 0.0
        follow_control.brake = 1.0

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from AWS IoT with result code {rc}")
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

# Configure TLS
mqtt_client.tls_set(ca_certs=CA_PATH, certfile=CERT_PATH, keyfile=KEY_PATH, tls_version=ssl.PROTOCOL_TLSv1_2)

# Connect to AWS IoT
print("Connecting to AWS IoT...")
mqtt_client.connect(AWS_ENDPOINT, 8883, 60)
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

# Spawn Car 2 (15 meters behind Car 1)
follow_spawn = carla.Transform(
    carla.Location(
        x=lead_spawn.location.x - 15 * math.cos(math.radians(lead_spawn.rotation.yaw)),
        y=lead_spawn.location.y - 15 * math.sin(math.radians(lead_spawn.rotation.yaw)),
        z=lead_spawn.location.z
    ),
    lead_spawn.rotation
)
follow_vehicle = world.spawn_actor(vehicle_bp, follow_spawn)
print("Spawned following vehicle")

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

# GNSS for Car 1 (publishes to AWS)
gnss_bp = bp_lib.find("sensor.other.gnss")
gnss = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=lead_vehicle)

last_sent_time = 0
GPS_UPDATE_INTERVAL = 0.1  # 100ms update rate for smoother platooning

def gps_callback(event):
    global last_sent_time, round_trip_start_time
    now = time.time()
    if now - last_sent_time >= GPS_UPDATE_INTERVAL:
        try:
            # Get vehicle velocity
            velocity = lead_vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # m/s
            
            # Get vehicle transform
            transform = lead_vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            
            # Get vehicle control state
            control = lead_vehicle.get_control()
            
            # Prepare data package with timestamp
            data = {
                "latitude": event.latitude,
                "longitude": event.longitude,
                "altitude": event.altitude,
                "carla_timestamp": now,  # Add CARLA timestamp
                "velocity": {
                    "x": velocity.x,
                    "y": velocity.y,
                    "z": velocity.z,
                    "speed": speed
                },
                "transform": {
                    "location": {
                        "x": location.x,
                        "y": location.y,
                        "z": location.z
                    },
                    "rotation": {
                        "pitch": rotation.pitch,
                        "yaw": rotation.yaw,
                        "roll": rotation.roll
                    }
                },
                "control": {
                    "throttle": control.throttle,
                    "steer": control.steer,
                    "brake": control.brake,
                    "hand_brake": control.hand_brake,
                    "reverse": control.reverse,
                    "manual_gear_shift": control.manual_gear_shift,
                    "gear": control.gear
                }
            }
            
            # Record start time for round trip measurement
            round_trip_start_time = time.time()
            
            # Publish data and measure CARLA to AWS latency
            mqtt_client.publish("carla/gps", json.dumps(data), qos=1)
            carla_to_aws_latency = (time.time() - round_trip_start_time) * 1000
            print(f"CARLA to AWS latency: {carla_to_aws_latency:.2f} ms")
            
            print(f"Sent vehicle data to AWS: {json.dumps(data)}")
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
        current_throttle = min(current_throttle + THROTTLE_INCREMENT, 0.7)  # Max 70% throttle
        current_brake = 0.0
    elif keys[K_s]:
        current_brake = min(current_brake + BRAKE_INCREMENT, 1.0)
        current_throttle = 0.0
    else:
        current_throttle = max(current_throttle - THROTTLE_INCREMENT, 0.0)
        current_brake = max(current_brake - BRAKE_INCREMENT, 0.0)
    
    # Smooth steering control
    if keys[K_a]:
        current_steer = max(current_steer - STEER_INCREMENT, -0.7)  # Smooth left turn, max 70% left
    elif keys[K_d]:
        current_steer = min(current_steer + STEER_INCREMENT, 0.7)   # Smooth right turn, max 70% right
    else:
        # Return steering to center
        if current_steer > 0:
            current_steer = max(current_steer - STEER_INCREMENT, 0.0)
        else:
            current_steer = min(current_steer + STEER_INCREMENT, 0.0)
    
    # Apply lead vehicle control
    lead_control.throttle = current_throttle
    lead_control.brake = current_brake
    lead_control.steer = current_steer
    lead_vehicle.apply_control(lead_control)

    # Apply control to following vehicle
    follow_vehicle.apply_control(follow_control)
    print(f"Applied control to following vehicle: throttle={follow_control.throttle}, steer={follow_control.steer}, brake={follow_control.brake}")

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
follow_vehicle.destroy()
mqtt_client.loop_stop()
mqtt_client.disconnect()
pygame.quit()