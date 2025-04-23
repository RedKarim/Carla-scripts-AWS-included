
#!/usr/bin/env python3

import glob
import os
import sys
import time
import json
import ssl
import argparse
import logging

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import pygame
import numpy as np
from pygame.locals import K_w, K_s, K_a, K_d, K_q, K_ESCAPE

from dotenv import load_dotenv
import paho.mqtt.client as mqtt

# ========== MQTT Setup ==========
load_dotenv()
AWS_ENDPOINT = os.getenv("AWS_ENDPOINT")
CLIENT_ID = os.getenv("CLIENT_ID")
CA_PATH = os.getenv("AWS_CA_PATH")
CERT_PATH = os.getenv("AWS_CERT_PATH")
KEY_PATH = os.getenv("AWS_KEY_PATH")

mqtt_client = mqtt.Client(client_id=CLIENT_ID)
mqtt_client.tls_set(ca_certs=CA_PATH, certfile=CERT_PATH, keyfile=KEY_PATH, tls_version=ssl.PROTOCOL_TLSv1_2)
mqtt_client.connect(AWS_ENDPOINT, 8883, 60)
mqtt_client.loop_start()
last_sent_time = 0

# ========== Carla Setup ==========
pygame.init()
display_width, display_height = 800, 600
screen = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("Manual Control")

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()

blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

# GNSS sensor
gnss_bp = blueprint_library.find("sensor.other.gnss")
gnss = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=vehicle)

# RGB camera sensor
camera_bp = blueprint_library.find("sensor.camera.rgb")
camera_bp.set_attribute("image_size_x", str(display_width))
camera_bp.set_attribute("image_size_y", str(display_height))
camera_bp.set_attribute("fov", "90")

camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))  # front hood
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# Frame buffer
camera_surface = None
def process_image(image):
    global camera_surface
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    camera_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

camera.listen(lambda image: process_image(image))

# GPS Listener → Send to AWS every 3 seconds
def gps_listener(event):
    global last_sent_time
    now = time.time()
    if now - last_sent_time >= 3:
        data = {
            "latitude": event.latitude,
            "longitude": event.longitude,
            "timestamp": now
        }
        mqtt_client.publish("carla/gps", json.dumps(data))
        print("Sent to AWS:", data)
        last_sent_time = now

gnss.listen(lambda event: gps_listener(event))

# Main control loop
clock = pygame.time.Clock()
running = True
control = carla.VehicleControl()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == K_ESCAPE or event.key == K_q:
                running = False

    keys = pygame.key.get_pressed()
    control.throttle = 1.0 if keys[K_w] else 0.0
    control.steer = -1.0 if keys[K_a] else 1.0 if keys[K_d] else 0.0
    control.brake = 1.0 if keys[K_s] else 0.0
    vehicle.apply_control(control)

    if camera_surface:
        screen.blit(camera_surface, (0, 0))
    pygame.display.flip()
    clock.tick(30)

# Cleanup
camera.stop()
camera.destroy()
gnss.stop()
gnss.destroy()
vehicle.destroy()
mqtt_client.loop_stop()
mqtt_client.disconnect()
pygame.quit()