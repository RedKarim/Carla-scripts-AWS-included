#!/usr/bin/env python

"""
CARLA 2-Car MPC Following Experiment

Controls:
    W/S:     Throttle/Brake
    A/D:     Steer Left/Right
    Space:   Hand Brake
    R:       Restart
    H:       Toggle Info
    ESC:     Quit
"""

import glob
import os
import sys
import numpy as np
from scipy.optimize import minimize
import carla
import pygame
import weakref
import math
import random
import time
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_b
from pygame.locals import K_c
from pygame.locals import K_d
from pygame.locals import K_g
from pygame.locals import K_h
from pygame.locals import K_i
from pygame.locals import K_l
from pygame.locals import K_m
from pygame.locals import K_n
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_t
from pygame.locals import K_v
from pygame.locals import K_w
from pygame.locals import K_x
from pygame.locals import K_z
from pygame.locals import K_MINUS
from pygame.locals import K_EQUALS
from carla import ColorConverter as cc

# MPC parameters
N = 10              # Prediction horizon
T = 0.1             # Sampling time
d_safe = 10         # Safe following distance
qv = 1              # Velocity error weight
r = 0.1             # Acceleration penalty weight
amin, amax = -3, 2  # Acceleration limits

class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            item.append(bp)
        self.index = None

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (
            force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

class MPCController:
    def __init__(self):
        self.last_control = 0.0
        
    def cost(self, u, x0, leader_traj):
        x = x0.copy()
        J = 0
        for k in range(N):
            a = u[k]
            x[0] += x[1] * T
            x[1] += a * T

            x1_k, v1_k = leader_traj[k]
            pos_err = (x1_k - x[0] - d_safe) ** 2
            vel_err = qv * (v1_k - x[1]) ** 2
            acc_cost = r * (a ** 2)

            J += pos_err + vel_err + acc_cost
        return J

    def get_control(self, follower_state, leader_traj):
        u0 = np.zeros(N)
        bounds = [(amin, amax)] * N
        res = minimize(self.cost, u0, args=(follower_state, leader_traj), bounds=bounds, method='SLSQP')
        return res.x[0]  # apply first control input

class World:
    def __init__(self, carla_world, hud):
        self.world = carla_world
        self.hud = hud
        self.leader = None
        self.follower = None
        self.camera_manager = None
        self.mpc_controller = MPCController()
        self.leader_trajectory = []
        self.follower_trajectory = []
        self.restart()

    def restart(self):
        if self.leader is not None:
            self.leader.destroy()
        if self.follower is not None:
            self.follower.destroy()
        if self.camera_manager is not None and self.camera_manager.sensor is not None:
            self.camera_manager.sensor.destroy()

        blueprint_library = self.world.get_blueprint_library()
        leader_bp = random.choice(blueprint_library.filter('vehicle.*'))
        leader_bp.set_attribute('role_name', 'leader')
        follower_bp = random.choice(blueprint_library.filter('vehicle.*'))
        follower_bp.set_attribute('role_name', 'follower')
        spawn_points = self.world.get_map().get_spawn_points()
        leader_spawn = random.choice(spawn_points)
        follower_spawn = carla.Transform(
            carla.Location(
                x=leader_spawn.location.x - 10.0,  # 10 meters behind
                y=leader_spawn.location.y,
                z=leader_spawn.location.z
            ),
            leader_spawn.rotation
        )
        self.leader = self.world.spawn_actor(leader_bp, leader_spawn)
        self.follower = self.world.spawn_actor(follower_bp, follower_spawn)
        self.camera_manager = CameraManager(self.leader, self.hud, 2.2)
        self.camera_manager.set_sensor(0)
        self.leader_trajectory = []
        self.follower_trajectory = []

    def update(self, control):
        # Apply manual control to leader
        self.leader.apply_control(control)

        # Get current states
        leader_transform = self.leader.get_transform()
        leader_velocity = self.leader.get_velocity()
        follower_transform = self.follower.get_transform()
        follower_velocity = self.follower.get_velocity()

        # Calculate leader trajectory for MPC
        leader_traj = []
        current_pos = leader_transform.location.x
        current_vel = math.sqrt(leader_velocity.x**2 + leader_velocity.y**2)
        for i in range(N):
            leader_traj.append([current_pos + i * current_vel * T, current_vel])

        # Get follower state
        follower_state = [
            follower_transform.location.x,
            math.sqrt(follower_velocity.x**2 + follower_velocity.y**2)
        ]

        # Calculate MPC control (acceleration)
        mpc_acc = self.mpc_controller.get_control(follower_state, np.array(leader_traj))

        # Convert acceleration to throttle/brake (simple proportional mapping)
        follower_control = carla.VehicleControl()
        # Assume max acceleration corresponds to full throttle, min to full brake
        if mpc_acc > 0:
            follower_control.throttle = min(mpc_acc / 2.0, 1.0)  # scale to [0,1]
            follower_control.brake = 0.0
        else:
            follower_control.throttle = 0.0
            follower_control.brake = min(-mpc_acc / 3.0, 1.0)  # scale to [0,1]
        follower_control.steer = 0.0  # always straight
        self.follower.apply_control(follower_control)

        self.leader_trajectory.append([current_pos, current_vel])
        self.follower_trajectory.append([follower_state[0], follower_state[1]])

    def destroy(self):
        if self.leader is not None:
            self.leader.destroy()
        if self.follower is not None:
            self.follower.destroy()

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.leader.get_transform()
        v = world.leader.get_velocity()
        c = world.leader.get_control()
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Leader Vehicle:',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '',
            'Follower Vehicle:',
            'Distance: % 15.1f m' % (t.location.x - world.follower.get_transform().location.x),
            '',
            'Controls:',
            'W/S:     Throttle/Brake',
            'A/D:     Steer Left/Right',
            'Space:   Hand Brake',
            'R:       Restart',
            'H:       Toggle Info',
            'ESC:     Quit'
        ]

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)

class HelpText(object):
    def __init__(self, font, width, height):
        lines = [
            'CARLA 2-Car MPC Following Experiment',
            '',
            'Controls:',
            'W/S:     Throttle/Brake',
            'A/D:     Steer Left/Right',
            'Space:   Hand Brake',
            'R:       Restart',
            'H:       Toggle Info',
            'ESC:     Quit'
        ]
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)

class KeyboardControl(object):
    def __init__(self, world):
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        self._autopilot_enabled = False
        self.world = world
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    self.world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    self.world.hud.help.toggle()
                elif event.key == K_r:
                    self.world.restart()
                elif event.key == K_ESCAPE:
                    return True

        keys = pygame.key.get_pressed()
        # Throttle/Brake
        if keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = 0.0
        if keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0
        # Steering
        steer_increment = 0.04
        if keys[K_a]:
            self._steer_cache = max(self._steer_cache - steer_increment, -0.7)
        elif keys[K_d]:
            self._steer_cache = min(self._steer_cache + steer_increment, 0.7)
        else:
            if self._steer_cache > 0:
                self._steer_cache = max(self._steer_cache - steer_increment, 0)
            else:
                self._steer_cache = min(self._steer_cache + steer_increment, 0)
        self._control.steer = round(self._steer_cache, 2)
        self._control.hand_brake = keys[K_SPACE]
        self._control.reverse = self._control.gear < 0
        self.world.leader.apply_control(self._control)
        return False

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

def main():
    pygame.init()
    pygame.font.init()
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    display = pygame.display.set_mode((1280, 720))
    hud = HUD(1280, 720)
    world_obj = World(world, hud)
    controller = KeyboardControl(world_obj)
    try:
        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events():
                return
            world_obj.update(controller._control)
            world.tick()
            world_obj.camera_manager.render(display)
            hud.render(display)
            pygame.display.flip()
    finally:
        world_obj.destroy()
        pygame.quit()

if __name__ == '__main__':
    main()
