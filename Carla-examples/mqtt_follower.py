#!/usr/bin/env python

"""
MQTT-based follower vehicle system for CARLA manual control.

This module provides vehicle following functionality with two modes:
1. MQTT mode: Leader publishes state via MQTT, follower subscribes (for network-based following)
2. Direct mode: Follower directly reads leader state from CARLA API (MQTT not required)

The system automatically falls back to direct mode if MQTT is unavailable.

Requirements:
    - paho-mqtt: pip install paho-mqtt (optional, for MQTT mode)
    - MQTT broker: Only needed for network-based following (default: localhost:1883)

Configuration:
    - MQTT_BROKER: MQTT broker address (default: "localhost")
    - MQTT_PORT: MQTT broker port (default: 1883)
    - FOLLOWER_DISTANCE: Target following distance in meters (default: 10.0)
"""

import json
import math
import threading
import time
import carla

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    mqtt = None
    print("警告: paho-mqttがインストールされていません。直接フォローモードのみ使用可能です。")
    print("MQTTモードを使用する場合は: pip install paho-mqtt")

try:
    from mqtt_latency_monitor import MQTTLatencyMonitor
    LATENCY_MONITOR_AVAILABLE = True
except ImportError:
    try:
        from mqtt_latency_monitor_simple import SimpleMQTTLatencyMonitor as MQTTLatencyMonitor
        LATENCY_MONITOR_AVAILABLE = True
        print("簡易レイテンシモニターを使用します（CSVのみ）。")
    except ImportError:
        LATENCY_MONITOR_AVAILABLE = False
        MQTTLatencyMonitor = None


MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_PUBLISH = "carla/leader/state"
MQTT_TOPIC_SUBSCRIBE = "carla/leader/state"
FOLLOWER_DISTANCE = 10.0


class MQTTVehiclePublisher:
    """MQTT publisher for vehicle state"""
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, topic=MQTT_TOPIC_PUBLISH, latency_monitor=None):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.connected = False
        self.latency_monitor = latency_monitor
        self.last_publish_time = None
        if MQTT_AVAILABLE:
            self.client = mqtt.Client(client_id="carla_leader_publisher")
            self._connect()
        else:
            print("MQTTパブリッシャー: paho-mqttが利用できないため無効化")

    def _connect(self):
        if not MQTT_AVAILABLE or not hasattr(self, 'client') or self.client is None:
            self.connected = False
            return
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            self.connected = True
            print(f"MQTT接続成功: {self.broker}:{self.port}")
        except Exception as e:
            print(f"MQTT接続エラー: {e}")
            print("MQTTブローカーが起動していない可能性があります。")
            self.connected = False

    def publish_state(self, vehicle):
        if not self.connected or vehicle is None:
            return
        
        try:
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            control = vehicle.get_control()
            
            publish_timestamp = time.perf_counter()
            
            state = {
                "location": {
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z
                },
                "rotation": {
                    "pitch": transform.rotation.pitch,
                    "yaw": transform.rotation.yaw,
                    "roll": transform.rotation.roll
                },
                "velocity": {
                    "x": velocity.x,
                    "y": velocity.y,
                    "z": velocity.z
                },
                "speed": math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6,
                "control": {
                    "throttle": control.throttle,
                    "steer": control.steer,
                    "brake": control.brake,
                    "hand_brake": control.hand_brake,
                    "reverse": control.reverse,
                    "gear": control.gear
                },
                "publish_timestamp": publish_timestamp,
                "system_time": time.time()
            }
            
            self.client.publish(self.topic, json.dumps(state), qos=1)
            
            if self.last_publish_time is not None and self.latency_monitor:
                publish_interval = (publish_timestamp - self.last_publish_time) * 1000
                if publish_interval > 0:
                    self.latency_monitor.record_latency(publish_interval, 'publish_interval')
            
            self.last_publish_time = publish_timestamp
        except Exception as e:
            print(f"MQTT送信エラー: {e}")

    def destroy(self):
        if self.connected:
            self.client.loop_stop()
            self.client.disconnect()


class MQTTFollowerVehicle:
    """Follower vehicle controlled via MQTT or direct following"""
    def __init__(self, world, leader_vehicle, broker=MQTT_BROKER, port=MQTT_PORT, topic=MQTT_TOPIC_SUBSCRIBE, latency_monitor=None):
        self.world = world
        self.leader = leader_vehicle
        self.broker = broker
        self.port = port
        self.topic = topic
        self.vehicle = None
        self.leader_state = None
        self.lock = threading.Lock()
        self.mqtt_connected = False
        self.latency_monitor = latency_monitor
        if MQTT_AVAILABLE:
            self.client = mqtt.Client(client_id="carla_follower_subscriber")
            self.client.on_connect = self._on_connect
            self.client.on_message = self._on_message
        else:
            self.client = None
            print("MQTTフォロワー: 直接フォローモードで動作します")
        self._spawn_follower()
        if MQTT_AVAILABLE:
            self._connect_mqtt()

    def _spawn_follower(self):
        if self.leader is None:
            return
        
        leader_transform = self.leader.get_transform()
        blueprint_library = self.world.get_blueprint_library()
        follower_bp = blueprint_library.filter('vehicle.*')[0]
        follower_bp.set_attribute('role_name', 'follower')
        if follower_bp.has_attribute('color'):
            follower_bp.set_attribute('color', '255,0,0')
        
        leader_yaw = math.radians(leader_transform.rotation.yaw)
        follower_location = carla.Location(
            x=leader_transform.location.x - FOLLOWER_DISTANCE * math.cos(leader_yaw),
            y=leader_transform.location.y - FOLLOWER_DISTANCE * math.sin(leader_yaw),
            z=leader_transform.location.z
        )
        follower_transform = carla.Transform(follower_location, leader_transform.rotation)
        
        self.vehicle = self.world.try_spawn_actor(follower_bp, follower_transform)
        if self.vehicle is None:
            print("フォロワー車両のスポーンに失敗しました")
        else:
            print(f"フォロワー車両をスポーンしました: {self.vehicle.id}")

    def _connect_mqtt(self):
        if not MQTT_AVAILABLE or self.client is None:
            self.mqtt_connected = False
            return
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            print(f"MQTT購読接続成功: {self.broker}:{self.port}")
            self.mqtt_connected = True
        except Exception as e:
            print(f"MQTT接続エラー: {e}")
            print("MQTTブローカーが起動していない可能性があります。")
            print("直接フォローモードに切り替えます（MQTT不要）。")
            self.mqtt_connected = False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.topic, qos=1)
            print(f"MQTT購読開始: {self.topic}")
            self.mqtt_connected = True
        else:
            print(f"MQTT接続失敗: リターンコード {rc}")
            print("直接フォローモードに切り替えます（MQTT不要）。")
            self.mqtt_connected = False

    def _on_message(self, client, userdata, msg):
        try:
            receive_timestamp = time.perf_counter()
            state = json.loads(msg.payload.decode())
            
            if self.latency_monitor and 'publish_timestamp' in state:
                latency_ms = (receive_timestamp - state['publish_timestamp']) * 1000
                if latency_ms > 0 and latency_ms < 1000:
                    self.latency_monitor.record_latency(latency_ms, 'round_trip')
            
            with self.lock:
                self.leader_state = state
        except Exception as e:
            print(f"MQTTメッセージ解析エラー: {e}")

    def _get_leader_state_direct(self):
        """Get leader state directly from CARLA API (MQTT fallback)"""
        if self.leader is None:
            return None
        
        try:
            transform = self.leader.get_transform()
            velocity = self.leader.get_velocity()
            control = self.leader.get_control()
            
            state = {
                "location": {
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z
                },
                "rotation": {
                    "pitch": transform.rotation.pitch,
                    "yaw": transform.rotation.yaw,
                    "roll": transform.rotation.roll
                },
                "velocity": {
                    "x": velocity.x,
                    "y": velocity.y,
                    "z": velocity.z
                },
                "speed": math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6,
                "control": {
                    "throttle": control.throttle,
                    "steer": control.steer,
                    "brake": control.brake,
                    "hand_brake": control.hand_brake,
                    "reverse": control.reverse,
                    "gear": control.gear
                },
                "timestamp": time.time()
            }
            return state
        except Exception as e:
            print(f"リーダー状態取得エラー: {e}")
            return None

    def update(self):
        if self.vehicle is None or self.leader is None:
            return
        
        if self.mqtt_connected:
            try:
                if not hasattr(self.client, '_sock') or self.client._sock is None:
                    self.mqtt_connected = False
                    return
            except:
                self.mqtt_connected = False
                return
            
            with self.lock:
                if self.leader_state is None:
                    return
                state = self.leader_state
        else:
            state = self._get_leader_state_direct()
            if state is None:
                return
        
        try:
            leader_location = carla.Location(
                x=state["location"]["x"],
                y=state["location"]["y"],
                z=state["location"]["z"]
            )
            leader_rotation = carla.Rotation(
                pitch=state["rotation"]["pitch"],
                yaw=state["rotation"]["yaw"],
                roll=state["rotation"]["roll"]
            )
            
            follower_transform = self.vehicle.get_transform()
            follower_location = follower_transform.location
            
            distance = math.sqrt(
                (leader_location.x - follower_location.x)**2 +
                (leader_location.y - follower_location.y)**2
            )
            
            target_speed = state["speed"] / 3.6
            current_velocity = self.vehicle.get_velocity()
            current_speed = math.sqrt(current_velocity.x**2 + current_velocity.y**2)
            
            control = carla.VehicleControl()
            
            if distance > FOLLOWER_DISTANCE * 1.5:
                control.throttle = 0.8
                control.brake = 0.0
            elif distance < FOLLOWER_DISTANCE * 0.5:
                control.throttle = 0.0
                control.brake = 0.5
            else:
                speed_diff = target_speed - current_speed
                if speed_diff > 0.5:
                    control.throttle = min(0.7, speed_diff / 5.0)
                    control.brake = 0.0
                elif speed_diff < -0.5:
                    control.throttle = 0.0
                    control.brake = min(0.5, abs(speed_diff) / 5.0)
                else:
                    control.throttle = 0.1
                    control.brake = 0.0
            
            follower_yaw = math.radians(follower_transform.rotation.yaw)
            dx = leader_location.x - follower_location.x
            dy = leader_location.y - follower_location.y
            
            angle_to_leader = math.atan2(dy, dx)
            yaw_diff = angle_to_leader - follower_yaw
            
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            
            control.steer = max(-1.0, min(1.0, yaw_diff * 0.8))
            control.hand_brake = False
            control.reverse = state["control"]["reverse"]
            
            self.vehicle.apply_control(control)
        except Exception as e:
            print(f"フォロワー更新エラー: {e}")

    def destroy(self):
        if self.vehicle is not None:
            self.vehicle.destroy()
        if self.client is not None and MQTT_AVAILABLE:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except:
                pass
