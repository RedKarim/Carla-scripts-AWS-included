import paho.mqtt.client as mqtt
import time

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")

def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()}")

# Create client instance
client = mqtt.Client()

# Set callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
print("Connecting to MQTT broker...")
try:
    client.connect("127.0.0.1", 1883, 60)
    client.loop_start()
    
    # Subscribe to a test topic
    client.subscribe("test/topic")
    
    # Publish a test message
    client.publish("test/topic", "Hello MQTT!")
    
    # Keep the script running
    time.sleep(5)
    
except Exception as e:
    print(f"Error: {e}")
finally:
    client.loop_stop()
    client.disconnect() 