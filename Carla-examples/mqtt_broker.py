#!/usr/bin/env python
"""
Simple Python MQTT Broker for CARLA follower system.

This script provides a lightweight MQTT broker that doesn't require
installing Mosquitto. It uses the hbmqtt library which is a pure
Python implementation.

Installation:
    pip install hbmqtt

Usage:
    python mqtt_broker.py

The broker will run on localhost:1883 by default.
"""

import asyncio
import sys
import signal

try:
    from hbmqtt.broker import Broker
    from hbmqtt.mqtt.constants import QOS_0, QOS_1, QOS_2
except ImportError:
    print("ERROR: hbmqtt library not installed!")
    print("\nPlease install it with:")
    print("  pip install hbmqtt")
    print("\nOr use an alternative:")
    print("  pip install aiomqtt")
    sys.exit(1)

# Broker configuration
BROKER_CONFIG = {
    'listeners': {
        'default': {
            'type': 'tcp',
            'bind': '127.0.0.1:1883',
        },
    },
    'auth': {
        'allow-anonymous': True,
    },
    'topic-check': {
        'enabled': False,
    }
}

broker = None
loop = None

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\n\nShutting down MQTT broker...')
    if broker:
        asyncio.create_task(broker.shutdown())
    if loop:
        loop.stop()
    sys.exit(0)

async def broker_coro():
    """Main broker coroutine"""
    global broker
    broker = Broker(BROKER_CONFIG)
    await broker.start()
    print("=" * 60)
    print("MQTT Broker started successfully!")
    print("=" * 60)
    print(f"Broker running on: {BROKER_CONFIG['listeners']['default']['bind']}")
    print("Press Ctrl+C to stop the broker")
    print("=" * 60)
    print()

def main():
    """Main function"""
    global loop
    
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(broker_coro())
        
        # Keep the broker running
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            pass
        finally:
            loop.run_until_complete(broker.shutdown())
            loop.close()
            
    except Exception as e:
        print(f"Error starting MQTT broker: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
