import json
import boto3
import math
import gzip
import time
import logging

# Set up logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

iot = boto3.client('iot-data', region_name='ap-northeast-1')
s3 = boto3.client('s3')

# IDM parameters (tuned for platooning)
DESIRED_DISTANCE = 5.0  # meters (reduced for closer following)
MAX_ACCELERATION = 1.5   # m/s² (increased for more responsive following)
DESIRED_SPEED = 15.0     # m/s (~54 km/h)
FOLLOW_SPEED = 12.0      # m/s
TIME_HEADWAY = 1.0       # seconds (reduced for tighter following)
COMFORT_DECEL = 2.5      # m/s² (increased for more responsive braking)

# State holder (reset every function call)
previous_lat = None
previous_lon = None
previous_time = None
last_control_time = None

def latlon_to_meters(lat1, lon1, lat2, lon2):
    # Rough conversion using Haversine formula
    R = 6371000  # radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_idm_acceleration(velocity, distance, leader_velocity=0):
    """Calculate acceleration using Intelligent Driver Model"""
    # Calculate desired distance
    desired_distance = DESIRED_DISTANCE + velocity * TIME_HEADWAY
    
    # Calculate acceleration
    if distance > 0:
        acceleration = MAX_ACCELERATION * (
            1 - (velocity / DESIRED_SPEED)**4 - 
            (desired_distance / distance)**2
        )
    else:
        acceleration = -MAX_ACCELERATION  # Emergency braking if too close
    
    return acceleration

def lambda_handler(event, context):
    global previous_lat, previous_lon, previous_time, last_control_time
    
    try:
        logger.info("Lambda function triggered")
        logger.info(f"Event: {json.dumps(event)}")

        # === 1. Get the uploaded file info ===
        record = event['Records'][0]
        bucket = record['s3']['bucket']['name']
        key = record['s3']['object']['key']
        logger.info(f"Processing S3 object: {bucket}/{key}")

        # === 2. Download and parse JSON ===
        response = s3.get_object(Bucket=bucket, Key=key)
        gps_data = json.loads(response['Body'].read().decode('utf-8'))
        logger.info(f"GPS data: {json.dumps(gps_data)}")

        lat = gps_data["latitude"]
        lon = gps_data["longitude"]
        timestamp = gps_data["timestamp"]

        # Calculate velocity
        velocity = 0.0
        if previous_lat is not None:
            dist = latlon_to_meters(previous_lat, previous_lon, lat, lon)
            dt = timestamp - previous_time
            if dt > 0:
                velocity = dist / dt
            logger.info(f"Calculated velocity: {velocity} m/s")

        # Save for next invocation
        previous_lat = lat
        previous_lon = lon
        previous_time = timestamp

        # === 3. IDM-based control ===
        # Calculate acceleration using IDM
        acceleration = calculate_idm_acceleration(velocity, DESIRED_DISTANCE)
        logger.info(f"Calculated acceleration: {acceleration} m/s²")
        
        # Convert acceleration to control values
        throttle = min(max(acceleration / MAX_ACCELERATION, 0.0), 1.0)
        brake = 0.0 if acceleration > 0 else min(abs(acceleration) / COMFORT_DECEL, 1.0)
        logger.info(f"Control values - throttle: {throttle}, brake: {brake}")

        # === 4. Publish to MQTT ===
        message = {
            "throttle": throttle,
            "steer": 0.0,  # No steering for now
            "brake": brake,
            "timestamp": timestamp
        }

        topic = "carla/control/vehicle2"
        logger.info(f"Publishing to topic: {topic}")
        logger.info(f"Message payload: {json.dumps(message)}")
        
        response = iot.publish(
            topic=topic,
            qos=1,
            payload=json.dumps(message)
        )
        
        logger.info(f"Publish response: {response}")
        logger.info("Control command sent successfully")
        
        return {
            'statusCode': 200,
            'body': json.dumps('Control command sent to carla/control/vehicle2')
        }

    except Exception as e:
        logger.error(f"Error in Lambda handler: {e}")
        return {
            'statusCode': 500,
            'body': json.dumps(f'Error: {str(e)}')
        }
