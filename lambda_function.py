import json
import boto3
import math
import gzip

iot = boto3.client('iot-data', region_name='ap-northeast-1')
s3 = boto3.client('s3')

# IDM parameters (simplified)
DESIRED_DISTANCE = 10.0  # meters
MAX_ACCELERATION = 1.0   # m/s²
DESIRED_SPEED = 15.0     # m/s (~54 km/h)
FOLLOW_SPEED = 12.0      # m/s
TIME_HEADWAY = 1.5       # seconds
COMFORT_DECEL = 2.0      # m/s²

# State holder (reset every function call)
previous_lat = None
previous_lon = None
previous_time = None

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

def lambda_handler(event, context):
    global previous_lat, previous_lon, previous_time

    # === 1. Get the uploaded file info ===
    record = event['Records'][0]
    bucket = record['s3']['bucket']['name']
    key = record['s3']['object']['key']

    # === 2. Download and parse JSON ===
    response = s3.get_object(Bucket=bucket, Key=key)
    gps_data = json.loads(response['Body'].read().decode('utf-8'))

    lat = gps_data["latitude"]
    lon = gps_data["longitude"]
    timestamp = gps_data["timestamp"]

    velocity = 0.0
    if previous_lat is not None:
        dist = latlon_to_meters(previous_lat, previous_lon, lat, lon)
        dt = timestamp - previous_time
        if dt > 0:
            velocity = dist / dt

    # Save for next invocation
    previous_lat = lat
    previous_lon = lon
    previous_time = timestamp

    # === 3. IDM-based simple response ===
    # We'll use relative velocity and target speed
    desired_distance = DESIRED_DISTANCE + velocity * TIME_HEADWAY
    acceleration = MAX_ACCELERATION * (1 - (velocity / DESIRED_SPEED)**4)

    # Simplified throttle logic
    throttle = min(max(acceleration / MAX_ACCELERATION, 0.0), 1.0)
    brake = 0.0 if acceleration > 0 else min(abs(acceleration) / COMFORT_DECEL, 1.0)

    # === 4. Publish to MQTT ===
    message = {
        "throttle": throttle,
        "steer": 0.0,
        "brake": brake
    }

    iot.publish(
        topic="carla/control/vehicle2",
        qos=1,
        payload=json.dumps(message)
    )

    print("Published:", message)
    return {
        'statusCode': 200,
        'body': json.dumps('Control command sent to carla/control/vehicle2')
    }
