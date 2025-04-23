import boto3
import json
import math
import os
import gzip

s3 = boto3.client('s3')
iot = boto3.client('iot-data', region_name=os.environ['AWS_REGION'])

def idm_control(distance, velocity_lead, velocity_follow):
    desired_gap = 10.0  # meters
    time_headway = 1.5  # seconds
    max_accel = 1.0
    desired_velocity = 15.0  # m/s (54 km/h)
    comfortable_brake = 1.5

    delta_v = velocity_follow - velocity_lead
    s_star = desired_gap + velocity_follow * time_headway + (velocity_follow * delta_v) / (2 * math.sqrt(max_accel * comfortable_brake))
    acceleration = max_accel * (1 - (velocity_follow / desired_velocity)**4 - (s_star / max(distance, 0.1))**2)
    acceleration = max(min(acceleration, 1.0), -1.0)
    return acceleration

def lambda_handler(event, context):
    bucket = event['Records'][0]['s3']['bucket']['name']
    key = event['Records'][0]['s3']['object']['key']

    response = s3.get_object(Bucket=bucket, Key=key)
    content = response['Body'].read().decode('utf-8')
    gps_data = json.loads(content)

    lat = gps_data.get('latitude')
    lon = gps_data.get('longitude')
    timestamp = gps_data.get('timestamp')

    # Dummy follower position (static for now)
    lat2 = lat - 0.0001
    lon2 = lon

    # Approximate distance (very naive)
    distance = math.sqrt((lat - lat2)**2 + (lon - lon2)**2) * 111000  # degrees to meters

    # For now assume lead is moving, follower is stopped
    vel_lead = 10.0  # m/s
    vel_follower = 0.0

    acc = idm_control(distance, vel_lead, vel_follower)

    control_msg = {
        "throttle": max(acc, 0),
        "brake": abs(min(acc, 0)),
        "steer": 0.0
    }

    iot.publish(
        topic="carla/control/vehicle2",
        qos=1,
        payload=json.dumps(control_msg)
    )

    return {
        'statusCode': 200,
        'body': json.dumps('Control command sent')
    }