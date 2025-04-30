import json
import boto3
import math
import time
import logging

# Set up logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

iot = boto3.client('iot-data', region_name='ap-northeast-1')
s3 = boto3.client('s3')

# Control parameters
DESIRED_DISTANCE = 5.0  # meters
MAX_SPEED = 15.0       # m/s
MIN_SPEED = 0.0        # m/s
STEERING_GAIN = 0.5    # Steering sensitivity

def calculate_control(leader_data):
    try:
        # Extract leader vehicle data
        leader_speed = leader_data['velocity']['speed']
        leader_control = leader_data['control']
        
        # Calculate target speed based on leader's speed
        target_speed = min(leader_speed, MAX_SPEED)
        
        # Calculate throttle based on target speed
        if target_speed > 0:
            throttle = min(1.0, target_speed / MAX_SPEED)
        else:
            throttle = 0.0
            
        # Use leader's steering as base, with some smoothing
        steer = leader_control['steer'] * STEERING_GAIN
        
        # Calculate brake based on speed difference
        brake = 0.0
        if leader_speed < MIN_SPEED:
            brake = 1.0
            
        # Prepare control message
        control_message = {
            "throttle": throttle,
            "steer": steer,
            "brake": brake,
            "timestamp": time.time()
        }
        
        logger.info(f"Calculated control: {control_message}")
        return control_message
        
    except Exception as e:
        logger.error(f"Error calculating control: {e}")
        # Return safe default values
        return {
            "throttle": 0.0,
            "steer": 0.0,
            "brake": 1.0,
            "timestamp": time.time()
        }

def lambda_handler(event, context):
    try:
        logger.info("Lambda function triggered")
        logger.info(f"Event: {json.dumps(event)}")

        # Get the uploaded file info
        record = event['Records'][0]
        bucket = record['s3']['bucket']['name']
        key = record['s3']['object']['key']
        logger.info(f"Processing S3 object: {bucket}/{key}")

        # Download and parse JSON
        response = s3.get_object(Bucket=bucket, Key=key)
        vehicle_data = json.loads(response['Body'].read().decode('utf-8'))
        logger.info(f"Vehicle data: {json.dumps(vehicle_data)}")

        # Calculate control values
        control_message = calculate_control(vehicle_data)
        
        # Publish control message
        topic = "carla/control/vehicle2"
        logger.info(f"Publishing to topic: {topic}")
        logger.info(f"Control message: {json.dumps(control_message)}")
        
        response = iot.publish(
            topic=topic,
            qos=1,
            payload=json.dumps(control_message)
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
