import json
import math
import boto3
import os
import logging
from datetime import datetime
from urllib.parse import unquote_plus

# Configure logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# Initialize IoT client
iot_client = boto3.client('iot-data')

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_control_commands(lead_data, follow_data):
    """Calculate control commands for the following vehicle."""
    try:
        # Get configuration from environment variables
        desired_distance = float(os.environ.get('DESIRED_DISTANCE', '5.0'))
        max_acceleration = float(os.environ.get('MAX_ACCELERATION', '3.0'))
        desired_speed = float(os.environ.get('DESIRED_SPEED', '30.0'))
        follow_speed = float(os.environ.get('FOLLOW_SPEED', '20.0'))
        time_headway = float(os.environ.get('TIME_HEADWAY', '1.5'))
        comfort_decel = float(os.environ.get('COMFORT_DECEL', '2.0'))
        
        # Extract data from both vehicles
        lead_location = lead_data['transform']['location']
        follow_location = follow_data['location']
        lead_speed = lead_data['velocity']['speed']
        follow_speed = follow_data.get('velocity', {}).get('speed', 0.0)
        
        # Calculate current distance between vehicles
        current_distance = calculate_distance(lead_location['x'], lead_location['y'], follow_location['x'], follow_location['y'])
        
        # Calculate desired speed based on lead vehicle speed
        desired_speed = min(lead_speed, follow_speed)
        
        # Calculate speed difference
        speed_diff = lead_speed - follow_speed
        
        # Calculate distance error
        distance_error = current_distance - desired_distance
        
        # Log all relevant values
        logger.info(f"Lead speed: {lead_speed}, Follow speed: {follow_speed}")
        logger.info(f"Current distance: {current_distance}, Desired distance: {desired_distance}")
        logger.info(f"Speed difference: {speed_diff}, Distance error: {distance_error}")
        
        # Calculate throttle based on speed difference and distance error
        # Make the control less conservative by reducing the thresholds
        if speed_diff > 1.0 or distance_error > 2.0:
            throttle = min(1.0, max_acceleration * 0.5)
            brake = 0.0
        elif speed_diff < -1.0 or distance_error < -2.0:
            throttle = 0.0
            brake = min(1.0, comfort_decel * 0.5)
        else:
            throttle = 0.3  # Maintain a small throttle to keep moving
            brake = 0.0
        
        # Calculate steering based on relative position
        # Make steering more responsive
        x_diff = lead_location['x'] - follow_location['x']
        y_diff = lead_location['y'] - follow_location['y']
        angle = math.atan2(y_diff, x_diff)
        steer = math.sin(angle) * 0.5  # Increase steering sensitivity
        
        # Log the final control commands
        logger.info(f"Calculated commands - Throttle: {throttle}, Steer: {steer}, Brake: {brake}")
        
        return {
            'throttle': throttle,
            'steer': steer,
            'brake': brake
        }
    except Exception as e:
        logger.error(f"Error calculating control commands: {str(e)}")
        return {
            'throttle': 0.0,
            'steer': 0.0,
            'brake': 0.0
        }

def lambda_handler(event, context):
    try:
        logger.info("Lambda function started")
        logger.info(f"Event: {json.dumps(event)}")

        # Initialize clients
        s3 = boto3.client('s3')
        iot = boto3.client('iot-data')

        # Get GPS data from S3
        bucket = event['Records'][0]['s3']['bucket']['name']
        key = unquote_plus(event['Records'][0]['s3']['object']['key'])
        
        logger.info(f"Getting GPS data from S3: bucket={bucket}, key={key}")
        
        try:
            response = s3.get_object(Bucket=bucket, Key=key)
            gps_data = json.loads(response['Body'].read().decode('utf-8'))
            logger.info(f"GPS data: {json.dumps(gps_data)}")
        except s3.exceptions.NoSuchKey:
            logger.error(f"File not found in S3: {key}")
            return {
                'statusCode': 404,
                'body': json.dumps(f'File not found: {key}')
            }
        except Exception as e:
            logger.error(f"Error reading from S3: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error reading GPS data from S3')
            }

        # Get follow vehicle data from IoT shadow
        try:
            logger.info("Getting follow vehicle data from IoT shadow")
            shadow_response = iot.get_thing_shadow(
                thingName='vehicle2'
            )
            follow_data = json.loads(shadow_response['payload'].read().decode('utf-8'))['state']['reported']
            logger.info(f"Follow vehicle data: {json.dumps(follow_data)}")
        except Exception as e:
            logger.error(f"Error getting follow vehicle data: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error getting follow vehicle data')
            }

        # Calculate control commands
        control_commands = calculate_control_commands(gps_data, follow_data)

        # Publish control commands to IoT topic
        logger.info(f"Publishing control commands: {json.dumps(control_commands)}")
        iot.publish(
            topic='carla/control/vehicle2',
            qos=1,
            payload=json.dumps(control_commands)
        )

        return {
            'statusCode': 200,
            'body': json.dumps('Control commands published successfully')
        }

    except Exception as e:
        logger.error(f"Error in lambda_handler: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps('Error processing GPS data')
        } 