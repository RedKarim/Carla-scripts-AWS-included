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
    """Calculate control commands using the Intelligent Driver Model (IDM) for car following."""
    try:
        # IDM Parameters - tuned for better response
        desired_speed = float(os.environ.get('MAX_SPEED', '15.0'))  # Reduced to 15 m/s for better control
        safe_time_headway = 1.5  # Increased for more safety margin
        max_acceleration = float(os.environ.get('MAX_ACCELERATION', '1.5'))  # Reduced for smoother acceleration
        comfortable_deceleration = 2.0  # m/s^2
        minimum_spacing = 7.0  # Increased minimum spacing
        delta = 4.0  # acceleration exponent
        
        # Extract and validate vehicle data
        if not all(key in lead_data.get('transform', {}) for key in ['location', 'rotation']):
            logger.error("Invalid lead vehicle transform data")
            return {'throttle': 0.0, 'steer': 0.0, 'brake': 1.0}
            
        if not all(key in follow_data for key in ['location', 'rotation', 'velocity']):
            logger.error("Invalid follow vehicle data")
            return {'throttle': 0.0, 'steer': 0.0, 'brake': 1.0}
        
        lead_location = lead_data['transform']['location']
        follow_location = follow_data['location']
        lead_speed = lead_data['velocity']['speed']
        follow_speed = follow_data['velocity']['speed']
        
        # Get vehicle headings
        lead_rotation = lead_data['transform']['rotation']
        follow_rotation = follow_data['rotation']
        lead_yaw = math.radians(lead_rotation.get('yaw', 0.0))
        follow_yaw = math.radians(follow_rotation.get('yaw', 0.0))
        
        # Calculate current distance
        current_distance = calculate_distance(lead_location['x'], lead_location['y'], 
                                           follow_location['x'], follow_location['y'])
        
        # Log state
        logger.info(f"Lead speed: {lead_speed}, Follow speed: {follow_speed}")
        logger.info(f"Current distance: {current_distance}")
        
        # Initialize control commands
        throttle = 0.0
        brake = 0.0
        steer = 0.0
        
        # If vehicles are too close, emergency brake
        if current_distance < minimum_spacing:
            logger.info("Emergency brake - too close")
            return {
                'throttle': 0.0,
                'steer': 0.0,
                'brake': 1.0
            }
        
        # Calculate desired minimum gap using IDM
        desired_minimum_gap = minimum_spacing + (follow_speed * safe_time_headway) + \
            ((follow_speed * (follow_speed - lead_speed)) / 
             (2 * math.sqrt(max_acceleration * comfortable_deceleration)))
        
        # Calculate IDM acceleration with smoother response
        free_road_term = 1 - (follow_speed / desired_speed)**delta
        interaction_term = (desired_minimum_gap / current_distance)**2
        
        acceleration = max_acceleration * (free_road_term - interaction_term)
        
        # Convert acceleration to throttle/brake commands with smoother transitions
        if acceleration > 0:
            throttle = min(acceleration / max_acceleration, 0.6)  # Reduced max throttle
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-acceleration / comfortable_deceleration, 0.8)  # Reduced max brake
        
        # Calculate steering with improved stability
        dx = lead_location['x'] - follow_location['x']
        dy = lead_location['y'] - follow_location['y']
        angle_to_lead = math.atan2(dy, dx)
        
        # Calculate steering based on relative angle and current heading
        angle_diff = angle_to_lead - follow_yaw
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        # Proportional steering control with reduced gain
        kp_steer = 0.2  # Reduced for smoother steering
        steer = kp_steer * angle_diff
        steer = max(-0.5, min(0.5, steer))  # Reduced max steering angle
        
        # Log control values
        logger.info(f"IDM acceleration: {acceleration}")
        logger.info(f"Desired gap: {desired_minimum_gap}, Current gap: {current_distance}")
        logger.info(f"Control commands - Throttle: {throttle}, Steer: {steer}, Brake: {brake}")
        
        return {
            'throttle': float(throttle),
            'steer': float(steer),
            'brake': float(brake)
        }
        
    except Exception as e:
        logger.error(f"Error calculating control commands: {str(e)}")
        return {
            'throttle': 0.0,
            'steer': 0.0,
            'brake': 1.0
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
            lead_data = json.loads(response['Body'].read().decode('utf-8'))
            logger.info(f"Lead vehicle data: {json.dumps(lead_data)}")
            
            # Validate lead vehicle data
            if not all(key in lead_data for key in ['transform', 'velocity']):
                logger.error("Invalid lead vehicle data format")
                return {
                    'statusCode': 400,
                    'body': json.dumps('Invalid lead vehicle data format')
                }
            
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
            shadow_data = json.loads(shadow_response['payload'].read().decode('utf-8'))
            
            if 'state' not in shadow_data or 'reported' not in shadow_data['state']:
                logger.error("Invalid shadow data format")
                return {
                    'statusCode': 400,
                    'body': json.dumps('Invalid shadow data format')
                }
                
            follow_data = shadow_data['state']['reported']
            logger.info(f"Follow vehicle data: {json.dumps(follow_data)}")
            
            # Validate follow vehicle data
            if not all(key in follow_data for key in ['location', 'rotation']):
                logger.error("Invalid follow vehicle data format")
                return {
                    'statusCode': 400,
                    'body': json.dumps('Invalid follow vehicle data format')
                }
                
        except Exception as e:
            logger.error(f"Error getting follow vehicle data: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error getting follow vehicle data')
            }

        # Calculate control commands
        control_commands = calculate_control_commands(lead_data, follow_data)
        
        if control_commands is None:
            logger.error("Failed to calculate control commands")
            return {
                'statusCode': 500,
                'body': json.dumps('Failed to calculate control commands')
            }

        # Publish control commands to IoT topic
        logger.info(f"Publishing control commands: {json.dumps(control_commands)}")
        try:
            iot.publish(
                topic='carla/control/vehicle2',
                qos=1,
                payload=json.dumps(control_commands)
            )
        except Exception as e:
            logger.error(f"Error publishing control commands: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error publishing control commands')
            }

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