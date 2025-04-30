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
    """Calculate control commands for the following vehicle using improved PID-like control."""
    try:
        # Get configuration from environment variables
        desired_distance = float(os.environ.get('DESIRED_DISTANCE', '10.0'))
        max_acceleration = float(os.environ.get('MAX_ACCELERATION', '3.0'))
        max_speed = float(os.environ.get('MAX_SPEED', '30.0'))
        
        # Extract data from both vehicles
        lead_location = lead_data['transform']['location']
        follow_location = follow_data['location']
        lead_speed = lead_data['velocity']['speed']
        follow_speed = follow_data.get('velocity', {}).get('speed', 0.0)
        
        # Get vehicle headings
        lead_rotation = lead_data['transform']['rotation']
        follow_rotation = follow_data.get('rotation', {'yaw': 0.0})
        lead_yaw = math.radians(lead_rotation.get('yaw', 0.0))
        follow_yaw = math.radians(follow_rotation.get('yaw', 0.0))
        
        # Calculate current distance between vehicles
        current_distance = calculate_distance(lead_location['x'], lead_location['y'], 
                                           follow_location['x'], follow_location['y'])
        
        # If lead vehicle is stationary (speed < 0.1 m/s), stop the follower
        if lead_speed < 0.1:
            return {
                'throttle': 0.0,
                'steer': 0.0,
                'brake': 1.0
            }
        
        # Calculate relative angle to lead vehicle
        dx = lead_location['x'] - follow_location['x']
        dy = lead_location['y'] - follow_location['y']
        angle_to_lead = math.atan2(dy, dx)
        
        # Calculate steering based on relative angle and current heading
        # Normalize the angle difference to [-pi, pi]
        angle_diff = angle_to_lead - follow_yaw
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        # Improved PID-like steering control with deadband
        steer_deadband = 0.05  # Reduced deadband for better tracking
        if abs(angle_diff) < steer_deadband:
            steer = 0.0
        else:
            # Adjusted gains for better steering response
            kp_steer = 0.4  # Increased from 0.3
            kd_steer = 0.15  # Increased from 0.1
            steer = kp_steer * angle_diff + kd_steer * (angle_diff - angle_diff)  # Simple derivative term
        
        # Clamp steering
        steer = max(-1.0, min(1.0, steer))
        
        # Distance and speed control with improved PID
        distance_error = current_distance - desired_distance
        speed_diff = lead_speed - follow_speed
        
        # PID gains for longitudinal control
        kp_dist = 0.08  # Increased from 0.05
        ki_dist = 0.005  # Reduced from 0.01
        kd_dist = 0.03  # Increased from 0.02
        kp_speed = 0.15  # Increased from 0.1
        
        # Calculate longitudinal control with deadband
        longitudinal_deadband = 0.3  # Reduced from 0.5 for more responsive control
        if abs(distance_error) < longitudinal_deadband and abs(speed_diff) < 0.3:
            longitudinal_control = 0.0
        else:
            longitudinal_control = (kp_dist * distance_error + 
                                 ki_dist * distance_error +  # Simple integral term
                                 kd_dist * speed_diff +      # Derivative term
                                 kp_speed * speed_diff)
        
        # Calculate throttle and brake with smoother transitions
        if longitudinal_control > 0:
            throttle = min(1.0, longitudinal_control)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, -longitudinal_control)
            
        # Enhanced safety checks
        if current_distance < desired_distance * 0.7:  # Increased from 0.5 for earlier braking
            throttle = 0.0
            brake = 1.0
        
        if follow_speed > max_speed:  # Speed limiting
            throttle = 0.0
            
        # Additional safety: if too far behind, increase throttle
        if current_distance > desired_distance * 1.5:
            throttle = min(1.0, throttle + 0.2)
            brake = 0.0
            
        # Log all relevant values
        logger.info(f"Lead speed: {lead_speed}, Follow speed: {follow_speed}")
        logger.info(f"Current distance: {current_distance}, Desired distance: {desired_distance}")
        logger.info(f"Angle diff: {math.degrees(angle_diff)}, Steer: {steer}")
        logger.info(f"Distance error: {distance_error}, Speed diff: {speed_diff}")
        logger.info(f"Calculated commands - Throttle: {throttle}, Steer: {steer}, Brake: {brake}")
        
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
            'brake': 1.0  # Default to brake when there's an error
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