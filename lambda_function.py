import json
import math
import boto3
import os
import logging
from datetime import datetime
from urllib.parse import unquote_plus
import time

# Configure logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# Initialize IoT client
iot_client = boto3.client('iot-data')

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_control_commands(lead_data, follow_data):
    """Calculate control commands using an advanced car-following model with predictive elements."""
    try:
        # Core IDM Parameters
        desired_speed = float(os.environ.get('MAX_SPEED', '15.0'))  # Base desired speed
        safe_time_headway = 2.0  # Base time headway
        max_acceleration = float(os.environ.get('MAX_ACCELERATION', '1.5'))  # Base acceleration
        comfortable_deceleration = 3.0  # Base deceleration
        minimum_spacing = 8.0  # Base minimum spacing
        delta = 4.0  # acceleration exponent
        
        # Advanced Safety Parameters
        emergency_braking_distance = 5.0  # Distance for emergency braking
        critical_distance = 3.0  # Distance for full emergency stop
        relative_speed_threshold = 5.0  # m/s relative speed threshold
        prediction_horizon = 1.0  # seconds to look ahead
        curve_detection_threshold = math.radians(15)  # Minimum angle for curve detection
        
        # Extract and validate vehicle data
        if not all(key in lead_data.get('transform', {}) for key in ['location', 'rotation']):
            logger.error("Invalid lead vehicle transform data")
            return {'throttle': 0.0, 'steer': 0.0, 'brake': 1.0}
            
        if not all(key in follow_data for key in ['location', 'rotation', 'velocity']):
            logger.error("Invalid follow vehicle data")
            return {'throttle': 0.0, 'steer': 0.0, 'brake': 1.0}
        
        # Get current states
        lead_location = lead_data['transform']['location']
        follow_location = follow_data['location']
        lead_speed = lead_data['velocity']['speed']
        follow_speed = follow_data['velocity']['speed']
        lead_rotation = lead_data['transform']['rotation']
        follow_rotation = follow_data['rotation']
        
        # Calculate relative states
        relative_speed = follow_speed - lead_speed
        current_distance = calculate_distance(lead_location['x'], lead_location['y'], 
                                           follow_location['x'], follow_location['y'])
        
        # Calculate relative heading and path curvature
        lead_yaw = math.radians(lead_rotation.get('yaw', 0.0))
        follow_yaw = math.radians(follow_rotation.get('yaw', 0.0))
        heading_diff = abs(lead_yaw - follow_yaw)
        if heading_diff > math.pi:
            heading_diff = 2 * math.pi - heading_diff
        
        # Scenario Detection and Adaptive Parameters
        is_curve = heading_diff > curve_detection_threshold
        is_approaching = relative_speed > 0 and current_distance < emergency_braking_distance
        is_accelerating = lead_speed > follow_speed and current_distance > minimum_spacing
        
        # Adaptive Parameters based on Scenario
        if is_curve:
            desired_speed *= 0.7  # Reduce speed in curves
            safe_time_headway *= 1.2  # Increase headway in curves
            minimum_spacing *= 1.2  # Increase spacing in curves
            
        if is_approaching:
            desired_speed = min(desired_speed, lead_speed)
            safe_time_headway *= 1.3  # Increase headway when approaching
            max_acceleration *= 0.7  # Reduce acceleration when approaching
            
        if is_accelerating:
            max_acceleration *= 1.2  # Allow more aggressive acceleration when safe
            
        # Predictive Safety Checks
        predicted_distance = current_distance + relative_speed * prediction_horizon
        predicted_relative_speed = relative_speed + (lead_speed - follow_speed) * prediction_horizon
        
        # Emergency Conditions
        if current_distance < critical_distance or predicted_distance < critical_distance:
            logger.info("Critical distance - full emergency stop")
            return {
                'throttle': 0.0,
                'steer': 0.0,
                'brake': 1.0
            }
            
        if (current_distance < emergency_braking_distance and 
            (relative_speed > relative_speed_threshold or predicted_relative_speed > relative_speed_threshold)):
            logger.info("Emergency braking - approaching too fast")
            return {
                'throttle': 0.0,
                'steer': 0.0,
                'brake': 1.0
            }
        
        # Enhanced IDM with Predictive Elements
        # Calculate desired minimum gap with dynamic adjustment
        base_gap = minimum_spacing + (follow_speed * safe_time_headway)
        relative_speed_gap = ((follow_speed * (follow_speed - lead_speed)) / 
                            (2 * math.sqrt(max_acceleration * comfortable_deceleration)))
        curve_gap = minimum_spacing * 0.2 if is_curve else 0.0
        desired_minimum_gap = base_gap + relative_speed_gap + curve_gap
        
        # Add predictive gap adjustment
        if predicted_relative_speed > 0:
            desired_minimum_gap *= (1 + predicted_relative_speed / 10.0)
        
        # Enhanced IDM Acceleration Calculation
        free_road_term = 1 - (follow_speed / desired_speed)**delta
        interaction_term = (desired_minimum_gap / current_distance)**2
        relative_speed_term = 0.5 * (relative_speed / desired_speed)**2
        curve_term = 0.3 if is_curve else 0.0
        
        acceleration = max_acceleration * (free_road_term - interaction_term - relative_speed_term - curve_term)
        
        # Convert acceleration to control commands with enhanced safety
        if acceleration > 0:
            # Distance-based throttle reduction
            distance_factor = min(1.0, current_distance / (2 * minimum_spacing))
            # Speed-based throttle reduction
            speed_factor = 1.0 - (follow_speed / desired_speed) if follow_speed > desired_speed else 1.0
            # Curve-based throttle reduction
            curve_factor = 1.0 - (heading_diff / math.pi) if is_curve else 1.0
            
            throttle = min(acceleration / max_acceleration, 0.6) * distance_factor * speed_factor * curve_factor
            brake = 0.0
        else:
            throttle = 0.0
            # Enhanced brake response
            brake_factor = 1.0 + (abs(relative_speed) / relative_speed_threshold)
            brake = min(-acceleration / comfortable_deceleration, 0.9) * brake_factor
        
        # Advanced Steering Control
        dx = lead_location['x'] - follow_location['x']
        dy = lead_location['y'] - follow_location['y']
        angle_to_lead = math.atan2(dy, dx)
        
        # Calculate steering based on relative angle and current heading
        angle_diff = angle_to_lead - follow_yaw
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        # Dynamic steering gain based on multiple factors
        base_kp_steer = 0.2
        speed_factor = max(0.5, min(1.0, follow_speed / desired_speed))
        distance_factor = min(1.0, current_distance / minimum_spacing)
        curve_factor = 1.2 if is_curve else 1.0
        kp_steer = base_kp_steer * speed_factor * distance_factor * curve_factor
        
        # Add predictive steering component
        predicted_angle = angle_diff + (lead_yaw - follow_yaw) * prediction_horizon
        steer = kp_steer * (0.7 * angle_diff + 0.3 * predicted_angle)
        steer = max(-0.4, min(0.4, steer))
        
        # Log detailed state information
        logger.info(f"Scenario: curve={is_curve}, approaching={is_approaching}, accelerating={is_accelerating}")
        logger.info(f"Lead speed: {lead_speed}, Follow speed: {follow_speed}")
        logger.info(f"Current distance: {current_distance}, Predicted distance: {predicted_distance}")
        logger.info(f"Relative speed: {relative_speed}, Predicted relative speed: {predicted_relative_speed}")
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
        start_time = time.time()
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
            s3_start_time = time.time()
            response = s3.get_object(Bucket=bucket, Key=key)
            lead_data = json.loads(response['Body'].read().decode('utf-8'))
            s3_latency = (time.time() - s3_start_time) * 1000
            logger.info(f"S3 read latency: {s3_latency:.2f} ms")
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
            shadow_start_time = time.time()
            shadow_response = iot.get_thing_shadow(
                thingName='vehicle2'
            )
            shadow_data = json.loads(shadow_response['payload'].read().decode('utf-8'))
            shadow_latency = (time.time() - shadow_start_time) * 1000
            logger.info(f"Shadow read latency: {shadow_latency:.2f} ms")
            
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
        control_start_time = time.time()
        control_commands = calculate_control_commands(lead_data, follow_data)
        control_latency = (time.time() - control_start_time) * 1000
        logger.info(f"Control calculation latency: {control_latency:.2f} ms")
        
        if control_commands is None:
            logger.error("Failed to calculate control commands")
            return {
                'statusCode': 500,
                'body': json.dumps('Failed to calculate control commands')
            }

        # Add timestamp to control commands
        control_commands['timestamp'] = time.time()
        
        # Publish control commands to IoT topic
        logger.info(f"Publishing control commands: {json.dumps(control_commands)}")
        try:
            publish_start_time = time.time()
            iot.publish(
                topic='carla/control/vehicle2',
                qos=1,
                payload=json.dumps(control_commands)
            )
            publish_latency = (time.time() - publish_start_time) * 1000
            logger.info(f"Publish latency: {publish_latency:.2f} ms")
        except Exception as e:
            logger.error(f"Error publishing control commands: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error publishing control commands')
            }

        # Calculate total latency
        total_latency = (time.time() - start_time) * 1000
        logger.info(f"Total Lambda execution time: {total_latency:.2f} ms")

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