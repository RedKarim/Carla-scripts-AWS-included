import json
import math
import boto3
import os
from datetime import datetime

# Initialize IoT client
iot_client = boto3.client('iot-data')

def calculate_control_commands(lead_data, follow_data=None):
    """
    Calculate control commands for the following vehicle based on lead vehicle data
    """
    # Get environment variables
    desired_distance = float(os.environ.get('DESIRED_DISTANCE', 5.0))
    max_acceleration = float(os.environ.get('MAX_ACCELERATION', 1.5))
    desired_speed = float(os.environ.get('DESIRED_SPEED', 15.0))
    follow_speed = float(os.environ.get('FOLLOW_SPEED', 12.0))
    time_headway = float(os.environ.get('TIME_HEADWAY', 1.0))
    comfort_decel = float(os.environ.get('COMFORT_DECEL', 2.5))
    
    # Extract lead vehicle data
    lead_velocity = lead_data['velocity']
    lead_speed = lead_velocity['speed']
    lead_location = lead_data['transform']['location']
    lead_rotation = lead_data['transform']['rotation']
    
    # Calculate desired speed based on lead vehicle speed
    target_speed = min(lead_speed, follow_speed)
    
    # Calculate acceleration based on speed difference
    speed_diff = target_speed - lead_speed
    acceleration = max(min(speed_diff, max_acceleration), -comfort_decel)
    
    # Calculate throttle and brake based on acceleration
    if acceleration > 0:
        throttle = min(acceleration / max_acceleration, 1.0)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(abs(acceleration) / comfort_decel, 1.0)
    
    # Calculate steering based on lead vehicle's yaw
    # This is a simplified model - in a real implementation, you'd want to consider
    # the actual path and relative positions of the vehicles
    steer = math.sin(math.radians(lead_rotation['yaw'])) * 0.5
    
    return {
        "throttle": throttle,
        "steer": steer,
        "brake": brake
    }

def lambda_handler(event, context):
    try:
        # Extract the GPS data from the S3 event
        s3_event = event['Records'][0]['s3']
        bucket_name = s3_event['bucket']['name']
        object_key = s3_event['object']['key']
        
        # Get the S3 client
        s3_client = boto3.client('s3')
        
        # Get the object from S3
        response = s3_client.get_object(Bucket=bucket_name, Key=object_key)
        gps_data = json.loads(response['Body'].read().decode('utf-8'))
        
        # Calculate control commands
        control_commands = calculate_control_commands(gps_data)
        
        # Publish control commands to IoT topic
        iot_client.publish(
            topic='carla/control/vehicle2',
            qos=1,
            payload=json.dumps(control_commands)
        )
        
        return {
            'statusCode': 200,
            'body': json.dumps({
                'message': 'Control commands published successfully',
                'commands': control_commands
            })
        }
        
    except Exception as e:
        print(f"Error processing GPS data: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps({
                'error': str(e)
            })
        } 