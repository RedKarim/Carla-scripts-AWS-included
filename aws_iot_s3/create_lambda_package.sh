#!/bin/bash

# Create a temporary directory
mkdir -p lambda_package

# Copy the Lambda function code
cp ../lambda_function.py lambda_package/

# Install required dependencies
pip install boto3 -t lambda_package/

# Create the zip file
cd lambda_package
zip -r ../function.zip .
cd ..

# Clean up
rm -rf lambda_package

echo "Lambda package created: function.zip" 