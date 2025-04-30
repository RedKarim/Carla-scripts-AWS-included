provider "aws" {
  region = "ap-northeast-1"
}

resource "random_id" "bucket_suffix" {
  byte_length = 4
}

resource "aws_s3_bucket" "gps_bucket" {
  bucket = "carla-gps-data-${random_id.bucket_suffix.hex}"
  force_destroy = true
}

resource "aws_iam_role" "iot_s3_role" {
  name = "iot-to-s3-role"

  assume_role_policy = jsonencode({
    Version = "2012-10-17",
    Statement = [
      {
        Effect = "Allow",
        Principal = {
          Service = "iot.amazonaws.com"
        },
        Action = "sts:AssumeRole"
      }
    ]
  })
}

resource "aws_iam_role_policy" "iot_s3_policy" {
  name = "iot-s3-write-policy"
  role = aws_iam_role.iot_s3_role.id

  policy = jsonencode({
    Version = "2012-10-17",
    Statement = [
      {
        Effect = "Allow",
        Action = [
          "s3:PutObject"
        ],
        Resource = "${aws_s3_bucket.gps_bucket.arn}/*"
      }
    ]
  })
}

resource "aws_iot_topic_rule" "iot_to_s3" {
  name        = "CarlaGpsToS3"
  sql         = "SELECT * FROM 'carla/gps'"
  sql_version = "2016-03-23"
  enabled     = true

  s3 {
    role_arn    = aws_iam_role.iot_s3_role.arn
    bucket_name = aws_s3_bucket.gps_bucket.bucket
    key         = "${timestamp()}.json"
  }
}

# Lambda function configuration
resource "aws_iam_role" "lambda_role" {
  name = "carla-platooning-lambda-role"

  assume_role_policy = jsonencode({
    Version = "2012-10-17",
    Statement = [
      {
        Effect = "Allow",
        Principal = {
          Service = "lambda.amazonaws.com"
        },
        Action = "sts:AssumeRole"
      }
    ]
  })
}

resource "aws_iam_role_policy" "lambda_policy" {
  name = "carla-platooning-lambda-policy"
  role = aws_iam_role.lambda_role.id

  policy = jsonencode({
    Version = "2012-10-17",
    Statement = [
      {
        Effect = "Allow",
        Action = [
          "logs:CreateLogGroup",
          "logs:CreateLogStream",
          "logs:PutLogEvents"
        ],
        Resource = "arn:aws:logs:*:*:*"
      },
      {
        Effect = "Allow",
        Action = [
          "s3:ListBucket"
        ],
        Resource = aws_s3_bucket.gps_bucket.arn
      },
      {
        Effect = "Allow",
        Action = [
          "s3:GetObject"
        ],
        Resource = "${aws_s3_bucket.gps_bucket.arn}/*"
      },
      {
        Effect = "Allow",
        Action = [
          "iot:Publish"
        ],
        Resource = "*"
      }
    ]
  })
}

# Create Lambda function
resource "aws_lambda_function" "platooning_lambda" {
  filename         = "../function.zip"
  function_name    = "carla-platooning-lambda"
  role             = aws_iam_role.lambda_role.arn
  handler          = "lambda_function.lambda_handler"
  runtime          = "python3.9"
  timeout          = 30
  memory_size      = 128

  environment {
    variables = {
      DESIRED_DISTANCE = "5.0"
      MAX_ACCELERATION = "1.5"
      DESIRED_SPEED = "15.0"
      FOLLOW_SPEED = "12.0"
      TIME_HEADWAY = "1.0"
      COMFORT_DECEL = "2.5"
    }
  }
}

# S3 trigger for Lambda
resource "aws_lambda_permission" "allow_s3" {
  statement_id  = "AllowS3Invoke"
  action        = "lambda:InvokeFunction"
  function_name = aws_lambda_function.platooning_lambda.function_name
  principal     = "s3.amazonaws.com"
  source_arn    = aws_s3_bucket.gps_bucket.arn
}

resource "aws_s3_bucket_notification" "bucket_notification" {
  bucket = aws_s3_bucket.gps_bucket.id

  lambda_function {
    lambda_function_arn = aws_lambda_function.platooning_lambda.arn
    events              = ["s3:ObjectCreated:*"]
    filter_suffix       = ".json"
  }

  depends_on = [aws_lambda_permission.allow_s3]
}

output "bucket_name" {
  value = aws_s3_bucket.gps_bucket.bucket
}

output "lambda_function_name" {
  value = aws_lambda_function.platooning_lambda.function_name
}

output "lambda_function_arn" {
  value = aws_lambda_function.platooning_lambda.arn
}
