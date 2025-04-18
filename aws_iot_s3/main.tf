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

output "bucket_name" {
  value = aws_s3_bucket.gps_bucket.bucket
}
