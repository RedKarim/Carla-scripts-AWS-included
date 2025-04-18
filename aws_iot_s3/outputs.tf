output "iot_topic_rule_name" {
  value = aws_iot_topic_rule.iot_to_s3.name
}

output "iot_s3_role_arn" {
  value = aws_iam_role.iot_s3_role.arn
}
