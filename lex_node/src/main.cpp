/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <aws/core/Aws.h>
#include <aws/core/config/AWSProfileConfigLoader.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <lex_node/lex_node.h>
#include <ros/ros.h>

/**
 * Start the lex node program.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "lex_node");

  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("lex_node"));
  Aws::SDKOptions options;
  Aws::InitAPI(options);

  auto lex_node = Aws::Lex::BuildLexNode();
  AWS_LOG_INFO(__func__, "Starting Lex Node...");

  // blocking here, waiting until shutdown.
  ros::spin();

  AWS_LOG_INFO(__func__, "Shutting down Lex Node...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  Aws::ShutdownAPI(options);
  return 0;
}