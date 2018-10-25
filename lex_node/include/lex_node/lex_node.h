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

#pragma once

#include <aws/lex/LexRuntimeServiceClient.h>
#include <lex_common_msgs/AudioTextConversation.h>
#include <lex_common_msgs/AudioTextConversationRequest.h>
#include <lex_common_msgs/AudioTextConversationResponse.h>
#include <lex_node/lex_param_helper.h>
#include <ros/ros.h>
#include <ros/spinner.h>

namespace Aws {
namespace Lex {

/**
 * Aws memory allocation tag.
 */
static const char * kAllocationTag = "lex";

class LexNode;

/**
 * Build a lex node for ros/aws use.
 *
 * @return a fully ros/aws configured lex node.
 */
LexNode BuildLexNode(std::shared_ptr<Client::ParameterReaderInterface> params = nullptr);

/**
 * LexNode is responsible for providing ROS API's and configuration for Amazon Lex.
 * The lex node will work on each incoming message serially and respond with the lex info.
 * @todo decide how the lex node will handle multiple requests.
 */
class LexNode
{
private:
  /**
   * The ros server for lex requests.
   */
  ros::ServiceServer lex_server_;

  /**
   * The Lex specific configuration for the amazon bot.
   */
  LexConfiguration lex_configuration_;

  /**
   * The lex runtime client to use for lex api calls.
   */
  std::shared_ptr<Aws::LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client_;

  /**
   * The ros node handle.
   */
  ros::NodeHandle node_handle_;

public:
  /**
   * Constructor.
   */
  LexNode();

  /**
   * Destructor.
   */
  ~LexNode() = default;

  /**
   * Initialize the lex node.
   */
  void Init();

  /**
   * Query if the service is in a valid state or not
   */
  bool IsServiceValid() { return (nullptr != static_cast<void *>(lex_server_)); }

  /**
   * Service callback for lex. Only allow one interaction with Lex at a time. If a new request comes
   * in, fail the last request, then make a new request.
   *
   * @param request to handle
   * @param response to fill
   * @return true if the service request was successful
   */
  bool LexServerCallback(lex_common_msgs::AudioTextConversationRequest & request,
                         lex_common_msgs::AudioTextConversationResponse & response);

  /**
   * Configure the lex node with lex client and config.
   *
   * @param lex_configuration message tags for lex calls
   * @param lex_runtime_client to call lex
   */
  void ConfigureAwsLex(
    LexConfiguration & lex_configuration,
    std::shared_ptr<Aws::LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client);

  /**
   * Return pointer to the Lex runtime client instance of this node
   *
   * @return pointer this node's Lex runtime client instance
   */
  std::weak_ptr<const Aws::LexRuntimeService::LexRuntimeServiceClient> GetLexRuntimeClient() const
  {
    return lex_runtime_client_;
  }

  /**
   * Conversion function since in ROS2, this class will inherit from Node.
   *
   * @return this functions node handle.
   */
  explicit operator ros::NodeHandle &() { return node_handle_; }
};

}  // namespace Lex
}  // namespace Aws
