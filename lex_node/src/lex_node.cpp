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
#include <aws/core/utils/HashingUtils.h>
#include <aws/core/utils/Outcome.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/core/utils/memory/stl/AWSString.h>
#include <aws/lex/model/PostContentRequest.h>
#include <aws/lex/model/PostContentResult.h>
#include <aws/lex/model/PostTextRequest.h>
#include <aws/lex/model/PostTextResult.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <lex_common_msgs/KeyValue.h>
#include <lex_node/lex_node.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <regex>

namespace Aws {
namespace Lex {

std::ostream & operator<<(std::ostream & os,
                          const Aws::LexRuntimeService::Model::PostContentRequest & request)
{
  os << "Request: " << std::endl;
  os << "Bot Alias: " << request.GetBotAlias() << std::endl;
  os << "Bot Name : " << request.GetBotName() << std::endl;
  std::stringstream ss;
  ss << request.GetBody()->rdbuf();
  os << "Input data: " << ss.str() << std::endl;
  os << "User Id: " << request.GetUserId() << std::endl;
  os << "Accept Type: " << request.GetAccept() << std::endl;
  os << "Content Type: " << request.GetContentType() << std::endl;
  return os;
}

std::ostream & operator<<(std::ostream & os,
                          const Aws::LexRuntimeService::Model::PostContentResult & result)
{
  os << "PostContentResult: " << std::endl;
  os << "Message: " << result.GetMessage() << std::endl;
  os << "Slot to elicit: " << result.GetSlotToElicit() << std::endl;
  using Aws::LexRuntimeService::Model::DialogStateMapper::GetNameForDialogState;
  using Aws::LexRuntimeService::Model::MessageFormatTypeMapper::GetNameForMessageFormatType;
  os << "Dialog State: " << GetNameForDialogState(result.GetDialogState()) << std::endl;
  os << "Message format type: " << GetNameForMessageFormatType(result.GetMessageFormat())
     << std::endl;
  os << "Slots: " << result.GetSlots() << std::endl;
  os << "Session Attributes: " << result.GetSessionAttributes() << std::endl;
  os << "Content Type: " << result.GetContentType() << std::endl;
  os << "Intent Name: " << result.GetIntentName() << std::endl;
  return os;
}

/**
 * Copy a result into an AudioTextConversionResponse.
 *
 * @param result to copy to the response
 * @param response [out] result copy
 * @return error code
 */
int CopyResult(Aws::LexRuntimeService::Model::PostContentResult & result,
               lex_common_msgs::AudioTextConversationResponse & response)
{
  using Aws::LexRuntimeService::Model::MessageFormatTypeMapper::GetNameForMessageFormatType;
  response.message_format_type = GetNameForMessageFormatType(result.GetMessageFormat()).c_str();
  response.text_response = result.GetMessage().c_str();

  std::streampos audio_size = result.GetAudioStream().seekg(0, std::ios_base::end).tellg();
  response.audio_response.data = std::vector<uint8_t>(audio_size);
  result.GetAudioStream().seekg(0, std::ios_base::beg);
  result.GetAudioStream().read(reinterpret_cast<char *>(&response.audio_response.data[0]),
                               audio_size);

  response.intent_name = result.GetIntentName().c_str();
  using Aws::LexRuntimeService::Model::DialogStateMapper::GetNameForDialogState;
  response.dialog_state = GetNameForDialogState(result.GetDialogState()).c_str();
  std::string session_attributes = result.GetSessionAttributes().c_str();
  auto slot_byte_buffer = Aws::Utils::HashingUtils::Base64Decode(result.GetSlots().c_str());
  Aws::String slot_string(reinterpret_cast<char *>(slot_byte_buffer.GetUnderlyingData()),
                          slot_byte_buffer.GetLength());
  auto slot_json = Aws::Utils::Json::JsonValue(slot_string);
  if (slot_json.WasParseSuccessful()) {
    AWS_LOGSTREAM_DEBUG(__func__, "slot_json: " << slot_string);

    auto view = slot_json.GetAllObjects();
    response.slots = std::vector<lex_common_msgs::KeyValue>(view.size());
    auto response_it = response.slots.begin();
    for (auto & element : view) {
      response_it->key = element.first.c_str();
      response_it->value = element.second.AsString().c_str();
      response_it++;
    }
  } else {
    AWS_LOGSTREAM_WARN(__func__, "Unable to parse slot string " << slot_string);
  }
  return 0;
}

/**
 * Post content to lex given an audio text conversation request and respond to it.
 * Configures the call with the lex configuration and lex_runtime_client.
 *
 * @param request to populate the lex call with
 * @param response to fill with data received by lex
 * @param lex_configuration to specify bot, and response type
 * @param lex_runtime_client to call lex with
 * @return true if the call succeeded, false otherwise
 */
bool PostContent(
  lex_common_msgs::AudioTextConversationRequest & request,
  lex_common_msgs::AudioTextConversationResponse & response,
  const LexConfiguration & lex_configuration,
  std::shared_ptr<const LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client)
{
  Aws::LexRuntimeService::Model::PostContentRequest post_content_request;
  post_content_request.WithBotAlias(lex_configuration.bot_alias.c_str())
    .WithBotName(lex_configuration.bot_name.c_str())
    .WithAccept(request.accept_type.c_str())
    .WithUserId(lex_configuration.user_id.c_str());

  post_content_request.SetContentType(request.content_type.c_str());
  auto io_stream = Aws::MakeShared<Aws::StringStream>(kAllocationTag);

  if (!request.audio_request.data.empty()) {
    std::copy(request.audio_request.data.begin(), request.audio_request.data.end(),
              std::ostream_iterator<unsigned char>(*io_stream));
  } else {
    *io_stream << request.text_request;
  }
  post_content_request.SetBody(io_stream);
  AWS_LOGSTREAM_DEBUG(__func__, "PostContentRequest " << post_content_request);
  auto post_content_result = lex_runtime_client->PostContent(post_content_request);
  bool is_valid = true;
  if (post_content_result.IsSuccess()) {
    auto & result = post_content_result.GetResult();
    AWS_LOGSTREAM_DEBUG(__func__, "PostContentResult succeeded: " << result.GetMessage());
    // @todo: use response variable for errors.
    /* auto error_code = */ CopyResult(result, response);
    // if (error_code) {
    //    is_valid = false;
    // }
  } else {
    is_valid = false;
    AWS_LOGSTREAM_ERROR(
      __func__, "PostContentResult failed: " << post_content_result.GetError().GetMessage());
  }
  return is_valid;
}

LexNode BuildLexNode(std::shared_ptr<Client::ParameterReaderInterface> params)
{
  LexNode lex_node;
  if (nullptr == params) {
    params = std::make_shared<Client::Ros1NodeParameterReader>();
  }
  auto lex_configuration = LoadLexParameters(*params);
  Client::ClientConfigurationProvider configuration_provider(params);
  auto lex_runtime_client = Aws::MakeShared<Aws::LexRuntimeService::LexRuntimeServiceClient>(
    kAllocationTag, configuration_provider.GetClientConfiguration());
  lex_node.ConfigureAwsLex(lex_configuration, lex_runtime_client);
  lex_node.Init();
  return lex_node;
}

LexNode::LexNode() : node_handle_("~") {}

void LexNode::Init()
{
  lex_server_ =
    node_handle_.advertiseService<>("lex_conversation", &LexNode::LexServerCallback, this);
}

void LexNode::ConfigureAwsLex(
  LexConfiguration & lex_configuration,
  std::shared_ptr<Aws::LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client)
{
  lex_configuration_ = lex_configuration;
  lex_runtime_client_ = lex_runtime_client;
}

bool LexNode::LexServerCallback(lex_common_msgs::AudioTextConversationRequest & request,
                                lex_common_msgs::AudioTextConversationResponse & response)
{
  if (!lex_runtime_client_) {
    // @todo define a new exception
    AWS_LOG_WARN(__func__, "Lex runtime client is not initialized, LoadConfiguration.");
    throw std::invalid_argument("Lex runtime client is not initialized, LoadConfiguration.");
  }
  return PostContent(request, response, lex_configuration_, lex_runtime_client_);
}

}  // namespace Lex
}  // namespace Aws
