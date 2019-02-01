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
#include <aws/core/utils/HashingUtils.h>
#include <aws/core/utils/Outcome.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/DefaultLogSystem.h>
#include <aws/lex/LexRuntimeServiceClient.h>
#include <aws_common/sdk_utils/aws_error.h>
#include <gtest/gtest.h>
#include <lex_node/lex_configuration.h>
#include <lex_node/lex_node.h>
#include <ros/ros.h>

#define PARAM_NS_SEPARATOR "/"
#define PARAM_NS_SEPARATOR_CHAR '/'

using namespace Aws;
using namespace Aws::Client;

namespace Aws {
namespace Lex {

/**
 * Copy a result into an AudioTextConversionResponse.
 *
 * @param result to copy to the response
 * @param response [out] result copy
 * @return error code
 */
int CopyResult(const LexRuntimeService::Model::PostContentResult & result,
               lex_common_msgs::AudioTextConversationResponse & response);

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
  std::shared_ptr<const LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client);

}  // namespace Lex
}  // namespace Aws

class LexNodeSuite : public ::testing::Test
{
protected:
  LexNodeSuite()
  {
    options_.loggingOptions.logLevel = Utils::Logging::LogLevel::Trace;

    request_.content_type = "text/plain; charset=utf-8";
    request_.accept_type = "text/plain; charset=utf-8";
    request_.text_request = "make a reservation";

    configuration_.user_id = "test_user";
    configuration_.bot_name = "test_bot";
    configuration_.bot_alias = "superbot";
  }

  void SetUp() override
  {
    InitAPI(options_);
    Utils::Logging::InitializeAWSLogging(MakeShared<Utils::Logging::DefaultLogSystem>(
      "lex_node_test", Utils::Logging::LogLevel::Trace, "aws_sdk_"));
  }

  void TearDown() override
  {
    Utils::Logging::ShutdownAWSLogging();
    ShutdownAPI(options_);
  }

  SDKOptions options_;
  lex_common_msgs::AudioTextConversationRequest request_;
  Lex::LexConfiguration configuration_;
};

/**
 * Parameter reader that sets the output using provided std::mapS.
 */
class TestParameterReader : public ParameterReaderInterface
{
public:
  TestParameterReader() {}

  TestParameterReader(const std::string & user_id, const std::string & bot_name,
                      const std::string & bot_alias)
  {
    int_map_ = {{"aws_client_configuration/connect_timeout_ms", 9000},
                {"aws_client_configuration/request_timeout_ms", 9000}};
    string_map_ = {{Lex::kUserIdKey, user_id},
                   {Lex::kBotNameKey, bot_name},
                   {Lex::kBotAliasKey, bot_alias},
                   {"aws_client_configuration/region", "us-west-2"}};
  }

  AwsError ReadParam(const ParameterPath & param_path, int & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (int_map_.count(name) > 0) {
      out = int_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, bool & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::string & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (string_map_.count(name) > 0) {
      out = string_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }
  
  AwsError ReadParam(const ParameterPath & param_path, String & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (string_map_.count(name) > 0) {
      out = string_map_.at(name).c_str();
      result = AWS_ERR_OK;
    }
    return result;
  }
  
  AwsError ReadParam(const ParameterPath & param_path, std::map<std::string, std::string> & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }
  
  AwsError ReadParam(const ParameterPath & param_path, std::vector<std::string> & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, double & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }

private:
  std::string FormatParameterPath(const ParameterPath & param_path) const
  {
    return param_path.get_resolved_path(PARAM_NS_SEPARATOR_CHAR, PARAM_NS_SEPARATOR_CHAR);
  }

  std::map<std::string, int> int_map_;
  std::map<std::string, std::string> string_map_;
};

class MockLexClient : public LexRuntimeService::LexRuntimeServiceClient
{
public:
  MockLexClient(bool succeed = false) : succeed_(succeed) {}

  // MockLexClient(LexRuntimeService::Model::PostContentOutcome outcome) : outcome_(outcome) {}

  virtual LexRuntimeService::Model::PostContentOutcome PostContent(
    const LexRuntimeService::Model::PostContentRequest & request) const override
  {
    if (succeed_) {
      LexRuntimeService::Model::PostContentResult result;

      result.SetContentType("test_content_type");

      result.SetIntentName("test_intent_name");

      constexpr unsigned char slot_string[] =
        "{\"test_slots_key1\": \"test_slots_value1\", \"test_slots_key2\": \"test_slots_value2\"}";
      Utils::ByteBuffer slot_buffer(slot_string, sizeof(slot_string));
      auto slot_stdstring = Utils::HashingUtils::Base64Encode(slot_buffer);
      result.SetSlots(slot_stdstring);

      result.SetSessionAttributes("test_session_attributes");

      result.SetMessage("test_message");

      result.SetMessageFormat(LexRuntimeService::Model::MessageFormatType::CustomPayload);

      result.SetDialogState(LexRuntimeService::Model::DialogState::Failed);

      result.SetSlotToElicit("test_active_slot");

      std::stringstream * audio_data = New<std::stringstream>("test");
      *audio_data << "blah blah blah";
      result.ReplaceBody(audio_data);

      return LexRuntimeService::Model::PostContentOutcome(std::move(result));
    } else {
      return LexRuntimeService::Model::PostContentOutcome(
        AWSError<LexRuntimeService::LexRuntimeServiceErrors>());
    }
  }

private:
  bool succeed_;
};

/**
 * Tests the creation of a Lex node instance with invalid parameters
 */
TEST_F(LexNodeSuite, BuildLexNodeWithEmptyParams)
{
  auto param_reader = std::make_shared<TestParameterReader>();

  try {
    auto lex_node = Lex::BuildLexNode(param_reader);
    FAIL() << "Expected std::invalid_argument exception to be thrown";
  } catch (const std::invalid_argument & err) {
    EXPECT_STREQ(err.what(), "Lex configuration not fully specified");
  } catch (...) {
    FAIL() << "Expected std::invalid_argument exception to be thrown";
  }
}

/**
 * Tests the creation of a Lex node instance with valid parameters
 */
TEST_F(LexNodeSuite, BuildLexNodeWithParams)
{
  auto param_reader = std::make_shared<TestParameterReader>(
    configuration_.user_id, configuration_.bot_name, configuration_.bot_alias);

  try {
    auto lex_node = Lex::BuildLexNode(param_reader);
    EXPECT_TRUE(lex_node.IsServiceValid());
  } catch (const std::exception & err) {
    FAIL() << "Unexpected error \"" << err.what() << "\" thrown";
  } catch (...) {
    FAIL() << "Unexpected exception thrown";
  }
}

/**
 * Test the result of PostContent() when the Lex runtime client fails to PostContent()
 */
TEST_F(LexNodeSuite, LexNodePostContentFail)
{
  auto param_reader = std::make_shared<TestParameterReader>(
    configuration_.user_id, configuration_.bot_name, configuration_.bot_alias);
  auto lex_node = Lex::BuildLexNode(param_reader);
  ASSERT_TRUE(lex_node.IsServiceValid());

  auto lex_runtime_client = std::make_shared<MockLexClient>(false);

  lex_common_msgs::AudioTextConversationResponse response;
  bool success = PostContent(request_, response, configuration_, lex_runtime_client);
  EXPECT_FALSE(success);

  // check that the response hasn't been filled because PostContent() was supposed to have failed
  EXPECT_TRUE(response.text_response.empty());
  EXPECT_TRUE(response.audio_response.data.empty());
  EXPECT_TRUE(response.slots.empty());
  EXPECT_TRUE(response.intent_name.empty());
  EXPECT_TRUE(response.message_format_type.empty());
  EXPECT_TRUE(response.dialog_state.empty());
}

/**
 * Test the result of PostContent() when the Lex runtime client fails to PostContent()
 */
TEST_F(LexNodeSuite, LexNodePostContentSucceed)
{
  auto param_reader = std::make_shared<TestParameterReader>(
    configuration_.user_id, configuration_.bot_name, configuration_.bot_alias);
  auto lex_node = Lex::BuildLexNode(param_reader);
  ASSERT_TRUE(lex_node.IsServiceValid());

  auto lex_runtime_client = std::make_shared<MockLexClient>(true);

  lex_common_msgs::AudioTextConversationResponse response;
  bool success = PostContent(request_, response, configuration_, lex_runtime_client);
  EXPECT_TRUE(success);

  // check that the response hasn't been filled because PostContent() was supposed to have failed
  EXPECT_EQ(response.text_response, "test_message");
  EXPECT_TRUE(0 == memcmp(response.audio_response.data.data(), "blah blah blah", 14));
  EXPECT_EQ(response.slots.size(), 2);
  EXPECT_EQ(response.slots[0].key, "test_slots_key1");
  EXPECT_EQ(response.slots[0].value, "test_slots_value1");
  EXPECT_EQ(response.slots[1].key, "test_slots_key2");
  EXPECT_EQ(response.slots[1].value, "test_slots_value2");
  EXPECT_EQ(response.intent_name, "test_intent_name");
  EXPECT_EQ(response.message_format_type, "CustomPayload");
  EXPECT_EQ(response.dialog_state, "Failed");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lex_node");
  return RUN_ALL_TESTS();
}
