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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <aws/core/Aws.h>
#include <aws/core/config/AWSProfileConfigLoader.h>
#include <aws/core/utils/HashingUtils.h>
#include <aws/core/utils/Outcome.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/DefaultLogSystem.h>

#include <aws/lex/LexRuntimeServiceClient.h>
#include <aws_common/sdk_utils/aws_error.h>

#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>

#include <lex_node/lex_node.h>
#include <ros/ros.h>

using testing::Return;
using testing::Invoke;
using testing::ElementsAreArray;
using testing::UnorderedElementsAreArray;
using testing::_;

using lex_common_msgs::AudioTextConversation;
using Aws::Lex::ErrorCode;

namespace lex_common_msgs
{
/**
 * @brief ros1 messages do not provide == operators
 */
bool operator==(const lex_common_msgs::KeyValue &left,
    const lex_common_msgs::KeyValue &right) {
  return left.key == right.key && left.value == right.value;
}
}  // namespace lex_common_msgs

class MockPostContentInterface : public Aws::Lex::PostContentInterface
{
 public:
  MOCK_METHOD2(PostContent,
      ErrorCode(const Aws::Lex::LexRequest & request, Aws::Lex::LexResponse & response));
};

typedef std::pair<std::string, std::string> SlotPair;

/**
 * Define a struct to convert between SlotPair and KeyValue.
 */
struct PairKeyValue
{
  SlotPair data;
  mutable lex_common_msgs::KeyValue key_value;

  explicit PairKeyValue(const SlotPair & data_pair)
  {
    data = data_pair;
  }

  operator lex_common_msgs::KeyValue & () const {
    key_value.key = data.first;
    key_value.value = data.second;
    return key_value;
  }
  operator SlotPair &() {
    return data;
  }
};

/**
 * Tests the creation of a Lex node instance with invalid parameters
 */
TEST(LexNodeSuite, BuildLexNodeWithEmptyParams)
{
  Aws::Lex::LexNode lex_node;
  ErrorCode error = lex_node.Init(nullptr);
  ASSERT_EQ(ErrorCode::INVALID_ARGUMENT, error);
}

/**
 * Spin up a lex node and initialize it with the mock_post_content.
 * Will Fail the running test should a timeout occur.
 *
 * @param will_succeed compares if the service call should succeed
 * @param mock_post_content for the lex node to call
 * @param test_request
 * @param test_result [out]
 *
 */
void ExecuteLexServiceTest(
    bool will_succeed,
    const std::shared_ptr<MockPostContentInterface> & mock_post_content,
    const std::shared_ptr<AudioTextConversation::Request> & test_request,
    std::shared_ptr<AudioTextConversation::Response> & test_result)
{
  auto lex_node = std::make_shared<Aws::Lex::LexNode>();

  Aws::Lex::ErrorCode error = lex_node->Init(mock_post_content);
  ASSERT_EQ(ErrorCode::SUCCESS, error);

  using ros::AsyncSpinner;
  AsyncSpinner executor(1);
  ros::Duration timeout(20);
  executor.start();
  ros::NodeHandle nh("~");
  auto client = nh.serviceClient<lex_common_msgs::AudioTextConversation>(
      "lex_conversation");
  client.waitForExistence(timeout);
  ASSERT_TRUE(client.exists()) << "Lex node service was not ready in time";
  AWS_LOG_INFO(__func__, "Sending lex request");
  ASSERT_EQ(will_succeed, client.call(*test_request, *test_result));
  executor.stop();
  AWS_LOG_INFO(__func__, "Lex request complete");
}

/**
 * Tests error code from lex node when there was a failure to post content.
 */
TEST(LexNodeSuite, TestLexServiceFailedPostContent)
{
  auto test_request = std::make_shared<AudioTextConversation::Request>();
  test_request->text_request = "text_request_test";
  test_request->content_type = "content_type_test";
  test_request->accept_type = "accept_type_test";
  test_request->audio_request.data = std::vector<std::uint8_t>{1, 2, 3};
  auto mock_post_content = std::make_shared<MockPostContentInterface>();
  EXPECT_CALL(*mock_post_content, PostContent(_, _))
      .WillOnce(Return(ErrorCode::FAILED_POST_CONTENT));
  auto result = std::make_shared<AudioTextConversation::Response>();
  ExecuteLexServiceTest(false, mock_post_content, test_request, result);
  // EXPECT_EQ(ErrorCode::FAILED_POST_CONTENT, (ErrorCode) result->error_code);
}

/**
 * Tests successful interaction with the lex node service.
 */
TEST(LexNodeSuite, TestLexServiceSuccess)
{
  ROS_DEBUG("Starting TestLexServiceSuccess");
  auto test_request = std::make_shared<AudioTextConversation::Request>();
  test_request->text_request = "text_request_test";
  test_request->content_type = "content_type_test";
  test_request->accept_type = "accept_type_test";
  test_request->audio_request.data = std::vector<std::uint8_t>{1, 2, 3};

  Aws::Lex::LexResponse test_response;
  test_response.text_response = "text_response_test";
  test_response.message_format_type = "message_format_type_test";
  test_response.intent_name = "intent_name_test";
  test_response.dialog_state = "dialog_state_test";
  test_response.audio_response = std::vector<std::uint8_t>{4, 5};
  test_response.session_attributes = "session_attributes_test";
  test_response.slots = {{"slot_1_key", "slot_1_value"}, {"slot_2_key", "slot_2_value"}};
  std::vector<PairKeyValue> slots;
  std::transform(test_response.slots.begin(), test_response.slots.end(), std::back_inserter(
      slots), [](const SlotPair &pair) {
    return PairKeyValue(pair);
  });
  auto mock_post_content = std::make_shared<MockPostContentInterface>();
  auto record_content = [&test_request, &test_response](const Aws::Lex::LexRequest & request,
                                                        Aws::Lex::LexResponse & response) -> ErrorCode {
    EXPECT_EQ(test_request->content_type, request.content_type);
    EXPECT_EQ(test_request->text_request, request.text_request);
    EXPECT_EQ(test_request->audio_request.data.size(), request.audio_request.size());
    EXPECT_THAT(request.audio_request, ElementsAreArray(test_request->audio_request.data));

    response.text_response = test_response.text_response;
    response.message_format_type = test_response.message_format_type;
    response.intent_name = test_response.intent_name;
    response.dialog_state = test_response.dialog_state;
    response.audio_response = test_response.audio_response;
    response.session_attributes = test_response.session_attributes;
    response.slots = test_response.slots;
    return ErrorCode::SUCCESS;
  };
  EXPECT_CALL(*mock_post_content, PostContent(_, _)).WillOnce(Invoke(record_content));
  auto result = std::make_shared<AudioTextConversation::Response>();
  ExecuteLexServiceTest(true, mock_post_content, test_request, result);

  EXPECT_EQ(test_response.text_response, result->text_response);
//  EXPECT_EQ(test_response.session_attributes, result->session_attributes);
  ASSERT_THAT(result->audio_response.data, ElementsAreArray(test_response.audio_response));
  EXPECT_EQ(test_response.dialog_state, result->dialog_state);
  EXPECT_EQ(test_response.intent_name, result->intent_name);
  EXPECT_EQ(test_response.message_format_type, result->message_format_type);
  ASSERT_THAT(result->slots, UnorderedElementsAreArray(slots));
}

/**
 * Interrupt handler to ensure rclcpp is shutdown.
 *
 * @param signum
 */
void h_sig_sigint(int signum)
{
  std::cout << "Signal interrupt" << std::endl;
  ros::shutdown();
}

int main(int argc, char ** argv) {
  signal(SIGINT, h_sig_sigint);
  ros::init(argc, argv, "test_node");
  Aws::Utils::Logging::InitializeAWSLogging(
      Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("test_node"));
  ::testing::InitGoogleMock(&argc, argv);
  auto result = RUN_ALL_TESTS();
  Aws::Utils::Logging::ShutdownAWSLogging();
  ros::shutdown();
  return result;
}