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

#include <string>

namespace Aws {
namespace Lex {

/**
 * \defgroup ROS parameter keys for lex configuration.
 */
/**@{*/
#define LEX_CONFIGURATION_PATH "lex_configuration/"

constexpr char kUserIdKey[] = LEX_CONFIGURATION_PATH "user_id";
constexpr char kBotNameKey[] = LEX_CONFIGURATION_PATH "bot_name";
constexpr char kBotAliasKey[] = LEX_CONFIGURATION_PATH "bot_alias";
/** @}*/

/**
 * Configuration to make calls to lex.
 */
struct LexConfiguration
{
  /**
   * The user id to call lex with. Unique per caller.
   */
  std::string user_id;

  /**
   * The lex bot to use.
   */
  std::string bot_name;

  /**
   * The lex alias of the bot to use.
   */
  std::string bot_alias;
};

}  // namespace Lex
}  // namespace Aws
