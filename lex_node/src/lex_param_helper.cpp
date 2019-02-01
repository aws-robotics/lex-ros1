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

#include <aws/core/utils/logging/LogMacros.h>
#include <lex_node/lex_param_helper.h>

using Aws::Client::ParameterPath;


namespace Aws {
namespace Lex {

LexConfiguration LoadLexParameters(const Client::ParameterReaderInterface & parameter_interface)
{
  LexConfiguration lex_configuration;
  bool is_invalid = false;
  is_invalid |= static_cast<bool>(parameter_interface.ReadParam(ParameterPath(kBotAliasKey), lex_configuration.bot_alias));
  is_invalid |= static_cast<bool>(parameter_interface.ReadParam(ParameterPath(kBotNameKey), lex_configuration.bot_name));
  is_invalid |= static_cast<bool>(parameter_interface.ReadParam(ParameterPath(kUserIdKey), lex_configuration.user_id));
  if (is_invalid) {
    AWS_LOG_INFO(__func__, "Lex configuration not fully specified");
    throw std::invalid_argument("Lex configuration not fully specified");
  }
  return lex_configuration;
}

}  // namespace Lex
}  // namespace Aws
