cmake_minimum_required(VERSION 2.8.3)
project(lex_common_msgs)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        audio_common_msgs
        message_generation
        )

add_message_files(
        DIRECTORY msg
        FILES
        KeyValue.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        AudioTextConversation.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
        DEPENDENCIES
        std_msgs
        audio_common_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)
