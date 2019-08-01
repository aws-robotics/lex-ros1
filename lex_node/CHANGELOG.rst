^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lex_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2019-08-01)
------------------
* increment patch version (`#25 <https://github.com/aws-robotics/lex-ros1/issues/25>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Add gtest and gmock as test dependencies (`#17 <https://github.com/aws-robotics/lex-ros1/issues/17>`_)
  * Add gtest and gmock as test dependencies
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * modify lex_node to use add_rostest_gmock()
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Update to use non-legacy ParameterReader API (`#12 <https://github.com/aws-robotics/lex-ros1/issues/12>`_)
* Update to use new ParameterReader API (`#10 <https://github.com/aws-robotics/lex-ros1/issues/10>`_)
  * update lex_node_test to use the new ParameterReader API
  * increment major version number in package.xml
* Revert ParameterReader change (`#8 <https://github.com/aws-robotics/lex-ros1/issues/8>`_)
  * Revert "Updating lex_node_test.cpp to conform to the new ParameterReader API (`#6 <https://github.com/aws-robotics/lex-ros1/issues/6>`_)"
  This reverts commit 98b1fe112bcff0d01cf43620d17e4c956e3e9832.
  https://github.com/aws-robotics/utils-common/issues/15
* Updating lex_node_test.cpp to conform to the new ParameterReader API (`#6 <https://github.com/aws-robotics/lex-ros1/issues/6>`_)
  * update lex_node_test.cpp to conform to the new ParameterReader API
  * update lex_node to be compatible with newer aws sdk
  * use master branch for cloud extension dependencies
* Contributors: AAlon, M. M, Ross Desmond
