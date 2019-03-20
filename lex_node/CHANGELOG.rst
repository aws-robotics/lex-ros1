^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lex_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Contributors: AAlon, M. M
