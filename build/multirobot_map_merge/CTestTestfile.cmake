# CMake generated Testfile for 
# Source directory: /home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge
# Build directory: /home/kev/ldlidar_ros2_ws/build/multirobot_map_merge
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_merging_pipeline "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/test_merging_pipeline.gtest.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_cmake_gtest/test_merging_pipeline.txt" "--command" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_merging_pipeline" "--gtest_output=xml:/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/test_merging_pipeline.gtest.xml")
set_tests_properties(test_merging_pipeline PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_merging_pipeline" TIMEOUT "60" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;154;ament_add_gtest;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/cppcheck.xunit.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/cppcheck.xunit.xml" "--include_dirs" "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/include" "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;164;ament_package;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
add_test(flake8 "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/flake8.xunit.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_flake8/flake8.txt" "--command" "/opt/ros/humble/bin/ament_flake8" "--xunit-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_flake8/cmake/ament_flake8.cmake;63;ament_add_test;/opt/ros/humble/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;18;ament_flake8;/opt/ros/humble/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;164;ament_package;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/lint_cmake.xunit.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;164;ament_package;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
add_test(pep257 "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/pep257.xunit.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_pep257/pep257.txt" "--command" "/opt/ros/humble/bin/ament_pep257" "--xunit-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;164;ament_package;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/uncrustify.xunit.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/humble/bin/ament_uncrustify" "--xunit-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;70;ament_add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;43;ament_uncrustify;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;164;ament_package;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/xmllint.xunit.xml" "--package-name" "multirobot_map_merge" "--output-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/kev/ldlidar_ros2_ws/build/multirobot_map_merge/test_results/multirobot_map_merge/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;164;ament_package;/home/kev/ldlidar_ros2_ws/src/m-explore-ros2/map_merge/CMakeLists.txt;0;")
subdirs("gtest")
