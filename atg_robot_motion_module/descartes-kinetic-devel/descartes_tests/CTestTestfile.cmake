# CMake generated Testfile for 
# Source directory: /home/conghui/diamond_ws_xavier/src/descartes-kinetic-devel/descartes_tests
# Build directory: /home/conghui/diamond_ws_xavier/src/descartes-kinetic-devel/descartes_tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_descartes_tests_gtest_descartes_tests_trajectory_utest "/home/conghui/diamond_ws_xavier/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/conghui/diamond_ws_xavier/test_results/descartes_tests/gtest-descartes_tests_trajectory_utest.xml" "--return-code" "/home/conghui/diamond_ws_xavier/devel/lib/descartes_tests/descartes_tests_trajectory_utest --gtest_output=xml:/home/conghui/diamond_ws_xavier/test_results/descartes_tests/gtest-descartes_tests_trajectory_utest.xml")
add_test(_ctest_descartes_tests_gtest_descartes_tests_planner_utest "/home/conghui/diamond_ws_xavier/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/conghui/diamond_ws_xavier/test_results/descartes_tests/gtest-descartes_tests_planner_utest.xml" "--return-code" "/home/conghui/diamond_ws_xavier/devel/lib/descartes_tests/descartes_tests_planner_utest --gtest_output=xml:/home/conghui/diamond_ws_xavier/test_results/descartes_tests/gtest-descartes_tests_planner_utest.xml")
add_test(_ctest_descartes_tests_rostest_test_moveit_launch_utest.launch "/home/conghui/diamond_ws_xavier/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/conghui/diamond_ws_xavier/test_results/descartes_tests/rostest-test_moveit_launch_utest.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/conghui/diamond_ws_xavier/src/descartes-kinetic-devel/descartes_tests --package=descartes_tests --results-filename test_moveit_launch_utest.xml --results-base-dir \"/home/conghui/diamond_ws_xavier/test_results\" /home/conghui/diamond_ws_xavier/src/descartes-kinetic-devel/descartes_tests/test/moveit/launch/utest.launch ")
