cmake_minimum_required(VERSION 3.5.1)

project(marine_simulation_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Each test gets a unique ROS_DOMAIN_ID to prevent DDS cross-talk
  # when CTest runs them sequentially. Without isolation, lingering
  # FastDDS shared-memory segments from one test interfere with the next.
  add_launch_test(test/test_goto_waypoint.py
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=51
  )
  add_launch_test(test/test_survey_line_set.py
    TIMEOUT 300
    ENV ROS_DOMAIN_ID=52
  )
endif()

ament_package()
