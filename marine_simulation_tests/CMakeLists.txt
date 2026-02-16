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
  add_launch_test(test/test_goto_waypoint.py
    TIMEOUT 120
  )
  add_launch_test(test/test_survey_line_set.py
    TIMEOUT 300
  )
endif()

ament_package()
