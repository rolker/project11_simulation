# Copyright 2025 Roland Arsenault
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(portsmouth_nh_gazebo)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# ---------- World generation at build time ----------

set(WORLD_NAME "portsmouth_harbor")
set(WORLD_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/config/portsmouth.yaml")
set(WORLD_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/worlds/${WORLD_NAME}")
set(WORLD_SDF "${WORLD_OUTPUT_DIR}/${WORLD_NAME}.sdf")

add_custom_command(
  OUTPUT "${WORLD_SDF}"
  COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_world.py"
    --config "${WORLD_CONFIG}"
    --output-dir "${WORLD_OUTPUT_DIR}"
  DEPENDS "${WORLD_CONFIG}"
  COMMENT "Generating ${WORLD_NAME} Gazebo world from chart data..."
)

add_custom_target(generate_world ALL DEPENDS "${WORLD_SDF}")

# ---------- Install ----------

# Generated world files (SDF, terrain model, textures)
install(
  DIRECTORY "${WORLD_OUTPUT_DIR}/"
  DESTINATION "share/${PROJECT_NAME}/worlds/${WORLD_NAME}"
)

# Launch files
install(
  DIRECTORY launch/
  DESTINATION "share/${PROJECT_NAME}/launch"
)

# Config (for reference / rebuild)
install(
  DIRECTORY config/
  DESTINATION "share/${PROJECT_NAME}/config"
)

# Environment hook: prepend worlds dir to GZ_SIM_RESOURCE_PATH
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
)

# ---------- Test ----------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
