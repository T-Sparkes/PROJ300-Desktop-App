# Set minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Dynamically set the project name based on the folder name
get_filename_component(PROJECT_NAME ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# Set the source files directory
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)

# Add all .cpp files from the src and thirdparty directories
file(GLOB_RECURSE SOURCES ${SOURCE_DIR}/*.cpp ${THIRD_PARTY_DIR}/*.cpp)

# Create the main executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add include directories for the target
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${INCLUDE_DIR} 
    ${THIRD_PARTY_DIR}
    ${INCLUDE_DIR}/UI  
    ${INCLUDE_DIR}/Core 
)

target_link_directories(${PROJECT_NAME} PRIVATE 
    ${SOURCE_DIR} 
    ${THIRD_PARTY_DIR}
    ${SOURCE_DIR}/UI
    ${SOURCE_DIR}/Core  
)

# Link SDL3
find_package(SDL3 CONFIG REQUIRED) 
target_link_libraries(${PROJECT_NAME} PRIVATE SDL3::SDL3)

# Link Eigen3
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

# Link ImGui
find_package(imgui CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE imgui::imgui)

# this is heuristically generated, and may not be correct
find_package(implot CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE implot::implot)

# Automatically copy runtime libraries (DLLs)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMAND_EXPAND_LISTS
)

# Optionally print project details
message(STATUS "Project Name: ${PROJECT_NAME}")
message(STATUS "Project Version: ${PROJECT_VERSION}")