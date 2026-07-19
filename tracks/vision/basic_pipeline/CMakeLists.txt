cmake_minimum_required(VERSION 3.20)
project(PerceptionPipeline LANGUAGES CXX)

# Require modern C++ standards for efficient memory management
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Automatically locate OpenCV on your Mac
find_package(OpenCV REQUIRED)

# 2. Locate ONNX Runtime (Homebrew path helper)
include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)

# Define your executable binary name and its source file
add_executable(perception_pipeline minimal_pipeline.cpp)

# 3. Bind the external libraries to your binary
target_link_libraries(perception_pipeline 
    PRIVATE 
    ${OpenCV_LIBS}
    onnxruntime
)
