# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set target name
get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE test_files  ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
list(REMOVE_ITEM souce_files ${test_files} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cnpy/cnpy.cpp)
find_package(ZLIB REQUIRED)
find_library(T800_MNN_LIBRARY NAMES MNN REQUIRED)
add_library(t800_blink_native_npz_private STATIC third_party/cnpy/cnpy.cpp)
set_target_properties(t800_blink_native_npz_private PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
target_include_directories(t800_blink_native_npz_private PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cnpy)
target_link_libraries(t800_blink_native_npz_private PRIVATE ZLIB::ZLIB)

# Add target
add_library(${CURRENT_TARGET_NAME} SHARED)
add_library(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})

# Set source file of target
target_sources(${CURRENT_TARGET_NAME} PRIVATE ${souce_files})

# Set head files of target
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${head_files})

# Set include path of target
target_include_directories(${CURRENT_TARGET_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Set link libraries of target
target_link_libraries(${CURRENT_TARGET_NAME}
  PUBLIC
    yaml-cpp::yaml-cpp
    glog::glog
    fmt::fmt
    Eigen3::Eigen
    src::runner::basic
    src::common::parameter
    src::common::tool
)

# Set installation of target
install(TARGETS ${CURRENT_TARGET_NAME}
  EXPORT ${CURRENT_TARGET_ALIAS_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

find_package(OpenSSL REQUIRED)
target_link_libraries(${CURRENT_TARGET_NAME} PUBLIC OpenSSL::Crypto pthread)
# Native BlinkMimic runner: no Python or ONNX Runtime dependency.
target_compile_features(${CURRENT_TARGET_NAME} PUBLIC cxx_std_20)
target_compile_options(${CURRENT_TARGET_NAME} PRIVATE -fno-fast-math)
target_include_directories(${CURRENT_TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rl_tracking>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/cnpy>)
target_link_libraries(${CURRENT_TARGET_NAME} PUBLIC src::core::math
  PRIVATE t800_blink_native_npz_private ZLIB::ZLIB ${T800_MNN_LIBRARY})
# nlohmann/json v3.11.3 is header-only. The official ARM64 sysroot
# does not include its CMake package.
target_include_directories(${CURRENT_TARGET_NAME} SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
