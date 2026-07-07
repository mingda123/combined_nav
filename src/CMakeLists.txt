include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(IMU)
add_subdirectory(COMMON)
add_subdirectory(GNSS)
add_subdirectory(COMBINED)

add_executable(forward_LCI forward_LCI.cpp)
target_include_directories(forward_LCI
    PUBLIC
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/GNSS
    ${CMAKE_SOURCE_DIR}/src/COMBINED
    ${CMAKE_SOURCE_DIR}/src/COMMON
)
target_link_libraries(forward_LCI
    PUBLIC
    imu
    gnss
    combined_module
    simpleini
)

add_executable(backward_LCI backward_LCI.cpp)
target_include_directories(backward_LCI
    PUBLIC
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/GNSS
    ${CMAKE_SOURCE_DIR}/src/COMBINED
    ${CMAKE_SOURCE_DIR}/src/COMMON
)
target_link_libraries(backward_LCI
    PUBLIC
    imu
    gnss
    combined_module
    simpleini
)

add_executable(fused_LCI fused_LCI.cpp)
target_include_directories(fused_LCI
    PUBLIC
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/GNSS
    ${CMAKE_SOURCE_DIR}/src/COMBINED
    ${CMAKE_SOURCE_DIR}/src/COMMON
)
target_link_libraries(fused_LCI
    PUBLIC
    imu
    gnss
    combined_module
    simpleini
)

add_executable(realtime_forward_LCI
    realtime_forward_LCI.cpp
    COMBINED/realtime_stream.cpp
    COMBINED/realtime_forward_runner.cpp
)
target_include_directories(realtime_forward_LCI
    PUBLIC
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/GNSS
    ${CMAKE_SOURCE_DIR}/src/COMBINED
    ${CMAKE_SOURCE_DIR}/src/COMMON
)
target_link_libraries(realtime_forward_LCI
    PUBLIC
    imu
    gnss
    combined_module
    simpleini
)

if(WIN32)
    target_link_libraries(realtime_forward_LCI PUBLIC ws2_32)
endif()

add_executable(realtime_file_sender
    realtime_file_sender.cpp
    COMBINED/realtime_stream.cpp
    COMBINED/realtime_file_sender.cpp
)
target_include_directories(realtime_file_sender
    PUBLIC
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/GNSS
    ${CMAKE_SOURCE_DIR}/src/COMBINED
    ${CMAKE_SOURCE_DIR}/src/COMMON
)
target_link_libraries(realtime_file_sender
    PUBLIC
    imu
    gnss
    combined_module
    simpleini
)

if(WIN32)
    target_link_libraries(realtime_file_sender PUBLIC ws2_32)
endif()
