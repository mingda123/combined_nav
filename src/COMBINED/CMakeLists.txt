project(combined_module)

set(COMBINED_SOURCES
    alignment.cpp
    lci_filter.cpp
    lci_runner.cpp
)

add_library(${PROJECT_NAME} ${COMBINED_SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/IMU
    ${CMAKE_SOURCE_DIR}/src/GNSS
    ${CMAKE_SOURCE_DIR}/src/COMMON
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    imu
    gnss
)
