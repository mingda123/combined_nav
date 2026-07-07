project(gnss)

file(GLOB GNSS_SOURCES "*.cpp")

add_library(${PROJECT_NAME} ${GNSS_SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    common
)
