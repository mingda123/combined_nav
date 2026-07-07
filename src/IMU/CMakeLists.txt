project(imu)

# 查找所有源文件
file(GLOB IMU_SOURCES "*.cpp")

# 创建静态(STATIC)/动态(SHARED)库
add_library(${PROJECT_NAME} ${IMU_SOURCES})

# 添加头文件搜索路径
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    common
)