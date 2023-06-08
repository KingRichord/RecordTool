# output directory
# ----------------
include(SetOutputDirectory)

# pre process
# -----------
find_package(Threads)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
add_subdirectory(communication)
#SET(OpenCV_DIR ${RBK_THIRDPARTY_LIB_DIR}/opencv)
#find_package(OpenCV 3 QUIET
#        COMPONENTS
#        core
#        imgproc
#        highgui
#        calib3d)
#include_directories(${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIB_PATH})

# define
# ------
add_definitions(-D__RBK_PLUGIN__="${PROJECT_NAME}")
add_definitions(-D__RBK_PLUGIN_ABBREVIATION__="RT")
# source
# ------
aux_source_directory(. SRCS)

IF (NOT IS_SUBPROJECT)
    include_directories(BEFORE ${RBK_PATH}/include/robokit/protocol)
ELSE ()
    include_directories(BEFORE ${RoboKit_SOURCE_DIR}/src/robokit/protocol)
ENDIF ()

file(GLOB HEADERS ${HEADERS})
file(GLOB SRCS ${SRCS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME}
        communication
        core
        chasis
        utils_base
        utils_error
        utils_code
        utils_globaldata
        utils_assets
        utils_system
        utils_timer
        utils_runtimedatadb
        utils_deviceinfo
        utils_network
        protocol
        foundation)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "rbk_plugin")

# other
# -----
