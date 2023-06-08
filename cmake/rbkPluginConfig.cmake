# project version
# ---------------

set(PROJ_NAME ${PROJECT_NAME})

STRING(TOUPPER "${PROJECT_NAME}" PROJ_NAME)

set(${PROJ_NAME}_VERSION_MAJOR "1")
set(${PROJ_NAME}_VERSION_MINOR "0")
set(${PROJ_NAME}_VERSION_PATCH "0")
set(${PROJ_NAME}_VERSION ${${PROJ_NAME}_VERSION_MAJOR}.${${PROJ_NAME}_VERSION_MINOR}.${${PROJ_NAME}_VERSION_PATCH})
mark_as_advanced(${PROJ_NAME}_VERSION)

message("Plugin ${PROJECT_NAME} v${${PROJ_NAME}_VERSION}, © 2015-2020 SEER Co,.Ltd.")
message("Auther: liuyangyang")
message("Email: liuyy@seer-group.com")
message("Website: http://www.seer-robotics.com")
message("=====================================================")

# urg 1.2.0
#find_package(Urg)
