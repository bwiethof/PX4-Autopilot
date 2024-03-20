
add_subdirectory(srukf/lib)

set(SOURCE_FILES UKF.cpp)
set(INCLUDE_FILES UKF.h)

add_library(visual_ukf ${SOURCE_FILES} ${INCLUDE_FILES})
target_link_libraries(visual_ukf INTERFACE srukf)
target_include_directories(visual_ukf PRIVATE $<TARGET_PROPERTY:srukf,INTERFACE_INCLUDE_DIRECTORIES>)
