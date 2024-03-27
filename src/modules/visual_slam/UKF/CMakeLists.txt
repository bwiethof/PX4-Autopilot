
add_subdirectory(srukf/lib)

set(SOURCE_FILES UKF.cpp
        state/Position_NED.cpp
        state/Position_NED.h)
set(INCLUDE_FILES UKF.h
        state/Position.cpp
        state/Position.h
        sensor/GPS.cpp
        sensor/GPS.h
        state/Velocity.cpp
        state/Velocity.h
        math/coordinates.cpp
        math/coordinates.h
        state/Position_NED.cpp
        state/Position_NED.h
)

add_library(visual_ukf ${SOURCE_FILES} ${INCLUDE_FILES}
        state/Position.cpp
        state/Position.h)
target_compile_options(visual_ukf PRIVATE -DDEBUG_BUILD -O0)
target_link_libraries(visual_ukf INTERFACE srukf)
target_include_directories(visual_ukf PRIVATE $<TARGET_PROPERTY:srukf,INTERFACE_INCLUDE_DIRECTORIES> SYSTEM GLAD)
