
set(SOURCE_FILES
        strap_down.cpp
        entities.h
        earth_model.cpp
        earth_model.h)

set(HEADER_FILES)

add_library(StrapDown ${SOURCE_FILES} ${SOURCE_FILES})
target_compile_options(StrapDown PRIVATE -DDEBUG_BUILD -O0)
