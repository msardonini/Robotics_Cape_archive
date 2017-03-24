cmake_minimum_required(VERSION 2.8)

include_directories(
    include/
)

file(GLOB flyMS_SRC
    "include/*.h"
    "src/*.c"
    "/root/Robotics_Cape/libraries/*.h"
    "/root/Robotics_Cape/libraries/*.c"
)



# Test for mavlink custom messages receive and send
add_executable(flyMS
    ${flyMS_SRC}
    )

target_link_libraries(flyMS
    pthread
    m

)


