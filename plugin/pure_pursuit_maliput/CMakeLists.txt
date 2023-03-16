find_package(ignition-cmake2 REQUIRED)

add_library(PurePursuitMaliput SHARED PurePursuitMaliput.cc)
set_property(TARGET PurePursuitMaliput PROPERTY CXX_STANDARD 17)

find_package(maliput)
find_package(maliput_osm)

target_link_libraries(PurePursuitMaliput
  PRIVATE
    ignition-plugin1::ignition-plugin1
    ignition-gazebo3::ignition-gazebo3
    ignition-transport8::ignition-transport8
    maliput::api
    maliput::plugin
    maliput::math
)

install(TARGETS PurePursuitMaliput DESTINATION share/${PROJECT_NAME}/plugins)
