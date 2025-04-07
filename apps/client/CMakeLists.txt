add_executable(client main.cc render.cc ../robot.hh)

target_link_libraries(
    client
    PRIVATE
    au
    raylib
    ntcore
    sim
)

target_include_directories(
  client
  PRIVATE
  ..
)

target_compile_features(client PRIVATE cxx_std_23)
