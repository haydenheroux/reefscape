project(points)

add_executable(points main.cc)

target_link_libraries(points PRIVATE au common ntcore raylib)

target_compile_features(points PRIVATE cxx_std_23)
