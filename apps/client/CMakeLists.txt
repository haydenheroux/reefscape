add_executable(client main.cc render.cc)

target_link_libraries(
    client
    PRIVATE
    au
    raylib
    ntcore
    sim
)

target_compile_features(client PRIVATE cxx_std_23)
