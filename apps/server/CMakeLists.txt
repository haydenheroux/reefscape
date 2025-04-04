add_executable(server main.cc)

target_link_libraries(
    server 
    PRIVATE
    au
    Eigen3::Eigen
    ntcore
    sim
)

target_compile_features(server PRIVATE cxx_std_23)
