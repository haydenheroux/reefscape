add_executable(main main.cc Elevator.cc render.cc)

target_link_libraries(
    main
    PRIVATE
    au
    Eigen3::Eigen
    raylib
)

target_compile_features(main PRIVATE cxx_std_23)
