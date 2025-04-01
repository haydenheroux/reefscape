add_executable(main main.cc)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(
    main
    PRIVATE
    au
    Eigen3::Eigen
)

target_compile_features(main PRIVATE cxx_std_23)
