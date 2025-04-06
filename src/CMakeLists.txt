add_library(sim Elevator.cc Elevator.hh units.hh common.hh)
target_include_directories(sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
    sim
    PRIVATE
    au
    Eigen3::Eigen
    raylib
)

target_compile_features(sim PRIVATE cxx_std_23)
