function(setup_dependencies)

    if (NOT TARGET au)
        include(FetchContent)
        FetchContent_Declare(
          Au
          GIT_REPOSITORY https://github.com/aurora-opensource/au
          GIT_TAG "main"
          EXCLUDE_FROM_ALL
        )
        FetchContent_MakeAvailable(Au)
    endif()

    if (NOT TARGET Eigen3::Eigen)
        find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    endif()

endfunction()
