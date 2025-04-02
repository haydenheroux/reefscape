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

    if (NOT TARGET raylib)
        FetchContent_Declare(
            raylib
            GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
            GIT_TAG "master"
            GIT_PROGRESS TRUE
        )

        FetchContent_MakeAvailable(raylib)
    endif()

endfunction()
