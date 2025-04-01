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

endfunction()
