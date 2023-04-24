cmake_minimum_required(VERSION 3.10)
project(sst-effects VERSION 0.5 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

if (${SST_EFFECTS_BUILD_TESTS})
    include(cmake/CPM.cmake)

    if (NOT TARGET sst-basic-blocks)
        CPMAddPackage(NAME sst-basic-blocks
                GITHUB_REPOSITORY surge-synthesizer/sst-basic-blocks
                GIT_TAG main
                )
    endif()

    if (NOT TARGET sst-filters)
        CPMAddPackage(NAME sst-filters
                GITHUB_REPOSITORY surge-synthesizer/sst-filters
                GIT_TAG main
                )
    endif()

    if (NOT TARGET simde)
        CPMAddPackage(NAME simde
                GITHUB_REPOSITORY simd-everywhere/simde
                VERSION 0.7.2
                )
        add_library(simde INTERFACE)
        target_include_directories(simde INTERFACE ${simde_SOURCE_DIR})
    endif()

    add_executable(${PROJECT_NAME}-test
            tests/sst-effects-test.cpp
            tests/create-effect.cpp
            tests/concrete-runs.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test PUBLIC simde sst-basic-blocks sst-filters ${PROJECT_NAME})
    target_compile_definitions(${PROJECT_NAME}-test PUBLIC _USE_MATH_DEFINES=1)
    target_compile_definitions(${PROJECT_NAME}-test PRIVATE CATCH_CONFIG_DISABLE_EXCEPTIONS=1)

endif ()
