cmake_minimum_required(VERSION 3.10)
project(sst-effects VERSION 0.5 LANGUAGES C CXX)

option(SST_EFFECTS_BUILD_EXAMPLES "Build the example drivers (which will also acivate tests)" OFF)
option(SST_EFFECTS_BUILD_TESTS "Build the test harness" OFF)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)
# there's a bit more below this

if (${SST_EFFECTS_BUILD_EXAMPLES})
    set(SST_EFFECTS_BUILD_TESTS ON)
endif()

if (${SST_EFFECTS_BUILD_TESTS})
    message(STATUS "Building tests")
    include(cmake/CPM.cmake)

    if (NOT TARGET sst-basic-blocks)
        CPMAddPackage(NAME sst-basic-blocks
                GITHUB_REPOSITORY surge-synthesizer/sst-basic-blocks
                GIT_TAG main
                )
    endif ()

    if (NOT TARGET sst-filters)
        CPMAddPackage(NAME sst-filters
                GITHUB_REPOSITORY surge-synthesizer/sst-filters
                GIT_TAG main
                )
    endif ()

    if (NOT TARGET sst-waveshapers)
        CPMAddPackage(NAME sst-waveshapers
                GITHUB_REPOSITORY surge-synthesizer/sst-waveshapers
                GIT_TAG main
        )
    endif ()

    if (NOT TARGET fmt)
        CPMAddPackage(NAME fmt
                GITHUB_REPOSITORY fmtlib/fmt
                GIT_TAG master
                )
    endif ()

    if (NOT TARGET eurorack)
        CPMAddPackage(NAME eurorack
                GITHUB_REPOSITORY surge-synthesizer/eurorack
                GIT_TAG surge
        )
    endif ()

    if (NOT TARGET simde)
        CPMAddPackage(NAME simde
                GITHUB_REPOSITORY simd-everywhere/simde
                VERSION 0.7.2
        )
        add_library(simde INTERFACE)
        target_include_directories(simde INTERFACE ${simde_SOURCE_DIR})
    endif ()

    add_executable(${PROJECT_NAME}-test
            tests/sst-effects-test.cpp
            tests/create-effect.cpp
            tests/create-voice-effect.cpp
            tests/concrete-runs.cpp
            tests/sfinae-test.cpp
            )

    if (MSVC)
        target_compile_definitions(${PROJECT_NAME}-test
                PUBLIC
                /wd4244
                /wd4267
                /wd4101
                /wd4305)
    endif()
    target_link_libraries(${PROJECT_NAME}-test PUBLIC simde sst-basic-blocks sst-filters sst-waveshapers fmt ${PROJECT_NAME})
    target_compile_definitions(${PROJECT_NAME}-test PUBLIC _USE_MATH_DEFINES=1)
    target_compile_definitions(${PROJECT_NAME}-test PRIVATE CATCH_CONFIG_DISABLE_EXCEPTIONS=1)
    target_include_directories(${PROJECT_NAME}-test PRIVATE libs/catch2)
    
    
    if(${SST_EFFECTS_BUILD_EXAMPLES})
        message(STATUS "Building Examples / CLI Driver")
        if (NOT TARGET dr_libs)
            CPMAddPackage(NAME dr_libs
                    GITHUB_REPOSITORY mackron/dr_libs
                    GIT_TAG master
                    )
            add_library(dr_libs INTERFACE)
            target_include_directories(dr_libs INTERFACE ${dr_libs_SOURCE_DIR})

            CPMAddPackage(NAME CL11
                    GITHUB_REPOSITORY CLIUtils/CLI11
                    GIT_TAG main)

        endif ()
        
        add_executable(voice-effect-example
            examples/voice-effect-example.cpp
        )
        target_link_libraries(voice-effect-example PUBLIC dr_libs CLI11::CLI11 simde sst-basic-blocks sst-filters sst-waveshapers fmt ${PROJECT_NAME})
        target_compile_definitions(voice-effect-example PUBLIC _USE_MATH_DEFINES=1)
        
        add_executable(effect-example
            examples/effect-example.cpp
        )
        target_link_libraries(effect-example PUBLIC dr_libs CLI11::CLI11 simde sst-basic-blocks sst-filters sst-waveshapers fmt ${PROJECT_NAME})
        target_compile_definitions(effect-example PUBLIC _USE_MATH_DEFINES=1)
    endif()

endif ()


if (TARGET eurorack)
    target_link_libraries(${PROJECT_NAME} INTERFACE eurorack)
    target_compile_definitions(${PROJECT_NAME} INTERFACE SST_EFFECTS_EURORACK=1)
else()
    message(STATUS "sst-effects built without eurorack library; Nimbus effect is no-op")
endif()
