# Get jrl-cmakemodules package

# Option 1: pass -DJRL_CMAKEMODULES_SOURCE_DIR=... to cmake command line
if(JRL_CMAKEMODULES_SOURCE_DIR)
    message(
        DEBUG
        "JRL_CMAKEMODULES_SOURCE_DIR variable set, adding jrl-cmakemodules from source directory: ${JRL_CMAKEMODULES_SOURCE_DIR}"
    )
    add_subdirectory(${JRL_CMAKEMODULES_SOURCE_DIR} jrl-cmakemodules)
    return()
endif()

# Option 2: use JRL_CMAKEMODULES_SOURCE_DIR environment variable (pixi might unset it, prefer option 1)
if(ENV{JRL_CMAKEMODULES_SOURCE_DIR})
    message(
        DEBUG
        "JRL_CMAKEMODULES_SOURCE_DIR environement variable set, adding jrl-cmakemodules from source directory: ${JRL_CMAKEMODULES_SOURCE_DIR}"
    )
    add_subdirectory(${JRL_CMAKEMODULES_SOURCE_DIR} jrl-cmakemodules)
    return()
endif()

# Try to look for the installed package
message(DEBUG "Looking for jrl-cmakemodules using find_package().")
find_package(jrl-cmakemodules 1.2.0 CONFIG QUIET)

# If we have the package, we are done.
if(jrl-cmakemodules_FOUND)
    message(DEBUG "Found jrl-cmakemodules package via find_package().")
    return()
else()
    message(DEBUG "jrl-cmakemodules package not found using find_package().")
endif()

# Fallback to FetchContent if not found
set(JRL_GIT_REPOSITORY "https://github.com/ahoarau/jrl-cmakemodules.git")
set(JRL_GIT_TAG "jrl-next")

message(
    DEBUG
    "Fetching jrl-cmakemodules using FetchContent:
  GIT_REPOSITORY: ${JRL_GIT_REPOSITORY}
  GIT_TAG       : ${JRL_GIT_TAG}
"
)

include(FetchContent)
FetchContent_Declare(
    jrl-cmakemodules
    GIT_REPOSITORY ${JRL_GIT_REPOSITORY}
    GIT_TAG ${JRL_GIT_TAG}
)
FetchContent_MakeAvailable(jrl-cmakemodules)
