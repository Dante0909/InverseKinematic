cmake_minimum_required(VERSION 3.15)
project(GTI320-labos)

#--------------------------------------------------
# Add google test and setup build
#--------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# No need for GMock
set( BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set( INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set( GTEST_FORCE_SHARED_CRT ON CACHE BOOL "" FORCE)
set( GTEST_DISABLE_PTHREADS ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

#--------------------------------------------------
# Add nanogui
#--------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  nanogui
  GIT_REPOSITORY https://github.com/mitsuba-renderer/nanogui.git
  GIT_PROGRESS TRUE
)
set( NANOGUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set( NANOGUI_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set( NANOGUI_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set( NANOGUI_USE_OPENGL ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nanogui)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include_directories(${CMAKE_SOURCE_DIR}/labo01/src ${COMMON_INCLUDES})
add_subdirectory(labo01)
add_subdirectory(labo_ik)

if( MSVC )
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT labo_ik)
endif()
  
