cmake_minimum_required(VERSION 3.15)

project(labo_ik)

# Setup language requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--------------------------------------------------
# Sous MAC, OpenGL est Deprecated, mais toujours
# fonctionnel, on veut éviter tous les warnings
# durant la compilation.
#--------------------------------------------------
if (APPLE)
  add_definitions( -DGL_SILENCE_DEPRECATION )
endif()

add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(${nanogui_SRC_DIR}/include)
include_directories(${NANOGUI_EXTRA_INCS})
include_directories(${PROJECT_SOURCE_DIR}/../labo01/src ${COMMON_INCLUDES})

find_package(OpenGL REQUIRED)

# Add .cpp and .h files
set(HEADERS IKApplication.h IKGLCanvas.h LinkUI.h TargetUI.h Armature.h IKSolver.h SVD.h)
set(SOURCE main.cpp IKApplication.cpp IKGLCanvas.cpp LinkUI.cpp TargetUI.cpp Armature.cpp IKSolver.cpp)
add_executable(labo_ik ${SOURCE} ${HEADERS})

target_link_libraries(labo_ik nanogui opengl32 ${NANOGUI_EXTRA_LIBS})

if(MSVC) 
	set_property(TARGET labo_ik PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/labo_ik)
endif()