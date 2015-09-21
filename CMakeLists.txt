cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)



project(ROBOTS)


if(UNIX)
	set(CMAKE_C_COMPILER "/usr/bin/gcc")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)


# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set include and lib folders
if(UNIX)
include_directories("/usr/xenomai/include")
include_directories("/opt/etherlab/include")
include_directories("/usr/lapack/include")
include_directories("/usr/Aris/include")
link_directories(/usr/lapack/lib)
link_directories(/usr/Aris/lib)
endif(UNIX)
if(WIN32)
include_directories("C:/lapack/include")
include_directories("C:/Aris/include")
link_directories(C:/lapack/lib)
link_directories(C:/Aris/lib)
endif(WIN32)

include_directories("src/HexapodIII")
include_directories("src/HexapodIV")
include_directories("src/Robot_Base")
include_directories("src/Robot_Server")


# set link files
if(WIN32)
set(SYSTEM_LINK_LIBS ws2_32)
set(LAPACK_LINK_LIBS liblapacke liblapack cblas libblas)
set(ARIS_LINK_LIBS Aris_Plan Aris_DynKer Aris_ExpCal Aris_Core)
endif(WIN32)
if(UNIX)
set(SYSTEM_LINK_LIBS pthread gfortran)
set(LAPACK_LINK_LIBS lapacke lapack cblas refblas)
set(ARIS_LINK_LIBS Aris_Control Aris_Plan Aris_DynKer Aris_ExpCal Aris_Core)
endif(UNIX)
set(ROBOTS_LINK_LIBS HexapodIV HexapodIII Robot_Server Robot_Base)



# build Robot_Base lib
set(SOURCE Robot_Base Robot_Gait)
PREPEND_CPP(FULL_SRC src/Robot_Base ${SOURCE})
PREPEND_H(FULL_H src/Robot_Base ${SOURCE})
add_library(Robot_Base STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# build Robot_Server lib
set(SOURCE Robot_Server)
PREPEND_CPP(FULL_SRC src/Robot_Server ${SOURCE})
PREPEND_H(FULL_H src/Robot_Server ${SOURCE})
add_library(Robot_Server STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# build Robot_Server lib
set(SOURCE Robot_Client)
PREPEND_CPP(FULL_SRC src/Robot_Client ${SOURCE})
PREPEND_H(FULL_H src/Robot_Client ${SOURCE})
add_library(Robot_Client STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# build HexapodIII lib
set(SOURCE HexapodIII)
PREPEND_CPP(FULL_SRC src/HexapodIII ${SOURCE})
PREPEND_H(FULL_H src/HexapodIII ${SOURCE})
add_library(HexapodIII STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# build HexapodIV lib
set(SOURCE HexapodIV)
PREPEND_CPP(FULL_SRC src/HexapodIV ${SOURCE})
PREPEND_H(FULL_H src/HexapodIV ${SOURCE})
add_library(HexapodIV STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})




################################### build demos ####################################
# Make demo projects
set(MAKE_DEMO " ")
if(MAKE_DEMO)



# build programs
set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_HexapodIII ${SOURCE_FILES})
add_executable(demo_HexapodIII ${FULL_SRC})
target_link_libraries(demo_HexapodIII ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_HexapodIV ${SOURCE_FILES})
add_executable(demo_HexapodIV ${FULL_SRC})
target_link_libraries(demo_HexapodIV ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Test ${SOURCE_FILES})
add_executable(demo_Test ${FULL_SRC})
target_link_libraries(demo_Test ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

if(WIN32)
set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_RobotServerIII ${SOURCE_FILES})
add_executable(demo_RobotServerIII ${FULL_SRC})
target_link_libraries(demo_RobotServerIII ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})
endif(WIN32)

endif(MAKE_DEMO)

############################################ install ############################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Robots")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/Robots")
endif(WIN32)

install(TARGETS HexapodIV HexapodIII Robot_Client Robot_Base Robot_Server ARCHIVE DESTINATION lib)
install(FILES ${INCLUDE_HEADER_FILES} DESTINATION include)
install(DIRECTORY src/HexapodIII/resource/ DESTINATION resource/HexapodIII)

############################################ uninstall ############################################
if(UNIX)
	add_custom_target(uninstall rm -r /usr/Robots)
endif(UNIX)
