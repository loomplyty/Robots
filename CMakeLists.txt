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

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
add_definitions(-DWIN32 -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
add_definitions(-DUNIX)
endif(UNIX)

# set include and lib folders
if(UNIX)
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat rt)
set(XSENS_INCL_DIR "/usr/Aris_Dependent/xsens/include")
set(XSENS_LINK_LIB xdeviceclass xcommunication xstypes)
set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")
set(XNNITE_INCL_DIR "/usr/include/nite")#Driver for kinect
set(XNNITE_LINK_DIR "/usr/lib")
set(XNNITE_LINK_LIB )
set(OPENNI_INCL_DIR "/usr/include/ni")#open NI for kinect
set(OPENNI_LINK_DIR "/usr/lib")
set(OPENNI_LINK_LIB OpenNI)
set(ARIS_INCL_DIR "/usr/aris/include")
set(ARIS_LINK_DIR "/usr/aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
set(ROBOTS_INCL_DIR "src/Robot_Type_I" "src/Robot_Type_II" "src/Robot_Base")
set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)

set(ALL_INCL_DIR ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${OPENNI_INCL_DIR} ${XNNITE_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${OPENNI_LINK_DIR} ${XNNITE_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${OPENNI_LINK_LIB} ${XNNITE_LINK_LIB} ${XSENS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)
if(WIN32)
set(GENERAL_INCL_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIB ws2_32 setupapi)
set(XSENS_INCL_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIB xdeviceclass)
set(ARIS_INCL_DIR "C:/aris/include")
set(ARIS_LINK_DIR "C:/aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
set(ROBOTS_INCL_DIR "src/Robot_Type_I" "src/Robot_Type_II" "src/Robot_Base")
set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)



set(ALL_INCL_DIR ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} )
set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${XSENS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(WIN32)

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

# build Robot_Base lib
set(SOURCE Robot_Base Robot_Gait)
PREPEND_CPP(FULL_SRC src/Robot_Base ${SOURCE})
PREPEND_H(FULL_H src/Robot_Base ${SOURCE})
add_library(Robot_Base STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# build Robot_Type_I lib
set(SOURCE Robot_Type_I)
PREPEND_CPP(FULL_SRC src/Robot_Type_I ${SOURCE})
PREPEND_H(FULL_H src/Robot_Type_I ${SOURCE})
add_library(Robot_Type_I STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# build Robot_Type_II lib
set(SOURCE Robot_Type_II)
PREPEND_CPP(FULL_SRC src/Robot_Type_II ${SOURCE})
PREPEND_H(FULL_H src/Robot_Type_II ${SOURCE})
add_library(Robot_Type_II STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})


################################### build tests for Robots ####################################
enable_testing()

add_executable(test_simple_walk test/test_simple_walk.cpp)
target_link_libraries(test_simple_walk ${ALL_LINK_LIB})
add_test(NAME test_simple_walk COMMAND test_simple_walk)


################################### build demos ####################################
# Make demo projects
set(MAKE_DEMO " ")
if(MAKE_DEMO)



# build programs
set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_HexapodIII ${SOURCE_FILES})
add_executable(demo_HexapodIII ${FULL_SRC})
target_link_libraries(demo_HexapodIII ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_HexapodIV ${SOURCE_FILES})
add_executable(demo_HexapodIV ${FULL_SRC})
target_link_libraries(demo_HexapodIV ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp plan.h plan.cpp)
PREPEND(FULL_SRC demo/demo_Plan ${SOURCE_FILES})
add_executable(demo_Plan ${FULL_SRC})
target_link_libraries(demo_Plan ${ALL_LINK_LIB})

if(WIN32)
set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_RobotServerIII ${SOURCE_FILES})
add_executable(demo_RobotServerIII ${FULL_SRC})
target_link_libraries(demo_RobotServerIII ${ALL_LINK_LIB})
endif(WIN32)

endif(MAKE_DEMO)

############################################ install ############################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Robots")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/Robots")
endif(WIN32)

install(TARGETS Robot_Type_I Robot_Type_II Robot_Base ARCHIVE DESTINATION lib)
install(FILES ${INCLUDE_HEADER_FILES} DESTINATION include)
install(DIRECTORY src/Robot_Type_I/resource/ DESTINATION resource/Robot_Type_I)

############################################ uninstall ############################################
if(UNIX)
	add_custom_target(uninstall rm -r /usr/Robots)
endif(UNIX)
