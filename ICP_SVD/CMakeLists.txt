cmake_minimum_required (VERSION 3.5)

# Executable name: 
##############################################################
SET(EXEC OUTFile)
##############################################################

project(${EXEC})

#include
include_directories(/usr/local/include/eigen3)
include_directories(/usr/local/include)
include_directories(/home/aniruddha/Desktop/ICP/ICP_SVD/include)
include_directories(/home/aniruddha/Desktop/ICP/ICP_SVD/data)
# include_directories(/usr/local/cuda/include)

#link
link_directories(/usr/local/build/libnabo.a)
link_directories(/usr/local/lib)

# support for Libnabo  
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")

set(nlopt_ADDED_LIBS /usr/local/lib/libnlopt.so)
set(UTLIB src/utilities.cpp 
	src/file_rw.cpp 
	src/transformation_utilities.cpp)

##############################################################
set(MAIN_FILE src/main_file.cpp)
##############################################################

add_executable(${EXEC} ${MAIN_FILE} ${UTLIB})

##LINKED LIBRARIES
#link library for libnabo
target_link_libraries(${EXEC} /usr/local/build/libnabo.a gomp)
#link library for nlopt 
target_link_libraries (${EXEC} ${nlopt_ADDED_LIBS})

# target_link_libraries(${EXEC} /usr/local/cuda/bin/nvcc)

set_property(TARGET ${EXEC} PROPERTY CXX_STANDARD 14)
