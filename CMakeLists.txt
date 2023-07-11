cmake_minimum_required(VERSION 3.10.0)
project(EBO VERSION 0.1.0)

# include(CTest)
# enable_testing()

set(CMAKE_CXX_STANDARD 17)

set(LIBS libs/tinyxml2.cpp)

# set(SRC_LIST src/readmap.cpp src/calcost.cpp src/heuristic.cpp
#         src/arrayoperations.cpp src/virtualtask.cpp src/generator.cpp src/HSAS.cpp src/fileio.cpp)
# set(SRC_LIST1 globalvar.cpp)
# set(SRC_LIST2 ls/ls.cpp ls/myls.cpp ls/mylsop.cpp)
# set(SRC_LIST3 TSA/TSA.cpp TSA/tsals.cpp)
 set(SRC_LIST4 MAENS/MAENS.cpp MAENS/operators.cpp)


set(SRC_LIST src/heuristic.cpp src/arrayoperations.cpp src/SBBA.cpp src/fileio.cpp)

add_executable(dcarp-sumo-compare main.cpp utils.cpp ${LIBS} ${SRC_LIST} ${SRC_LIST4})#${SRC_LIST1} ${SRC_LIST1} ${SRC_LIST2} ${SRC_LIST3} )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include(CPack)
