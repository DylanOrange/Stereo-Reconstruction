cmake_minimum_required( VERSION 2.8 )
project(reconstruction)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# find_package( OpenCV 3.1 REQUIRED )
find_Package(G2O REQUIRED)
find_Package(Eigen3 REQUIRED)
# find_Package(Ceres REQUIRED)
# find_Package(Sophus REQUIRED)
# find_Package(CSparse REQUIRED)
find_package( OpenCV 3.2 REQUIRED )
# find_package( PCL REQUIRED )

SET(G2O_LIBS "/usr/local/lib/libg2o_csparse_extension.so" ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY}  "/usr/local/lib/libg2o_types_sba.so" cxsparse)

message("CSPARSE_INCLUDE_DIR: ${CSPARSE_INCLUDE_DIR}")
message("G2O_CSPARSE_EXTENSION_LIBRARY: ${G2O_CSPARSE_EXTENSION_LIBRARY}")
message("G2O_STUFF_LIBRARY: ${G2O_STUFF_LIBRARY}")


include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    "/usr/include/eigen3/"
    ${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR}
)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(HEADERS 
    "include.h"
)

set(SOURCES
    main.cpp
    orb.cpp
    sift.cpp
    undistort.cpp
    FindEssentialMatrix.cpp
    RecoverRT.cpp
    helper.cpp
    Rectify_KITTI.cpp
)

set(KIT
    kitti_kit.cpp
)

set(MATCHING
    matching.cpp
    orb.cpp
    undistort.cpp
    FindEssentialMatrix.cpp
    RecoverRT.cpp
    helper.cpp
    pointcloud.cpp
)

set(TEST_EXP
    test.cpp
    orb.cpp
    undistort.cpp
    FindEssentialMatrix.cpp
    RecoverRT.cpp
    helper.cpp
)

set(SIMULATION
    simulation.cpp
    orb.cpp
    undistort.cpp
    FindEssentialMatrix.cpp
    RecoverRT.cpp
    helper.cpp
)

set(BA
    bundle_adjustment.cpp
    undistort.cpp
    FindEssentialMatrix.cpp
    RecoverRT.cpp
    helper.cpp
    orb.cpp
    sift.cpp
)

set(RECTIFY
    rectify.cpp
    helper.cpp
    Rectify_KITTI.cpp
)


add_executable( main ${HEADERS} ${SOURCES}  )
target_link_libraries( main ${OpenCV_LIBS} )

add_executable( kit ${HEADERS} ${KIT}  )
target_link_libraries( kit ${OpenCV_LIBS} )

add_executable( matching ${HEADERS} ${MATCHING}  )
target_link_libraries( matching ${OpenCV_LIBS} )

add_executable(ba ${HEADERS} ${BA})
target_link_libraries(ba ${G2O_LIBS}  ${OpenCV_LIBS})

add_executable( test_exp ${HEADERS} ${TEST_EXP} )
target_link_libraries( test_exp  ${OpenCV_LIBS})

add_executable( simulation ${HEADERS} ${SIMULATION} )
target_link_libraries( simulation  ${OpenCV_LIBS})

add_executable( rectify ${HEADERS} ${RECTIFY} )
target_link_libraries( rectify  ${OpenCV_LIBS})