project(K-CBS-Examples)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED) # make sure this is the ompl with the K-CBS implementation

include_directories(
    includes
    ${OMPL_INCLUDE_DIRS}
)

add_executable(demo_Empty32x32_10robots_dyn2ndOrderCars src/demo_Empty32x32_10robots_dyn2ndOrderCars.cpp)
target_link_libraries(demo_Empty32x32_10robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

add_executable(demo_Empty32x32_15robots_dyn2ndOrderCars src/demo_Empty32x32_15robots_dyn2ndOrderCars.cpp)
target_link_libraries(demo_Empty32x32_15robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

add_executable(demo_Empty32x32_20robots_dyn2ndOrderCars src/demo_Empty32x32_20robots_dyn2ndOrderCars.cpp)
target_link_libraries(demo_Empty32x32_20robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")
