project(K-CBS-Examples)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(ompl REQUIRED)


# # Add OMPL library here! This is computer specific
# add_library(ompl SHARED IMPORTED)
# set_target_properties(ompl PROPERTIES
#     IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/../ompl/build/Release/lib/libompl.so"
#     INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/../ompl/build/Release/src"
# )

include_directories(
    includes
    ${OMPL_INCLUDE_DIRS}
)

add_executable(demo_Empty32x32_10robots_dyn2ndOrderCars src/demo_Empty32x32_10robots_dyn2ndOrderCars.cpp)
target_link_libraries(demo_Empty32x32_10robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")
