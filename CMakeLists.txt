project(K-CBS-Examples)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED) # make sure this is the ompl with the K-CBS implementation

include_directories(
    includes
    ${OMPL_INCLUDE_DIRS}
)

# add_executable(demo_Empty32x32_10robots_dyn2ndOrderCars src/demo_Empty32x32_10robots_dyn2ndOrderCars.cpp)
# target_link_libraries(demo_Empty32x32_10robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

# add_executable(demo_Empty32x32_15robots_dyn2ndOrderCars src/demo_Empty32x32_15robots_dyn2ndOrderCars.cpp)
# target_link_libraries(demo_Empty32x32_15robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

# add_executable(demo_Empty32x32_20robots_dyn2ndOrderCars src/demo_Empty32x32_20robots_dyn2ndOrderCars.cpp)
# target_link_libraries(demo_Empty32x32_20robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path

# add_executable(demo_Benchmark_Empty32x32_20robots_dyn2ndOrderCars src/demo_Benchmark_Empty32x32_20robots_dyn2ndOrderCars.cpp)
# target_link_libraries(demo_Benchmark_Empty32x32_20robots_dyn2ndOrderCars ${OMPL_LIBRARIES}) # also adds the required include path


file(GLOB DEMO_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}/src/ src/*.cpp )


include(CMakePrintHelpers)
cmake_print_variables(DEMO_SOURCES)
cmake_print_variables(CMAKE_SOURCE_DIR)

foreach( demoFile ${DEMO_SOURCES} )
    string( REPLACE ".cpp" "" demoName ${demoFile} )
    add_executable( ${demoName} src/${demoFile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${demoName} ${OMPL_LIBRARIES} )
endforeach( demoFile ${DEMO_SOURCES} )

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")
