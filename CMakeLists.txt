cmake_minimum_required(VERSION 3.22)

set(LIB_MODULE_DIRS math crc pid filter tools)

set(LIB_SOURCES)

foreach(module ${LIB_MODULE_DIRS})
    file(GLOB module_sources ${CMAKE_CURRENT_SOURCE_DIR}/${module}/src/*.cpp)
    list(APPEND LIB_SOURCES ${module_sources})
endforeach()

add_library(pnx_libraries STATIC ${LIB_SOURCES})

set(LIB_PUBLIC_INCLUDES)

foreach(module ${LIB_MODULE_DIRS})
    list(APPEND LIB_PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/${module}/include)
endforeach()

target_include_directories(pnx_libraries PUBLIC ${LIB_PUBLIC_INCLUDES})

target_link_libraries(pnx_libraries PUBLIC pnx_platform pnx_os)

target_compile_features(pnx_libraries PUBLIC cxx_std_17)
