cmake_minimum_required(VERSION 3.20)
project(RoverBrain CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(asio CONFIG REQUIRED)
find_package(Threads REQUIRED)


file(GLOB_RECURSE ROVER_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(rover_brain ${ROVER_SOURCES})


target_include_directories(rover_brain PRIVATE src)
target_compile_definitions(rover_brain PRIVATE ASIO_STANDALONE)


target_link_libraries(rover_brain 
    PRIVATE 
        asio::asio
        Threads::Threads
)


if(ROVER_STATIC_BUILD)
    target_link_options(rover_brain PRIVATE -static -static-libgcc -static-libstdc++)
endif()