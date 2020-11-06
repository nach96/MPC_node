cmake_minimum_required(VERSION 2.8)
project(myNLP)
set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)
#set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)

include_directories(${IPOPT_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
link_directories(${IPOPT_LIBRARY_DIRS})

message("ipopt binary path is: " ${IPOPT_LIBRARY_DIRS} )
message("ipopt include path is: " ${IPOPT_INCLUDE_DIRS} )

add_library(myNLPlib "mynlp.cpp")
add_executable(${PROJECT_NAME} "main.cpp")
#target_link_libraries(${PROJECT_NAME} ipopt)
target_link_libraries(myNLPlib ipopt)
target_link_libraries(${PROJECT_NAME} myNLPlib)
