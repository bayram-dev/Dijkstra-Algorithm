cmake_minimum_required(VERSION 3.19)
project(Dijkstra)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(
	Dijkstra
	dijkstra.cpp
	dijkstra.h
	input.cpp
	input.h
	main.cpp
	structures.h
	)
