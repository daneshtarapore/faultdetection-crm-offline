cmake_minimum_required(VERSION 2.8)

project (faultdetection-crm-offline)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_executable (faultdetection-crm-offline 
                main.cpp
		celldatacontainers.h 
		celldatacontainers.cpp
		featurevectorsinrobotagent.h
		featurevectorsinrobotagent.cpp 
		crminrobotagent_optimised.h 
		crminrobotagent_optimised.cpp )