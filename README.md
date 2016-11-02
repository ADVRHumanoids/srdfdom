srdfdom
=======

Parser for Semantic Robot Description Format (SRDF).

Includes a cpp and a python parser

## Build Status

master branch: [![Build Status](https://travis-ci.org/ros-planning/srdfdom.png?branch=master)](https://travis-ci.org/ros-planning/srdfdom)

## Authors

Original reflection implementation for SDF and URDF.
*	Thomas Moulard - `urdfpy` implementation, integration
*	David Lu - `urdf_python` implementation, integration
*	Kelsey Hawkins - `urdf_parser_python` implementation, integration
*	Antonio El Khoury - bugfixes
*	Eric Cousineau - reflection (serialization?) changes
Reused for srdf python parser
*	Guillaume Walck - `srdfpy` conversion, integration

## Cpp example

test_parser.cpp contains examples how to access the srdf elements from the cpp parser

## Test

catkin_make run_tests_srdfdom



