add_library(sensors
	sensors.h
	sensors.cpp
)

target_link_libraries(sensors
	hardware_i2c
	pico_stdlib
)
