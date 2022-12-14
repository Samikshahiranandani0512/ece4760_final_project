# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(ece-4760-project)

# initialize the sdk
pico_sdk_init()

#######

# name anything you want
add_executable(final_proj)

# must match with pio filename and executable name from above
pico_generate_pio_header(final_proj ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(final_proj ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(final_proj ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(final_proj PRIVATE 
	
	final_project.c
	vga16_graphics.c
	song.c
	)

# must match with executable name
target_link_libraries(final_proj PRIVATE 
	pico_stdlib 
	pico_bootsel_via_double_reset 
	hardware_pio 
	hardware_dma 
	hardware_adc 
	hardware_sync
	hardware_irq
	hardware_pwm
	hardware_spi
	pico_multicore)

# must match with executable name
pico_add_extra_outputs(final_proj)

add_compile_options(-Ofast)
