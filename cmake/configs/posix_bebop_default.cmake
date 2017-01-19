include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf-raspbian.cmake)

add_definitions(
	-D__PX4_POSIX_BEBOP
	-D__DF_LINUX # Define needed DriverFramework
	-D__DF_BEBOP # Define needed DriverFramework
	)

set(CMAKE_PROGRAM_PATH
	"${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin"
	${CMAKE_PROGRAM_PATH}
	)

set(config_module_list

  # examples/px4_simple_app

	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	platforms/posix/drivers/bebop_flow
	platforms/posix/drivers/df_ak8963_wrapper
	platforms/posix/drivers/df_bebop_bus_wrapper
	platforms/posix/drivers/df_bebop_rangefinder_wrapper
	platforms/posix/drivers/df_mpu6050_wrapper
	platforms/posix/drivers/df_ms5607_wrapper

	#
	# System commands
	#
	systemcmds/esc_calib
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/topic_listener
	systemcmds/ver

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/local_position_estimator
	modules/position_estimator_inav

	#
	# Vehicle Control
	#
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Library modules
	#
	modules/commander
	modules/dataman
	modules/land_detector
	modules/logger
	modules/mavlink
	modules/navigator
	modules/param
	modules/sdlog2
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB

	#
	# PX4 drivers
	#
	drivers/gps

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	lib/mathlib
	lib/mathlib/math/filter
	lib/OpticalFlow
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/terrain_estimation
	lib/version

	#
	# POSIX
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
)

set(config_df_driver_list
	ak8963
	bebop_bus
	bebop_rangefinder
	mpu6050
	ms5607
	mt9v117
)
