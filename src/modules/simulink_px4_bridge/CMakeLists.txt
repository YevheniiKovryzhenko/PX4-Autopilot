px4_add_module(
	MODULE custom__simulink_px4_bridge
	MAIN simulink_px4_bridge
	SRCS
		simulink_px4_bridge.cpp
	MODULE_CONFIG
		module.yaml
	)

