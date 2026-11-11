set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/parameters.yaml
)

# Keep stock parameters that are still used when their owning controller is disabled.
foreach(controller mc_pos_control fw_pos_control fw_att_control fw_rate_control vtol_att_control)
	string(TOUPPER "${controller}" controller_symbol)
	if(NOT CONFIG_MODULES_${controller_symbol})
		set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES
			${CMAKE_CURRENT_SOURCE_DIR}/stock_${controller}.yaml
		)
	endif()
endforeach()
