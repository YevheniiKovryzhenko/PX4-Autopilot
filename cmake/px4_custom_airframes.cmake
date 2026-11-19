# Collection flags select source directories; no airframe names belong here.
set(PX4_AIRFRAME_METADATA_ROOT ${PX4_BINARY_DIR}/airframe_metadata)
file(REMOVE_RECURSE ${PX4_AIRFRAME_METADATA_ROOT})
file(MAKE_DIRECTORY ${PX4_AIRFRAME_METADATA_ROOT}/airframes)
set(PX4_AIRFRAME_DEPENDENCIES)
set(PX4_CUSTOM_AIRFRAME_COPY_COMMANDS)
set(PX4_GZ_AIRFRAMES)
foreach(init_dir init.d init.d-posix)
    if(init_dir STREQUAL "init.d-posix" AND NOT PX4_PLATFORM STREQUAL "posix")
        continue()
    endif()
    set(selected_airframes)
    if(CONFIG_BOARD_STOCK_AIRFRAMES)
        file(GLOB stock_airframes CONFIGURE_DEPENDS
            ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/${init_dir}/airframes/[0-9]*)
        list(APPEND selected_airframes ${stock_airframes})
    endif()
    if(CONFIG_BOARD_CUSTOM_AIRFRAMES)
        file(GLOB custom_airframes CONFIGURE_DEPENDS
            ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/custom_airframes/${init_dir}/airframes/[0-9]*)
        if(init_dir STREQUAL "init.d-posix" AND PX4_PLATFORM STREQUAL "posix" AND CONFIG_MODULES_SIMULATION_GZ_BRIDGE)
            file(GLOB project_airframes CONFIGURE_DEPENDS
                ${PX4_SOURCE_DIR}/Tools/simulation/gz/airframes/[0-9]*)
            list(APPEND custom_airframes ${project_airframes})
        endif()
        list(APPEND selected_airframes ${custom_airframes})
        list(APPEND PX4_CUSTOM_AIRFRAME_COPY_COMMANDS COMMAND ${CMAKE_COMMAND} -E make_directory
            ${PX4_BINARY_DIR}/etc/${init_dir}/airframes)
        foreach(airframe IN LISTS custom_airframes)
            get_filename_component(name ${airframe} NAME)
            list(APPEND PX4_CUSTOM_AIRFRAME_COPY_COMMANDS COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${airframe} ${PX4_BINARY_DIR}/etc/${init_dir}/airframes/${name})
        endforeach()
    endif()
    list(APPEND PX4_AIRFRAME_DEPENDENCIES ${selected_airframes})
    foreach(airframe IN LISTS selected_airframes)
        if(init_dir STREQUAL "init.d")
            get_filename_component(name ${airframe} NAME)
            configure_file(${airframe} ${PX4_AIRFRAME_METADATA_ROOT}/airframes/${name} COPYONLY)
        elseif(airframe MATCHES "_gz_")
            list(APPEND PX4_GZ_AIRFRAMES ${airframe})
        endif()
    endforeach()
endforeach()
# Common custom-airframe defaults use the same runtime paths as before.
if(CONFIG_BOARD_CUSTOM_AIRFRAMES)
    set(custom_defaults ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/custom_airframes/init.d/rc.development_airframe_defaults)
    list(APPEND PX4_CUSTOM_AIRFRAME_COPY_COMMANDS COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${custom_defaults} ${PX4_BINARY_DIR}/etc/init.d/rc.development_airframe_defaults)
    list(APPEND PX4_AIRFRAME_DEPENDENCIES ${custom_defaults})
endif()

# A changed collection must regenerate outputs even when a file was removed.
string(JOIN "\n" airframe_selection ${PX4_AIRFRAME_DEPENDENCIES})
file(CONFIGURE OUTPUT ${PX4_BINARY_DIR}/airframe_selection.txt CONTENT "${airframe_selection}\n" @ONLY)
list(APPEND PX4_AIRFRAME_DEPENDENCIES ${PX4_BINARY_DIR}/airframe_selection.txt)

set(PX4_HAS_SIMULINK OFF)
if(CONFIG_MODULES_SIMULINK_IO OR CONFIG_MODULES_SIMULINK_PARAMETERS)
    set(PX4_HAS_SIMULINK ON)
endif()
# Build-specific parameter defaults are applied before saved parameters load.
if(CONFIG_BOARD_CUSTOM_AIRFRAMES OR PX4_HAS_SIMULINK)
    configure_file(${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/custom_airframes/rc.development_defaults.in
        ${PX4_BINARY_DIR}/rc.development_defaults @ONLY)
    list(APPEND PX4_CUSTOM_AIRFRAME_COPY_COMMANDS COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${PX4_BINARY_DIR}/rc.development_defaults ${PX4_BINARY_DIR}/etc/init.d/rc.development_defaults)
    list(APPEND PX4_AIRFRAME_DEPENDENCIES ${PX4_BINARY_DIR}/rc.development_defaults)
endif()
if(NOT CONFIG_BOARD_STOCK_AIRFRAMES AND NOT CONFIG_BOARD_CUSTOM_AIRFRAMES)
    message(FATAL_ERROR "At least one airframe collection must be enabled")
endif()
if(CONFIG_BOARD_SIMULINK_MODE GREATER 0 AND NOT PX4_HAS_SIMULINK)
    message(FATAL_ERROR "Simulink startup requires the branch's Simulink module")
endif()

# Preserve actuator configuration metadata without linking the stock allocator.
if(PX4_HAS_SIMULINK AND NOT CONFIG_MODULES_CONTROL_ALLOCATOR)
    set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES
        ${PX4_SOURCE_DIR}/src/modules/control_allocator/module.yaml)
endif()
