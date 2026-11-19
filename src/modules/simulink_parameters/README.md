# Simulink parameters module and common PX4 setup

`simulink_parameters` provides this branch's project parameter definitions and the stock parameter metadata needed by retained components when their owning controllers are excluded. The generated control application is supplied by the existing MATLAB support-package workflow. [Controller integration](CONTROLLER_INTEGRATION.md) documents the existing project/model interface; vehicle setup belongs in the airframe guides linked below.

Both modified branches use the same startup modes, airframe selection flags and per-link telemetry settings. PX4 v1.16.1 retains its existing Simulink support-package integration and parameter module; v1.17.0 uses the custom `px4_simulink_io` toolbox and `simulink_io` runner. Model-specific controller parameters stay with their project.

Vehicle-specific builds, preset parameters, output assignments and commissioning are documented separately:

| Airframe | Hardware preset IDs | Guide |
| --- | --- | --- |
| R1D1 CAN avionics quadrotor | 22010 | [R1D1](../../../Documentation/airframes/r1d1.md) |
| Holybro X500 V2 Simulink variants | 4030, 4031 | [X500](../../../Documentation/airframes/x500.md) |
| S2F-X200 external-control VTOL | 22000 | [X200](../../../Documentation/airframes/x200.md) |

## Build and startup

Run these commands from the selected PX4 checkout. Replace `cubeorangeplus` with `cubeorange` or use `px4_fmu-v6c` for their corresponding Simulink profiles.

| Target | Initial `SIMULINK_MODE` | Controllers | Hardware airframes |
| --- | --- | --- | --- |
| `make cubepilot_cubeorangeplus_default` | 0 | Stock PX4 | Stock |
| `make cubepilot_cubeorangeplus_simulink` | 1 | Stock PX4 + Simulink tasks | Stock + custom |
| `make cubepilot_cubeorangeplus_simulinkctrl` | 2 | External control; stock controllers and allocator omitted | Custom only |
| `make px4_sitl_simulink` | 1 | Stock PX4 + Simulink tasks | Stock + custom simulation airframes |
| `make px4_sitl_simulinkctrl` | 2 | External control | Stock + custom simulation airframes |

Hardware `default.px4board` files follow their upstream release. The existing v1.16.1 SITL default configuration is intentionally retained. Upload the resulting target's `.px4` package using the normal PX4 uploader or QGroundControl custom firmware workflow.

`SIMULINK_MODE` is a saved System parameter; reboot after changing it. Zero runs stock vehicle startup and starts no additional Simulink tasks. One runs stock vehicle startup and then the branch's Simulink tasks. Two starts the land detector (and airspeed selector for fixed-wing/VTOL), followed by the Simulink tasks, while omitting stock controller and allocator startup. In v1.17.0 these tasks are `simulink_io` and `simulink_guidance`; in v1.16.1 the shared startup starts `simulink_guidance`, and the existing support-package deployment supplies the generated application. Mode zero cannot restore controllers absent from a minimal binary: choose a full build when testing stock control.

Board configuration flags make packaging explicit, without interpreting target names:

| Flag | Purpose |
| --- | --- |
| `CONFIG_BOARD_STOCK_AIRFRAMES=y/n` | Include/exclude stock hardware and simulation airframe scripts and their selection metadata |
| `CONFIG_BOARD_CUSTOM_AIRFRAMES=y/n` | Include/exclude development airframes and common defaults |
| `CONFIG_BOARD_SIMULINK_MODE=0/1/2` | Initial parameter default for this build |

Enabling the branch's Simulink module also packages vehicle startup scripts and defaults, even when their usual owning controllers are excluded. Minimal builds therefore retain scripts such as `rc.mc_defaults`. Actuator configuration metadata is retained without linking the allocator task. Builds do not select an airframe or change `SYS_AUTOSTART`. Select the desired preset in QGroundControl and reboot; saved user parameters continue to take precedence over defaults.

## Adding custom airframes

Place hardware scripts in `ROMFS/px4fmu_common/custom_airframes/init.d/airframes/` and simulation scripts in `ROMFS/px4fmu_common/custom_airframes/init.d-posix/airframes/`, using the usual numeric ID and PX4 airframe header. CMake discovers these files automatically; no filename lists or parser changes are required. Custom Gazebo scripts supplied by `Tools/simulation/gz/airframes/` are also discovered automatically for Gazebo builds. Keep IDs unique within each collection and make custom scripts self-contained or dependent only on other enabled scripts.

The stock/custom flags select entire collections independently of Simulink mode or build name. Both custom X500 presets are included when custom airframes are enabled; select the one compatible with the compiled controllers and `SIMULINK_MODE`. The standard PX4 parser and flat runtime script paths are unchanged.

## Custom airframes in QGroundControl

Upload the selected `.px4` file through QGroundControl’s **Custom Firmware** option, then fully close and restart QGC to see the new airframes. Select the desired preset and apply/reboot the flight controller.

## Shared custom-airframe parameters

These defaults come from `ROMFS/px4fmu_common/custom_airframes/init.d/rc.development_airframe_defaults`; stock airframes do not source this script. They apply when the selected custom airframe loads, and saved values take precedence.

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `COM_DISARM_LAND` | -1 | Disable automatic disarming after landing |
| `COM_DISARM_PRFLT` | -1 | Disable the preflight automatic-disarm timer |
| `COM_KILL_DISARM` | 0 | No configured delay before disarming on kill |
| `COM_LKDOWN_TKO` | 1 s | Allow attitude-failure disarming during the first second after takeoff, in applicable flight modes |
| `MAV_0_MODE` | 15 | Simulink telemetry: Normal messages plus selected parameter rate overrides |

These settings do not eliminate other Commander disarming/failsafe paths. Review kill, lockdown and disarming behavior for the installed vehicle. `MAV_0_MODE` controls telemetry; `SIMULINK_MODE` controls task startup. Selecting telemetry mode 15 does not start a Simulink controller.

## Gazebo assets

Both branches pin the same existing experimental model revision, `5b6fdc041f24d7a570b162dade68e083426f4e7f`, from `v1.16.1-custom-frames` in the Gazebo-model fork. The historical branch name does not select the PX4 firmware version. Use `git submodule update --init --recursive` to obtain the pinned models. Launch commands and model limitations belong in each airframe guide.

## Independent MAVLink links

Custom hardware airframes preset `MAV_0_MODE=15` for the configured telemetry link. This custom Simulink MAVLink profile starts with the complete Normal-profile message set, including `DISTANCE_SENSOR`, `OBSTACLE_DISTANCE` and `SCALED_PRESSURE`, then applies the selected standard-stream rate overrides and custom streams. Other profiles retain their existing behavior. This telemetry profile is independent of `SIMULINK_MODE` and does not start a Simulink controller. Saved profile settings take precedence; select mode 15 in QGC if an existing setup still uses mode 0.

Custom Simulink streams can accompany any standard MAVLink profile, including normal telemetry and USB. `MAV_n_SM_MASK` selects custom streams for link profile `n`; `MAV_n_SM_STD` selects the standard-stream overrides used by MAVLink mode 15 (Simulink). A mask of `-1` inherits `SM_MAV` or `SM_MAVSTD`; zero disables that custom selection. Each `MAV_n_SM_Ck_R` or `MAV_n_SM_Sk_R` rate is in Hz: `-1` inherits the corresponding global rate, zero disables the selected stream, and a positive value requests that rate. Reboot/restart the link after changing these settings. Actual throughput remains limited by source updates and the link bandwidth budget.

| Link | Profile `n` |
| --- | --- |
| Configured `MAV_0`, `MAV_1`, `MAV_2` serial/network links | 0, 1, 2 respectively |
| Hardware USB | 3 |
| SITL GCS, offboard, payload, gimbal, SIH | 0, 1, 2, 3, 4 respectively |
| Manually started link | Supply `mavlink start -I n ...` (0–5); omission uses global settings |

These indices identify configuration profiles, independent of runtime MAVLink instance start order. For example, set `MAV_0_SM_MASK=0` to suppress custom streams on TELEM1 and `MAV_3_SM_MASK=511` to select all custom streams on USB. Set `MAV_3_SM_C3_R=50` to request USB's primary outbound stream at 50 Hz without changing other links.

| `k` | Custom stream (`Ck_R`) | Global rate |
| --- | --- | --- |
| 0 | `COMPANION_GUIDANCE_OUTBOUND` | `SMG_MAVOUT_R` |
| 1 | `COMPANION_GUIDANCE_INBOUND` | `SMG_MAVIN_R` |
| 2 | `SIMULINK_INBOUND` | `SM_MAVIN_R` |
| 3 | `SIMULINK_OUTBOUND` | `SM_MAVOUT_0_R` |
| 4–6 | `SIMULINK_OUTBOUND_1..3` | `SM_MAVOUT_1_R..3_R` |
| 7 | `SIMULINK_GUIDANCE` | `SMG_MAV_R` |
| 8 | `SIMULINK_INBOUND_CTRL` | `SM_MAVCTRL_R` |

| `k` | Standard stream (`Sk_R`, mode 15) | Global rate |
| --- | --- | --- |
| 0 | `ATTITUDE` | `SM_MAV_ATT_R` |
| 1 | `ATTITUDE_QUATERNION` | `SM_MAV_ATTQ_R` |
| 2 | `ATTITUDE_TARGET` | `SM_MAV_ATTT_R` |
| 3 | `LOCAL_POSITION_NED` | `SM_MAV_LP_R` |
| 4 | `POSITION_TARGET_LOCAL_NED` | `SM_MAV_LPT_R` |
| 5 | `GLOBAL_POSITION` | `SM_MAV_GP_R` |
| 6 | `GLOBAL_POSITION_INT` | `SM_MAV_GPI_R` |
| 7 | `POSITION_TARGET_GLOBAL_INT` | `SM_MAV_GPT_R` |
| 8 | `SERVO_OUTPUT_RAW_0` | `SM_MAV_SRV_R` |
| 9 | `ESC_STATUS` | `SM_MAV_ESC_R` |
| 10 | `RAW_RPM` | `SM_MAV_RPM_R` |

Masks use the same bit numbers as the tables. A cleared standard override bit leaves the inherited Normal-profile rate in place; it does not suppress every stock stream with that name. Custom MAVLink messages still require the matching ground-side dialect. DroneCAN device discovery and configuration are separate from this MAVLink stream policy.

## Toolbox and installation

The v1.17.0 custom toolbox is `/home/jack/GitHub/px4_simulink_io`; its `px4/` folder mirrors matching firmware integration files. Export generated model code into `src/modules/simulink_io/generated_code`, then build the selected target. Keep the runner and generator versions together. v1.16.1 continues using its existing support-package deployment and `simulink_parameters`; the common firmware startup does not invent a generated controller task when no model is deployed.

The installation script belongs in the toolbox at `tools/install_mod.sh`, mirrored from `/home/jack/Jack_Scripts/ubuntu/PX4/install_mod.sh`. Run it deliberately: it offers to replace an existing checkout, clones the modified branch with pinned submodules, installs dependencies and test-builds SITL. The mirrored installer currently targets `v1.17.0-mod`; use the existing v1.16.1 checkout/project installation for the support-package workflow. It is not part of the PX4 source overlay.

PX4 owns input calibration/source selection, accepted mode, arming, estimation, output drivers and transport. External controllers should use `manual_control_setpoint` for the selected RC/joystick source and `vehicle_status` for accepted navigation/arming state, then publish the output topic expected by the selected driver. Controller-specific stick mapping, state transitions and tuning remain in the model. Avoid multiple controllers publishing competing actuator demands.

## MATLAB support-package deployment

Install **UAV Toolbox Support Package for PX4 Autopilots** in MATLAB and launch its setup app.

1. Select the firmware directory installed by the script, normally `~/PX4/v1.16.1-mod` (expand `~` to your Ubuntu home directory if required by the file picker).
2. Select **Validate**. A firmware validation warning is expected for this modified tree; proceed with setup. Actual validation errors should be resolved before continuing.
3. Select the build configuration for your target using the table below.
4. Leave the option to **disable multicopter and fixed-wing modules** **unchecked**. The selected board profile determines which stock controllers are included; the setup app should not independently remove them.
5. Complete the remaining setup steps for your target.

### Select the build configuration

Paths in this table are relative to the firmware directory.

| Target | Configuration to select in the setup app |
| --- | --- |
| PX4 Host Target / SITL | Select the `default` configuration. The app does not allow a custom board configuration for this target. |
| CubeOrange | Select the custom file `boards/cubepilot/cubeorange/simulinkctrl.px4board`. |
| CubeOrange+ | Select the custom file `boards/cubepilot/cubeorangeplus/simulinkctrl.px4board`. |
| PX4 FMU-v6C | Select the custom file `boards/px4/fmu-v6c/simulinkctrl.px4board`. |

| Board profile | Purpose |
| --- | --- |
| `simulinkctrl.px4board` | Minimal Simulink-controlled build: excludes stock controllers and selected unnecessary drivers; retains guidance and custom parameters |
| `simulink.px4board` | Stock controllers plus project Simulink features |
| `default.px4board` | Required SITL setup entry point; hardware default profiles remain stock |

For hardware, select the explicit `simulinkctrl.px4board` file. Hardware `default.px4board` files remain stock; select the explicit custom profile.

### When to rerun setup

| Change | Required action |
| --- | --- |
| Different board or CMake target | Rerun the MATLAB setup app |
| Add/remove uORB message definitions used by Simulink | Rerun the MATLAB setup app |
| Change parameter definitions or defaults | Rebuild the firmware; setup need not be repeated solely for this change |

After setup, use Connected I/O, Run on Board or normal compilation as supported by the target.

## Maintain parameter definitions

The module supplies parameter definitions for the custom application and for PX4 components retained in a minimal build.

| File / configuration | Role |
| --- | --- |
| `parameters.yaml` | Custom Simulink parameter definitions |
| `stock_*.yaml` | Stock parameter definitions included when their owning controller is disabled |
| `simulinkctrl` profile | Excludes stock controller code from compilation, avoiding competing controllers and reducing firmware size/memory |
| `CONFIG_MODULES_SIMULINK_PARAMETERS=y` | Packages the required vehicle defaults and startup scripts through the shared CMake configuration |


Rebuild after changing parameter definitions or metadata. MATLAB model bindings do not create firmware parameter definitions. Existing saved parameter values can override updated defaults. Use the build commands above for the selected target.
## Debug-array topic contract

Each named topic below is an independent channel using the `DebugArray` structure.

| Structure / use | Contract |
| --- | --- |
| Payload | 58 `float32` values plus boot-relative microsecond timestamp |
| Identity fields | uORB `id` and 10-character `name`; the MAVLink streamer supplies fixed identities for the named channels |
| Simulink access | Publish/subscribe by the exact topic name; these are not interchangeable instance numbers |
| Application data | Producer and consumer must agree on the array layout |
| Guidance channels | Reserved for guidance state, references and companion traffic; see [topic ownership](CONTROLLER_INTEGRATION.md#guidance-topic-ownership) |

Publishing one channel does not update the others. Stock controllers do not populate these project-specific topics.

| uORB topic | MAVLink `array_id` | MAVLink name | `SM_MAV` bit | Streaming rate parameter |
| --- | --- | --- | --- | --- |
| `simulink_inbound` | 200 | `inbound` | 2 | `SM_MAVIN_R` |
| `simulink_inbound_ctrl` | 250 | `in_ctrl` | 8 | `SM_MAVCTRL_R` |
| `simulink_outbound` | 100 | `outbound` | 3 | `SM_MAVOUT_0_R` |
| `simulink_outbound_1` | 101 | `outbound1` | 4 | `SM_MAVOUT_1_R` |
| `simulink_outbound_2` | 102 | `outbound2` | 5 | `SM_MAVOUT_2_R` |
| `simulink_outbound_3` | 103 | `outbound3` | 6 | `SM_MAVOUT_3_R` |
| `simulink_guidance` | 150 | `guidance` | 7 | `SMG_MAV_R` |
| `companion_guidance_inbound` | 121 | `compg_in` | 1 | `SMG_MAVIN_R` |
| `companion_guidance_outbound` | 120 | `compg_out` | 0 | `SMG_MAVOUT_R` |


Named custom channels are carried in MAVLink `DEBUG_FLOAT_ARRAY` and need MAVLink 2. The streamer supplies their fixed channel ID/name; a model normally populates `timestamp` and `data`. See [Controller integration](CONTROLLER_INTEGRATION.md) for the existing project payload and topic ownership. Global and per-link rate controls are documented above.
## ULog configuration

Logging has three independent decisions: enable the profile, select topics, and run a recording session.

| Setting | Role | Useful values |
| --- | --- | --- |
| `SDLOG_PROFILE` | Include the Simulink topic profile (bit 12, value 4096) | **4097**: standard + Simulink; **4227**: SITL profile 131 + Simulink |
| `SM_LOG` | Select custom topic groups | **511**: all nine groups (default) |
| Interval parameters | Limit the recording frequency of selected groups | See table below |
| `SDLOG_MODE` | Decide when recording runs | See session table below |
| Custom `etc/logging/logger_topics.txt` | A nonempty file replaces normal profile selection | Include desired topics explicitly when using it |

`SM_LOG` alone does not enable the Simulink profile. The general SITL profile 131 does not include bit 12; select a profile that includes it when these topics are needed. Data is recorded only while the logger/session is active and the topic is publishing.

| `SM_LOG` bit | Topics | Interval parameter | Default |
| --- | --- | --- | --- |
| 0 | `sim_control_status` | `SM_CTRLST_LOG_T` | 20 ms |
| 1 | `simulink_inbound`, `simulink_inbound_ctrl` | `SM_INBD_LOG_T` | 20 ms |
| 2 | `simulink_outbound`, `simulink_outbound_1`, `simulink_outbound_2`, `simulink_outbound_3` | `SM_OUTBD_LOG_T` | 20 ms |
| 3 | `companion_guidance_inbound` | `SMG_IN_LOG_T` | 20 ms |
| 4 | `companion_guidance_outbound` | `SMG_OUT_LOG_T` | 20 ms |
| 5 | `sim_guidance_request` | `SMG_REQ_LOG_T` | 0 ms |
| 6 | `sim_guidance_status` | `SMG_ST_LOG_T` | 0 ms |
| 7 | `simulink_guidance` | `SMG_LOG_T` | 20 ms |
| 8 | `sim_guidance_trajectory` | `SMG_TR_LOG_T` | 20 ms |

| Interval setting | Effect |
| --- | --- |
| Allowed range | **[0, 10000] ms**; reboot after changes |
| 0 ms | No interval limit; this does not disable the topic |
| 20 ms | Up to 50 Hz |
| 100 ms | Up to 10 Hz |
| 1000 ms | Up to 1 Hz |
| Exclude a topic group | Clear its `SM_LOG` bit |

Actual recording is limited by publication rate, logger scheduling and storage. Several topics share an interval. Logging and MAVLink streaming are independent; neither changes the controller's sample time.

`SDLOG_MODE` determines when a logging session runs:

| Value | Logging session |
| --- | --- |
| -1 | Disabled |
| 0 | Armed until disarm (parameter default) |
| 1 | Boot until disarm (SITL startup default) |
| 2 | Boot until shutdown |
| 3 | While manual input AUX1 is above 30% |
| 4 | First arming until shutdown |

`SDLOG_BOOT_BAT=1` additionally prevents boot-start logging without detected battery power; its default is 0. Reboot after changing logging mode/profile settings. Selecting topics does not by itself start a recording session.

## Troubleshooting

| Symptom | Check / action |
| --- | --- |
| R2026b startup errors, connection failure or Run on Board failure | Before a clean setup, preserve custom files and remove the entire `src/modules/px4_simulink_utils` directory, then rerun MATLAB setup. An empty directory still prevents setup from copying the module. |
| Missing `simulink_custom_message.h` | Ensure the message definition is present and registered, then rerun setup. Preserve custom work before restoring clean project firmware if generated integration files are stale. |
| Missing `MC_ROLL_P`, `HTE_VXY_THR` or similar tuning parameters | These can be absent in a minimal build because their stock controllers/estimator are excluded. Required shared definitions come from `simulink_parameters`; do not re-enable controllers solely to restore private tuning settings. |
| Missing `rc.mc_defaults` or similar startup script | Check `CONFIG_MODULES_SIMULINK_PARAMETERS=y` and shared startup CMake packaging, then rebuild. Vehicle defaults are needed even when stock controllers are excluded. |
| Guidance cannot find its trajectory in SITL | Supply the selected file under local `./Trajectories/`. Simulation airframes can select `SMG_TRAJ_DIR=0`; saved SD-card settings can override this. Set `SMG_TRAJ_DIR=0` for an existing session if needed. |

**Verify the R2026b support module after setup**

The `px4_simulink_utils` directory should contain `CMakeLists.txt`, `Kconfig`, `px4_simulink_utils.cpp`, `px4_simulink_utils.h`, and `px4_simulink_utils_params.c`. Git does not track empty directories, so reverting files may leave an empty directory that must still be removed before a clean setup.

Optional stock defaults use `param set-default -s` in `rc.vtol_defaults`, allowing excluded-controller parameters to be skipped quietly. The trajectory directory setting selects a location; it does not create trajectory files.

## Items to complete

- Verify independent MAVLink rates with the actual GCS/toolbox and connection bandwidth.
- Verify each project's generated-controller deployment, output ownership and failsafes before using startup mode 2.
- Complete the installation-specific checks in the selected airframe guide.
