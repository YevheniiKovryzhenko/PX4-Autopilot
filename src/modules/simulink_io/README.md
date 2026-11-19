# Simulink I/O module and common PX4 setup

`simulink_io` runs the model exported by the `px4_simulink_io` toolbox. The handwritten task owns scheduling and performance counters. The generated `simulink_model_wrapper.h` owns model initialization, parameter refresh, stepping, and termination. Model names, uORB topics, and sample periods require no edits to the PX4 task.

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

## Runtime behavior

```sh
simulink_io start
simulink_io status
simulink_io stop
```

The task initializes the model once, calls its combined step function at the compiled base period, and calls termination on a normal stop. `status` reports the model name, configured period, observed intervals, execution time, and overruns. An overrun resets the scheduling baseline without replaying missed steps. Scheduling uses PX4's high-resolution clock; it does not guarantee hard real-time execution or synchronize model steps with individual sensor publications.

The task imposes no arming, offboard, takeoff, or estimator-validity conditions. Those decisions belong to the generated model and the PX4 modules it interacts with. Task priority (`SCHED_PRIORITY_DEFAULT`) and stack size (8192 bytes) are platform settings in the handwritten module; model computation must fit those resources and its selected period.

## Generated model contract

The toolbox reads the compiled interface from `codeInfo.mat`, including the resolved base period and entry-point names. It supports fixed-step, single-tasking, nonreusable C models with one combined `void step(void)` entry point; slower rates can execute inside that generated step. Periods must be representable as whole microseconds. Unsupported interfaces fail during export.

## Model configuration and generation workflow

Configure the model manually in Simulink or use this optional settings helper once:

```matlab
px4io.deployment.configureModel('myModel');
```

The helper selects fixed-step, single-tasking, nonreusable C generation and enables internal/external startup memory initialization, including initialization settings on referenced models. It preserves the selected solver and sample time. It does not save the model, generate code, or export anything. Save the affected model settings in Simulink; ordinary model edits and PX4 updates do not require repeating the configuration step.

Keep the model's Post-code generation command configured as:

```matlab
px4io.px4API.runPostCodeGen(buildInfo);
```

For everyday work, edit the model, press **Generate Code**, let the callback export, and rebuild PX4. `slbuild('myModel')` is the command-line alternative to the button, not another required step. The callback automatically collects parameter bindings, reads the compiled sample period and entry points, generates the adapter and uORB glue, and stages the complete export before replacing `generated_code`.

| Change | Action |
| --- | --- |
| Model logic, topics, parameters, or sample time | Generate Code, then rebuild PX4 |
| New model or changed code-generation settings | Configure and save settings, then Generate Code |
| PX4 code with unchanged message interfaces | Rebuild PX4 |
| PX4 message definitions | Refresh Simulink's API/buses before Generate Code, then rebuild PX4 |
| Generator/runner interface | Use matching versions, regenerate, then rebuild PX4 |

After editing PX4 `.msg` definitions in an existing MATLAB session, run `px4io.px4API.forceGenerateAll()` before generating model code. This refreshes message interfaces; it does not change model settings. The post-generation callback cannot retroactively update bus layouts used to compile the model.

Export validates settings and the compiled interface without changing them. An export failure leaves the previous deployed code in place. CMake detects changes to the generated source file set. Generate persistent changes through the toolbox rather than editing exported files. Export does not compile or flash PX4.

## Messages and parameters at runtime

The generated adapter refreshes cached parameters before each model step. PX4 parameter handles are resolved at initialization; values refresh on parameter update notifications. Parameters must be defined in the selected firmware with matching types. Model bindings alone do not create PX4 parameter definitions.

uORB reads return the last received message, initially zero, and retain it when there is no new publication. Model logic should inspect timestamps when freshness matters. Message initializers can set floating fields to NaN, and writers publish the fields provided by the model. Read buffers reset on initialization, and normal termination releases subscriptions. The generated model owns the meaning and use of these messages, including requests sent to `simulink_guidance` or commander.

## Diagnostics

- Missing adapter members such as `period_us` indicate a mismatched generator and runner or stale generated code. Regenerate with the matching toolbox.
- Unsupported interface or restart-setting errors require correcting model configuration before generating again.
- Missing parameter warnings require checking the model binding and firmware's parameter definitions.
- Overruns indicate that execution did not finish before the next deadline. Use execution-time and interval counters to assess timing.
- SITL lockstep needs simulator time to advance. A standalone module lifecycle test does not measure flight-loop timing.

## Guidance and control ownership

A model that publishes position/trajectory setpoints and requests stock PX4 control needs startup mode 1. A model that publishes `actuator_motors` and `actuator_servos` directly can use mode 2 and must provide the replacement control/allocation path. The output drivers still implement QGC function-to-port assignment and arming limits; the model and allocator must not compete to publish actuator demands. The hardware control variants retain PX4IO, DShot and FMU PWM output support.

`simulink_guidance` has separate enable, output-topic, and trajectory-location parameters. `SMG_OUT_TYPE` is a bitmask: 1 publishes `trajectory_setpoint`, 2 publishes `sim_guidance_trajectory`, and 3 publishes both. Ensure that the generated model subscribes to the selected topic and that independent modules do not compete to publish the same setpoint. `SMG_TRAJ_DIR=0` selects local SITL trajectories; 1 selects the hardware SD card. Starting the module alone does not enable updates when `SMG_EN=0`.


## Items to complete

- Verify independent MAVLink rates with the actual GCS/toolbox and connection bandwidth.
- Verify each project's generated-controller deployment, output ownership and failsafes before using startup mode 2.
- Complete the installation-specific checks in the selected airframe guide.
