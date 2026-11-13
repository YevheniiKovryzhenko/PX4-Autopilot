# PX4 and Simulink setup

This project uses a modified PX4 v1.16.1 firmware tree with custom Simulink support. Use the project installer and board configurations described below. The stock PX4 release does not include all of the required project changes.

| Document | Use it for |
| --- | --- |
| This setup guide | Installation, build selection, firmware configuration and troubleshooting |
| [Controller interface](CONTROLLER_INTEGRATION.md) | Pilot inputs, GCS commands, actuator outputs and telemetry payloads |

Identical copies are distributed in the firmware's `src/modules/simulink_parameters` folder and the aircraft project's `S2F-X200/PX4-Firmware Setup` folder. Keep the two documents together so their relative links work. Source paths below are relative to the firmware root; project paths are independent of the OneDrive mount location.

## 1. Install the firmware in Ubuntu

| Item | Location / requirement |
| --- | --- |
| Environment | Ubuntu 24.04 |
| Installer | Aircraft project: `S2F-X200/PX4-Firmware Setup/install.sh` |
| Firmware destination | `~/PX4/v1.16.1-mod/` |
| Installer actions | Clone the modified firmware, install dependencies and test-build the default SITL target |

Open an Ubuntu terminal in `PX4-Firmware Setup`, or copy `install.sh` into a convenient Ubuntu directory and open a terminal there. Run:

```bash
bash install.sh
```

After a successful installation and test build, continue with MATLAB setup. The installer is distributed with the aircraft project, not in the firmware documentation folder.

### Update an existing installation

Update the existing repository rather than rerunning the installer. From the firmware directory, run:

```bash
cd ~/PX4/v1.16.1-mod
git pull
git submodule update --init --recursive
```

| Command / next step | Purpose |
| --- | --- |
| `git pull` | Fetch and integrate updates to the currently checked-out firmware branch |
| `git submodule update --init --recursive` | Initialize missing submodules and check out the exact revisions recorded by the updated firmware, including nested submodules |
| Rebuild | Generate a new firmware binary; use the build command for your selected target below |
| Hardware upload | Flash the rebuilt firmware to apply the update on the autopilot |
| MATLAB setup | Rerun if the board/target or uORB message definitions used by Simulink changed |

Preserve or commit local changes before updating; resolve any conflicts before rebuilding. The recursive submodule command is safe to run after every pull and does not track arbitrary newer submodule revisions. Updating source files alone does not update a running SITL binary or flashed firmware. Existing saved parameter values remain in effect.

## 2. Configure the MATLAB support package

Install **UAV Toolbox Support Package for PX4 Autopilots** in MATLAB and launch its setup app.

1. Select the firmware directory installed by the script, normally `~/PX4/v1.16.1-mod` (expand `~` to your Ubuntu home directory if required by the file picker).
2. Select **Validate**. A firmware validation warning is expected for this modified tree; proceed with setup. Actual validation errors should be resolved before continuing.
3. Select the build configuration for your target using the table below.
4. Leave the option to **disable multicopter and fixed-wing modules** **unchecked**. The project board configuration already excludes the stock controllers needed to hand control over to Simulink.
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
| `default.px4board` | Required SITL setup entry point; hardware defaults in this project also use the copied minimal profile |

For hardware, select the explicit `simulinkctrl.px4board` file. The project's hardware defaults are not stock reference configurations.

### When to rerun setup

| Change | Required action |
| --- | --- |
| Different board or CMake target | Rerun the MATLAB setup app |
| Add/remove uORB message definitions used by Simulink | Rerun the MATLAB setup app |
| Change parameter definitions or defaults | Rebuild the firmware; setup need not be repeated solely for this change |

After setup, use Connected I/O, Run on Board or normal compilation as supported by the target.

## 3. Maintain custom parameters

The module supplies parameter definitions for the custom application and for PX4 components retained in a minimal build.

| File / configuration | Role |
| --- | --- |
| `parameters.yaml` | Custom Simulink parameter definitions |
| `stock_*.yaml` | Stock parameter definitions included when their owning controller is disabled |
| `simulinkctrl` profile | Excludes stock controller code from compilation, avoiding competing controllers and reducing firmware size/memory |
| `CONFIG_MODULES_SIMULINK_PARAMETERS=y` | Packages the required vehicle defaults and startup scripts through the shared CMake configuration |

### Rebuild after changing parameter definitions

**Rebuild after adding or changing parameters, defaults or metadata.** Editing YAML alone does not update an existing binary.

From the firmware root (`cd ~/PX4/v1.16.1-mod`), run only the command for your target:

| Target | Build command |
| --- | --- |
| SITL | `make px4_sitl_default` |
| CubeOrange | `make cubepilot_cubeorange_simulinkctrl` |
| CubeOrange+ | `make cubepilot_cubeorangeplus_simulinkctrl` |
| FMU-v6C | `make px4_fmu-v6c_simulinkctrl` |

If setup explicitly uses a hardware `default` profile, build that target instead, for example `make cubepilot_cubeorangeplus_default`.

| After rebuilding | What to do |
| --- | --- |
| SITL | Run the rebuilt binary |
| Hardware | Upload the rebuilt firmware; building does not flash it |
| Changed default value | Check existing saved values, which take precedence over the new default |

## 4. Modified firmware: topics, telemetry, and logging

For the current pilot-input mapping, accepted mode and arm/kill state, actuator outputs, GCS command formats and controller telemetry, see the [X200 PX4–Simulink interface overview](CONTROLLER_INTEGRATION.md).

### S2F-X200 hardware airframe

Select **S2F-X200** in the **Standard VTOL** airframe group, or set `SYS_AUTOSTART=22000` and reboot. Its startup file is `ROMFS/px4fmu_common/init.d/airframes/22000_s2f_x200_simulink`. It uses the standard VTOL vehicle defaults and exposes five motor channels and seven servo channels for the Simulink model:

| Logical channel | Model function |
| --- | --- |
| Motors [1, 4] | Lift propellers LP-1 through LP-4 |
| Motor 5 | Cruise propeller MP-1 |
| Servo 1 | Left outboard aileron |
| Servo 2 | Left flap |
| Servo 3 | Right flap |
| Servo 4 | Right outboard aileron |
| Servo 5 | Left elevator |
| Servo 6 | Right elevator |
| Servo 7 | Rudder |

| Responsibility | Owner / action |
| --- | --- |
| Control and mixing | Simulink publishes `actuator_motors` and `actuator_servos` directly |
| Physical PWM/CAN routing | Assign the corresponding motor/servo output functions in QGC; existing saved routing is preserved |
| Airspeed selection and VTOL land detection | Started by the X200 startup path |
| Stock flight controllers and allocator | Not started on this airframe |
| Generated application installation/startup | MATLAB support-package setup |

The channel list describes logical outputs, not a tuned PX4 allocation matrix.

| Build | Airframes packaged |
| --- | --- |
| Hardware `simulinkctrl`, or copied minimal `default` | X200 only, with matching single-airframe `airframes.xml` |
| Stock-control `simulink` | Normal airframe selection |
| SITL | Existing simulation airframes plus external-simulator X200 (22000); experimental Gazebo X200 (22001) comes from the Gazebo fork |

A GCS may display its own cached/bundled airframe catalog. Select the airframe ID directly if the custom entry is missing.

### Guidance trajectory location

The X200 presets select the trajectory location for the platform:

| Platform | `SMG_TRAJ_DIR` default | Trajectory directory |
| --- | --- | --- |
| Hardware | **1: SD card** | `/fs/microsd/Trajectories/`; copy `Trajectories` to the SD-card root |
| SITL | **0: local** | `./Trajectories/` relative to the PX4 working directory, normally the SITL `rootfs` directory |

The shared hardware airframe sets the SD-card default. The SITL airframe sources it first, then overrides the default to local storage. Place the trajectory files in the selected directory; the preset does not create them. Saved parameter values take precedence, so reset `SMG_TRAJ_DIR` to its airframe default or set it explicitly when changing platforms. A Gazebo-specific wrapper should source the SITL X200 preset or apply the same local override after the shared defaults.

### X200 disarming defaults

The shared X200 airframe sets these defaults for hardware, external-simulator SITL, and the Gazebo X200 variant:

| Parameter | Default | Effect |
| --- | --- | --- |
| `COM_DISARM_LAND` | -1 | Disable landing-triggered automatic disarming. |
| `COM_DISARM_PRFLT` | -1 | Disable preflight/no-takeoff automatic disarming. |
| `COM_KILL_DISARM` | 0 | Disarm without a configured delay after kill/lockdown. |
| `COM_LKDOWN_TKO` | 1 s | Shorten the early critical-failure disarm window; does not disable it. |

| Behavior | Meaning |
| --- | --- |
| Normal RC disarm | `COM_DISARM_MAN=1` permits airborne RC switch/stick/button disarm in multicopter manual-thrust modes such as Stabilized |
| FW or ordinary MAVLink disarm | The same airborne exception does not apply; Commander checks the request |
| Emergency force-disarm | `MAV_CMD_COMPONENT_ARM_DISARM`, `param1=0`, `param2=21196` |
| Automatic protection | Commander lockdown, spoolup battery/ESC failures and configured failsafes can still disarm or terminate outputs |
| Early critical-failure window | Includes `COM_SPOOLUP_TIME` plus `COM_LKDOWN_TKO`, measured from arming |

These defaults do not make disarming exclusively pilot-controlled. Existing saved values take precedence; reset the four settings to airframe defaults or set them explicitly when updating an already configured vehicle.

### Separate debug-array topics for Simulink

Each named topic below is an independent channel using the `DebugArray` structure.

| Structure / use | Contract |
| --- | --- |
| Payload | 58 `float32` values plus boot-relative microsecond timestamp |
| Identity fields | uORB `id` and 10-character `name`; the MAVLink streamer supplies fixed identities for the named channels |
| Simulink access | Publish/subscribe by the exact topic name; these are not interchangeable instance numbers |
| Application data | Producer and consumer must agree on the array layout |
| X200 GCS output | `simulink_outbound`; three extra outbound channels are available but unused |
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

### MAVLink streaming to the ground station

Custom Simulink telemetry is available on **every MAVLink profile**. The profile selects the standard telemetry preset; `SM_MAV` independently selects which custom topics to send on all links.

| Setting | Purpose | X200 default |
| --- | --- | --- |
| `MAV_0_MODE` | Standard telemetry preset for the primary hardware link | **15: Simulink**, a lean status/navigation preset with configurable standard streams |
| `SM_MAVSTD` | Select additional standard messages within the Simulink preset | See the standard-stream table below |
| `SM_MAV` | Select custom Simulink topics on every link | **511: all custom topics enabled** |
| Custom rate parameters | Set each custom stream's target frequency; names are in the topic table above | **20 Hz** |
| `MAV_PROTO_VER` | MAVLink wire protocol | **2**, required for `DEBUG_FLOAT_ARRAY` |
| `MAV_n_RATE` | Overall bandwidth budget for link `n` | Depends on the link configuration |

SITL selects the Simulink preset through its startup script. Other profiles, including Normal, Onboard, Config and Companion, keep their own standard telemetry while using the same custom-topic selection.

| Desired custom telemetry | `SM_MAV` value |
| --- | --- |
| Disabled | **0** |
| X200 GCS output only (`simulink_outbound`) | **8** |
| All custom topics | **511** |
| Any other combination | Sum the bit values for the desired topics, using the table above |

The general firmware default is 255; X200 overrides it to 511. A selected stream also needs a positive rate. Restart the affected links or reboot after changing the automatic stream settings. Explicit GCS stream requests can override the configured rates.

**What the ground station receives**

| Item | Interface |
| --- | --- |
| MAVLink message | `DEBUG_FLOAT_ARRAY` (message 350), containing 58 float values |
| Active X200 output | `simulink_outbound`: `array_id=100`, `name="outbound"` |
| Spare output channels | `simulink_outbound_1`, `_2`, `_3`; available but unused by the current model |
| Channel identification | PX4 assigns each named topic's fixed ID and name when transmitting; the GCS selects the channel by `array_id` |
| Fields to populate in Simulink | `timestamp` and `data`; no need to assign `id` or `name` |

The [interface overview](CONTROLLER_INTEGRATION.md) describes the X200 payload and reserves the guidance channels. PX4's outgoing ID/name assignment does not change the original uORB or logged fields.

Telemetry is sent only when new topic data is available. Publication rate and link bandwidth can reduce the achieved rate; these streaming settings do not change the controller's sample time or ULog recording rate.

### Automatic ULog topic logging

Logging has three independent decisions: enable the profile, select topics, and run a recording session.

| Setting | Role | Useful values |
| --- | --- | --- |
| `SDLOG_PROFILE` | Include the Simulink topic profile (bit 12, value 4096) | **4097**: standard + Simulink (X200 default); **4227**: SITL profile 131 + Simulink |
| `SM_LOG` | Select custom topic groups | **511**: all nine groups (default) |
| Interval parameters | Limit the recording frequency of selected groups | See table below |
| `SDLOG_MODE` | Decide when recording runs | See session table below |
| Custom `etc/logging/logger_topics.txt` | A nonempty file replaces normal profile selection | Include desired topics explicitly when using it |

`SM_LOG` alone does not enable the Simulink profile. The general SITL profile 131 does not include bit 12; X200 supplies its own default. Data is recorded only while the logger/session is active and the topic is publishing.

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

### Standard telemetry rate controls in Simulink mode

| Control | Behavior |
| --- | --- |
| Scope | Selected standard PX4 messages in the Simulink MAVLink preset |
| Selection | `SM_MAVSTD=2015` enables all defined bits below; bit 5 is unused |
| Frequency | Each selected message uses its own rate parameter |
| Clear bit / zero rate | Skip that override; another mechanism may still configure the stream |
| Apply changes | Restart the MAVLink instance or reboot |

These controls are separate from custom-topic streaming (`SM_MAV`) and recording (`SM_LOG`).

| `SM_MAVSTD` bit | MAVLink stream | Rate parameter | Default | Range |
| --- | --- | --- | --- | --- |
| 0 | `ATTITUDE` | `SM_MAV_ATT_R` | 15 Hz | [0, 250] Hz |
| 1 | `ATTITUDE_QUATERNION` | `SM_MAV_ATTQ_R` | 10 Hz | [0, 250] Hz |
| 2 | `ATTITUDE_TARGET` | `SM_MAV_ATTT_R` | 2 Hz | [0, 250] Hz |
| 3 | `LOCAL_POSITION_NED` | `SM_MAV_LP_R` | 10 Hz | [0, 250] Hz |
| 4 | `POSITION_TARGET_LOCAL_NED` | `SM_MAV_LPT_R` | 10 Hz | [0, 250] Hz |
| 6 | `GLOBAL_POSITION_INT` | `SM_MAV_GPI_R` | 5 Hz | [0, 50] Hz |
| 7 | `POSITION_TARGET_GLOBAL_INT` | `SM_MAV_GPT_R` | 1 Hz | [0, 50] Hz |
| 8 | `SERVO_OUTPUT_RAW_0` | `SM_MAV_SRV_R` | 10 Hz | [0, 250] Hz |
| 9 | `ESC_STATUS` | `SM_MAV_ESC_R` | 1 Hz | [0, 50] Hz |
| 10 | `RAW_RPM` | `SM_MAV_RPM_R` | 2 Hz | [0, 100] Hz |

| Source file | Definitions |
| --- | --- |
| `src/modules/mavlink/module.yaml` | Custom and standard telemetry controls |
| `src/modules/logger/module.yaml` | Logging masks and intervals |
| `msg/DebugArray.msg` | Shared debug-array schema and routing IDs |

## Troubleshooting

| Symptom | Check / action |
| --- | --- |
| R2026b startup errors, connection failure or Run on Board failure | Before a clean setup, preserve custom files and remove the entire `src/modules/px4_simulink_utils` directory, then rerun MATLAB setup. An empty directory still prevents setup from copying the module. |
| Missing `simulink_custom_message.h` | Ensure the message definition is present and registered, then rerun setup. Preserve custom work before restoring clean project firmware if generated integration files are stale. |
| Missing `MC_ROLL_P`, `HTE_VXY_THR` or similar tuning parameters | These can be absent in a minimal build because their stock controllers/estimator are excluded. Required shared definitions come from `simulink_parameters`; do not re-enable controllers solely to restore private tuning settings. |
| Missing `rc.mc_defaults` or similar startup script | Check `CONFIG_MODULES_SIMULINK_PARAMETERS=y` and shared startup CMake packaging, then rebuild. Vehicle defaults are needed even when stock controllers are excluded. |
| Guidance cannot find its trajectory in SITL | Supply the selected file under local `./Trajectories/`. X200 uses `SMG_TRAJ_DIR=0`; saved SD-card settings can override this. Set `SMG_TRAJ_DIR=0` for an existing session if needed. |

**Verify the R2026b support module after setup**

The `px4_simulink_utils` directory should contain `CMakeLists.txt`, `Kconfig`, `px4_simulink_utils.cpp`, `px4_simulink_utils.h`, and `px4_simulink_utils_params.c`. Git does not track empty directories, so reverting files may leave an empty directory that must still be removed before a clean setup.

Optional stock defaults use `param set-default -s` in `rc.vtol_defaults`, allowing excluded-controller parameters to be skipped quietly. The trajectory directory setting selects a location; it does not create trajectory files.
