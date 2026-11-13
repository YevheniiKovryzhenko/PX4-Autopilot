# PX4 and Simulink setup

This project uses a modified PX4 v1.16.1 firmware tree with custom Simulink
support. Use the project installer and board configurations described below.
The stock PX4 release does not include all of the required project changes.

## 1. Install the firmware in Ubuntu

Run the project installer in your Ubuntu 24.04 instance. The script is located
in the project files at:

```text
Research/Simulation Models/S2F-X200/Simulink Model R2026b V1/install_px4.sh
```

Copy `install_px4.sh` into Ubuntu, then run it from any directory:

```bash
./install_px4.sh
```

The installer clones the required modified firmware into
`~/PX4/v1.16.1-mod/`, installs the dependencies, and performs a test compilation
of the default SITL target. If the script completes successfully, the initial
firmware installation and test build are complete. Continue with MATLAB setup.

## 2. Configure the MATLAB support package

Install **UAV Toolbox Support Package for PX4 Autopilots** in MATLAB and launch
its setup app.

1. Select the firmware directory installed by the script, for example
   `/home/jack/PX4/v1.16.1-mod`.
2. Select **Validate**. A firmware validation warning is expected for this
   modified tree; proceed with setup. Actual validation errors should be
   resolved before continuing.
3. Select the build configuration for your target using the table below.
4. Leave the option to **disable multicopter and fixed-wing modules**
   **unchecked**. The project board configuration already excludes the stock
   controllers needed to hand control over to Simulink.
5. Complete the remaining setup steps for your target.

### Select the build configuration

Paths in this table are relative to the firmware directory.

| Target | Configuration to select in the setup app |
| --- | --- |
| PX4 Host Target / SITL | Select the `default` configuration. The app does not allow a custom board configuration for this target. |
| CubeOrange | Select the custom file `boards/cubepilot/cubeorange/simulinkctrl.px4board`. |
| CubeOrange+ | Select the custom file `boards/cubepilot/cubeorangeplus/simulinkctrl.px4board`. |
| PX4 FMU-v6C | Select the custom file `boards/px4/fmu-v6c/simulinkctrl.px4board`. |

The `.px4board` files provide the board configuration consumed by the PX4
CMake build:

- **`simulinkctrl.px4board`** is the minimal configuration for control from
  Simulink. It excludes stock control modules and selected unnecessary drivers
  and support modules, while retaining project features such as guidance and
  custom parameters.
- **`simulink.px4board`** retains the stock control system and adds the project
  Simulink features. Use this when you want stock PX4 control alongside custom
  functionality.
- **`default.px4board`** is the entry point required by the SITL setup workflow.
  The project also copied the hardware `simulinkctrl` profiles into hardware
  defaults. Do not assume these defaults are stock reference configurations.
  Select the explicit `simulinkctrl.px4board` file for hardware setup so that
  the intended profile is clear.

### When to rerun setup

Rerun the MATLAB setup app whenever you:

- Switch to a different board or CMake/build target.
- Add or remove uORB messages used by the Simulink integration.

Once setup is complete, open the Simulink model and use Connected I/O, run on
hardware, or compile normally as supported by the selected target. No additional
project-specific build steps are required for the usual model workflow.

## 3. Maintain custom parameters

`src/modules/simulink_parameters` serves two purposes:

- Provide a dedicated place for custom Simulink parameters in
  `src/modules/simulink_parameters/parameters.yaml`.
- Supply the necessary stock parameter definitions when the stock control
  system is excluded, so the remaining firmware can compile without those
  controller modules. The `stock_*.yaml` files are included conditionally when
  their owning controllers are disabled.

The `simulinkctrl` profiles exclude the stock controller code from compilation.
This prevents the excluded controllers from conflicting with Simulink control
and saves firmware space and memory compared with compiling the controllers but
simply not starting them through the MATLAB setup option. The shared startup
CMake configuration also packages the vehicle defaults and app scripts needed
by these profiles when `CONFIG_MODULES_SIMULINK_PARAMETERS=y`.

### Rebuild after changing the parameter YAML

**Every time a new parameter is added to
`src/modules/simulink_parameters/parameters.yaml`, rebuild the firmware
to regenerate the parameter metadata and firmware image.** YAML edits alone do
not update an existing firmware binary. Rebuild after changing definitions,
defaults, or other parameter settings as well.

The most convenient method is to invoke `make` directly in Ubuntu instead of
rerunning the MATLAB setup app solely for a parameter change:

```bash
cd ~/PX4/v1.16.1-mod
make px4_sitl_default
```

For hardware, build the configuration selected during setup:

```bash
make cubepilot_cubeorange_simulinkctrl
make cubepilot_cubeorangeplus_simulinkctrl
make px4_fmu-v6c_simulinkctrl
```

Run only the command for your target. If your workflow explicitly uses the
copied hardware `default` configuration, use its corresponding target instead,
for example `make cubepilot_cubeorangeplus_default`.

Use the rebuilt SITL binary or upload the rebuilt hardware firmware before
trying to access the new parameter. Rebuilding generates the metadata; it does
not automatically flash hardware. Changing a YAML default also does not replace
an already saved parameter value on the vehicle.

## 4. Modified firmware: topics, telemetry, and logging

For the required controller publications, PX4 arming/land detection, VTOL state,
and S2F-X200 SITL/Gazebo launch instructions, see
`src/modules/simulink_parameters/CONTROLLER_INTEGRATION.md`.

### S2F-X200 hardware airframe

Select **S2F-X200** in the **Standard VTOL** airframe group, or set
`SYS_AUTOSTART=22000` and reboot. Its startup file is
`ROMFS/px4fmu_common/init.d/airframes/22000_s2f_x200_simulink`.
It uses the standard VTOL vehicle defaults and exposes five motor channels and
seven servo channels for the Simulink model:

| Logical channel | Model function |
| --- | --- |
| Motors 1–4 | Lift propellers LP-1 through LP-4 |
| Motor 5 | Cruise propeller MP-1 |
| Servo 1 | Left outboard aileron |
| Servo 2 | Left flap |
| Servo 3 | Right flap |
| Servo 4 | Right outboard aileron |
| Servo 5 | Left elevator |
| Servo 6 | Right elevator |
| Servo 7 | Rudder |

Physical output routing is deliberately not preset. Assign motor and servo
functions to the actual PWM/CAN outputs in QGC. The airframe does not overwrite
existing saved routing. The counts and surface labels describe the logical
outputs, not a tuned PX4 allocation matrix: the Simulink model publishes
`actuator_motors` and `actuator_servos` directly. The S2F-X200 startup path runs
airspeed selection and VTOL land detection without starting stock controllers
or the PX4 control allocator. MATLAB setup remains responsible for installing
and starting its generated application.

Hardware `simulinkctrl` builds package only S2F-X200 and generate a matching
single-airframe `airframes.xml`. Hardware `default` builds using the copied
minimal profile get the same filter. Stock-control `simulink` builds retain the
normal airframe selection. SITL retains its existing simulation airframes and
adds the external-simulator S2F-X200 (`22000`), plus the experimental Gazebo
S2F-X200 (`22001`) from the Gazebo fork. A GCS
that uses its own bundled or cached airframe catalog may still show stock
entries; firmware filtering does not rewrite that GCS catalog. The airframe ID
can be selected directly even if the GCS has not loaded the custom metadata.

### X200 disarming defaults

The shared X200 airframe sets these defaults for hardware, external-simulator
SITL, and the Gazebo X200 variant:

| Parameter | Default | Effect |
| --- | --- | --- |
| `COM_DISARM_LAND` | -1 | Disable landing-triggered automatic disarming. |
| `COM_DISARM_PRFLT` | -1 | Disable preflight/no-takeoff automatic disarming. |
| `COM_KILL_DISARM` | 0 | Disarm without a configured delay after kill/lockdown. |
| `COM_LKDOWN_TKO` | 1 s | Shorten the early critical-failure disarm window; does not disable it. |

These do not make every disarm pilot-only or guarantee acceptance of every
ordinary disarm request. Commander still has automatic lockdown and failsafe
paths. The early critical-failure window is measured from arming and includes
`COM_SPOOLUP_TIME` in addition to `COM_LKDOWN_TKO`; spoolup battery/ESC failures
also have disarm paths. Some configured failsafes can terminate outputs.

`COM_DISARM_MAN=1` (the stock default) permits RC switch/stick/button disarming
while airborne in multicopter manual-thrust modes such as Stabilized. It does
not grant the same exception in FW flight or for ordinary MAVLink disarm
commands. Kill uses a forced path; an explicitly requested MAVLink force-disarm
uses `MAV_CMD_COMPONENT_ARM_DISARM` with `param1=0`, `param2=21196`.

Airframe defaults do not overwrite saved user values. On an already configured
vehicle, set the four parameters explicitly or reset those parameters to their
airframe defaults after rebuilding/restarting.

### Separate debug-array topics for Simulink

`msg/DebugArray.msg` defines one shared message structure: a timestamp in
microseconds, an ID, a 10-character name field, and 58 `float32` data values.
Several **separate named uORB topics** use this structure. They are independent
channels: publishing `simulink_outbound_1` does not update `simulink_outbound`,
`simulink_inbound`, or any other topic. These are named topics using the same
structure, rather than interchangeable uORB instance numbers of one topic.

In Simulink, publish or subscribe to each topic by its own name. Use the same
structure but assign your own meaning to the array elements consistently at
both ends. The names describe intended use; they do not restrict a topic to
publishing or subscribing. No stock controller owns these project-specific
channels, so stock controller traffic does not populate them.

The guidance channels have project-specific users: `src/modules/simulink_guidance`
can read `simulink_inbound` when `SMG_IN_TYPE=1`, publishes `simulink_guidance`
when the debug-array output is selected with `SMG_OUT_TYPE` bit 2, and uses the
companion guidance topics for companion communication. Reserve those payloads
when using guidance; they are not unused scratch channels in that configuration.
The MAVLink receiver, streamers, and logger also intentionally access these topics.

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

The modified MAVLink integration automatically configures these streams in
**Simulink mode** (`MAV_n_MODE=15`) and **Config mode** (`MAV_n_MODE=5`, typically
USB), subject to `SM_MAV` and the rate parameters. Replace `n` with the MAVLink
instance number. Other modes do not automatically enable this entire set.
**Companion mode** (`MAV_n_MODE=16`) separately configures only
`companion_guidance_inbound` at `SMG_MAVIN_R`, plus basic link/status streams;
that branch does not use the `SM_MAV` mask.

Each custom stream is sent as the same MAVLink message type,
`DEBUG_FLOAT_ARRAY` (message ID 350), distinguished by its `array_id` and name
above. A generic ground station can therefore display many debug arrays and
become cluttered. Custom ground-station code should dispatch by `array_id`
and display the channels separately. The streamer assigns each topic's fixed
ID and name; changing its uORB `id` field does not change this mapping.
Incoming MAVLink `DEBUG_FLOAT_ARRAY` messages are likewise routed to the named
uORB topic by `array_id`; unrecognized IDs go to the ordinary `debug_array` topic.

`SM_MAV` is a bitmask, not a rate. Its default is **255** (bits 0–7), which
includes every listed custom stream **except `simulink_inbound_ctrl`**. Use
**511** to include bit 8 as well, or sum only the desired bit values (`1 << bit`).
A positive rate configures the selected stream; zero skips its automatic
configuration. All custom rates default to **20 Hz**. `SM_MAVOUT_0_R` through
`SM_MAVOUT_3_R` allow **0–100 Hz**; the other rates in the table allow
**0–250 Hz**.

These controls limit outgoing telemetry, not the Simulink publisher rate or
incoming MAVLink traffic. A stream sends only when the topic has updated, and
link bandwidth can reduce the actual rate. The MAVLink instance's `MAV_n_RATE`
sets its overall sending budget in bytes/second. Restart the affected MAVLink
instance or reboot after changing automatic stream settings; the custom rate
setup is read when default streams are configured, even though `SM_MAVCTRL_R`
is not marked reboot-required in its parameter metadata. An explicit GCS stream
request can subsequently change a stream's rate.

### Automatic ULog topic logging

The modified logger registers the project topics when **`SDLOG_PROFILE` bit 12**
is enabled. This bit has value **4096**: add it to the desired existing profile
without clearing other bits. For example, **4097** enables the standard profile
plus Simulink logging; **4227** adds Simulink logging to profile 131 used as the
SITL startup default. The current SITL default of 131 alone does **not** enable
bit 12. `SM_LOG` alone cannot enable this profile.

Within that profile, `SM_LOG` selects topic groups. Its default is **511**,
which enables all nine groups below. Logging remains conditional on the logger
running, a logging session being active, and the topic actually being published.
A nonempty `etc/logging/logger_topics.txt` custom topic list replaces the normal
profile selection, so include the desired topics there explicitly if using one.

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

All listed logging intervals accept **0–10000 ms** and require a reboot.
A positive interval limits logging frequency: 20 ms is up to 50 Hz, 100 ms is
up to 10 Hz, and 1000 ms is up to 1 Hz. **Zero means unrestricted by this
interval, not disabled**; clear the corresponding `SM_LOG` bit to exclude a
group. Actual logging cannot exceed the topic's publication rate, and logger
scheduling/storage limits still apply. Several topics share one interval, as
shown above. Streaming and logging are independent: changing an `SM_MAV*` rate
does not change the ULog rate, and changing a logging interval does not change
telemetry or the underlying Simulink sample time.

`SDLOG_MODE` determines when a logging session runs:

| Value | Logging session |
| --- | --- |
| -1 | Disabled |
| 0 | Armed until disarm (parameter default) |
| 1 | Boot until disarm (SITL startup default) |
| 2 | Boot until shutdown |
| 3 | While manual input AUX1 is above 30% |
| 4 | First arming until shutdown |

`SDLOG_BOOT_BAT=1` additionally prevents boot-start logging without detected
battery power; its default is 0. Reboot after changing logging mode/profile
settings. Selecting topics does not by itself start a recording session.

### Standard telemetry rate controls in Simulink mode

Simulink MAVLink mode also provides independent rate overrides for selected
standard PX4 messages. `SM_MAVSTD` selects these overrides; its default is
**2015**, enabling all defined bits below (bit 5 is unused). Clearing a bit or
setting its rate to zero skips the override; it does not necessarily disable a
stream already configured by another mechanism. These settings are separate
from both `SM_MAV` debug-array streaming and `SM_LOG` logging.

| `SM_MAVSTD` bit | MAVLink stream | Rate parameter | Default | Range |
| --- | --- | --- | --- | --- |
| 0 | `ATTITUDE` | `SM_MAV_ATT_R` | 15 Hz | 0–250 Hz |
| 1 | `ATTITUDE_QUATERNION` | `SM_MAV_ATTQ_R` | 10 Hz | 0–250 Hz |
| 2 | `ATTITUDE_TARGET` | `SM_MAV_ATTT_R` | 2 Hz | 0–250 Hz |
| 3 | `LOCAL_POSITION_NED` | `SM_MAV_LP_R` | 10 Hz | 0–250 Hz |
| 4 | `POSITION_TARGET_LOCAL_NED` | `SM_MAV_LPT_R` | 10 Hz | 0–250 Hz |
| 6 | `GLOBAL_POSITION_INT` | `SM_MAV_GPI_R` | 5 Hz | 0–50 Hz |
| 7 | `POSITION_TARGET_GLOBAL_INT` | `SM_MAV_GPT_R` | 1 Hz | 0–50 Hz |
| 8 | `SERVO_OUTPUT_RAW_0` | `SM_MAV_SRV_R` | 10 Hz | 0–250 Hz |
| 9 | `ESC_STATUS` | `SM_MAV_ESC_R` | 1 Hz | 0–50 Hz |
| 10 | `RAW_RPM` | `SM_MAV_RPM_R` | 2 Hz | 0–100 Hz |

Reboot or restart the MAVLink instance after changing these automatic defaults.
The custom telemetry parameters are defined in `src/modules/mavlink/module.yaml`;
the logging mask and intervals are defined in `src/modules/logger/module.yaml`.
The shared debug-array schema and routing IDs are in `msg/DebugArray.msg`.

## Troubleshooting

### R2026b: startup errors or Run on Board does not work properly

Before a clean support-package setup, make sure the old
`src/modules/px4_simulink_utils` directory is deleted, not merely emptied.
Preserve any custom files in that directory before deleting it, then rerun the
MATLAB setup app so it can install the required module files.

Git does not track empty directories, so discarding files or reverting changes
in VS Code can leave this directory behind. The R2026b setup checks whether the
directory exists and skips copying the module files if it does, even when the
directory is empty. This can cause startup errors such as
`px4_simulink_utils: not found`, connection failures, or problems with **Run on
Board**. After setup, verify that the directory contains `CMakeLists.txt`,
`Kconfig`, `px4_simulink_utils.cpp`, `px4_simulink_utils.h`, and
`px4_simulink_utils_params.c`.

### Missing controller tuning parameters in a minimal VTOL build

`simulinkctrl` intentionally excludes stock controllers and the stock hover-thrust
estimator. Their private tuning parameters (for example `MC_ROLL_P` and
`HTE_VXY_THR`) can therefore be absent. `rc.vtol_defaults` applies those optional
defaults with `param set-default -s`, preserving stock-build values while quietly
skipping absent parameters. Required shared parameters remain supplied by
`src/modules/simulink_parameters`; there is no need to re-enable the excluded
controllers to obtain their private tuning settings.

In SITL, X200 airframes default `SMG_TRAJ_DIR=0` (local `./Trajectories/`). This
does not create trajectory files. If the guidance module is used, provide the
selected trajectory there. Existing saved values can override the new default;
use `param set SMG_TRAJ_DIR 0` to switch an existing session from the SD-card path.

### Missing custom uORB header during Simulink setup

An error such as `uORB/topics/simulink_custom_message.h: No such file or
directory` indicates that the generated app and available uORB message
definitions are inconsistent. Check that the required message definitions are
present and registered, then rerun setup. If generated integration changes are
stale, restoring the clean modified project firmware and repeating setup has
resolved this issue in this project. Preserve your own changes before restoring
files.

### Missing vehicle startup defaults in a minimal build

If startup cannot find `rc.mc_defaults` or similar vehicle scripts, verify that
`CONFIG_MODULES_SIMULINK_PARAMETERS=y` is enabled and the shared startup CMake
changes are present. These scripts must be packaged even when the corresponding
stock controllers are excluded. Rebuild after correcting the configuration.
