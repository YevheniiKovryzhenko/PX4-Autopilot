# S2F-X200 PX4–Simulink interface

Simulink reads PX4's selected inputs and accepted state, then publishes actuator demands and controller telemetry through uORB.

| Component | Responsibility |
| --- | --- |
| Simulink application | Command generation, flight control, mixing and actuator demand |
| PX4 | Input calibration/source selection, accepted mode, arming, estimation, output drivers, logging and MAVLink transport |
| Model / airframe | `S2F_X200_PX4_26b`, X200 (`SYS_AUTOSTART=22000`) |
| GCS output | `simulink_outbound` |
| Stock controllers / allocator | Not started on this airframe |

Installation and tuning are covered in [README.md](README.md). Keep these documents together so links work in either the firmware or the project's `PX4-Firmware Setup` folder. Source-file paths are relative to the installed firmware root.

## Pilot input and control level

RC receivers and MAVLink joysticks converge at `manual_control_setpoint`. PX4 selects one complete source according to `COM_RC_IN_MODE`; it does not mix axes from several sources. RC calibration, reversal and channel-function mapping happen before this topic. The Simulink application therefore uses the same normalized fields regardless of which source is selected.

The four primary fields enter the controller's configurable stick-mapping function as follows. Every axis, including throttle, spans **[-1, +1]**.

| PX4 field | Mapping-function input | Meaning before configurable remapping |
| --- | --- | --- |
| `roll` | `dlat_in = roll` | Lateral stick |
| `pitch` | `dver_in = -pitch` | Longitudinal stick, inverted at this interface |
| `yaw` | `ddir_in = yaw` | Directional stick |
| `throttle` | `dacc_in = throttle` | Throttle stick, full range without offset |

The mapping function selects/swaps/inverts the sticks for the configured layout and outputs `[dLAT; dVER; dDIR; dACC]`. These are controller axes, not fixed physical sticks. The controller applies its own deadzones, filtering, speed-dependent response and holds.

| Control level | dVER path | dACC path |
| --- | --- | --- |
| 1 and 2 | Direct longitudinal/pitch demand | Propulsor demand |
| 3 and 4 | Vertical/path command | Acceleration/speed command |

### Auxiliary inputs and pilot-command vector

| Input / vector entries | Purpose |
| --- | --- |
| Entries [1, 4] | dLAT, dVER, dDIR, dACC from the stick-mapping function |
| `aux1` → entries [5, 14] | Selected automatic-flight event: TKO, DXN, NAV, AXN, LND, then M1–M5 |
| `aux1` → auxiliary paths | Can alternatively select source handoff or model RTH; RTH does not select PX4 RTL |
| `aux2` | Rotary-dial command, mapped from [-1, +1] to [0, +1] |
| Entries [15, 16] | Configured nacelle automatic/manual selection and manual angle; not joystick axes |
| Entry 17 | `SIGNAL_LOSS = ~manual_control_setpoint.valid` |
| Entry 18 | Model handoff signal, not PX4's `data_source` enumeration |
| Entries [19, 20] | Reserved zeros |
| AUX3–AUX6 / `flaps` | Available in PX4 but unused by this input map |

AUX1 uses `(aux1 + 1)/2`. A two-position switch therefore supplies 0 or 1; a rising 0→1 requests an event. Release before requesting another event. TKO/DXN/NAV/AXN/LND mean takeoff, departure transition, navigation, arrival transition and landing. M1–M5 are reserved and have no maneuver-engagement implementation.

**Use `SM_PI_MODE=1` in Pilot Inceptor Blending.** The input labelled Transmitter already contains PX4's selected RC or joystick data. The separate GCS input is inactive and can replace commands with zeros if selected. PX4 handles source selection and its input timeout; a cycle without a new uORB sample is not itself signal loss.

### Accepted flight mode

The control-level function reads **`vehicle_status.nav_state`**, the accepted active mode, and retains its last mapped level for all other values:

| Accepted PX4 mode | `nav_state` | Controller level |
| --- | --- | --- |
| Manual | 0 | 1: direct/calibration path |
| Stabilized | 15 | 2: attitude-command path |
| Altitude | 1 | 3: full command-generation/TCS path |
| Position | 2 | 4: full command-generation/TCS path |

| Situation | Controller behavior |
| --- | --- |
| Application starts | Initialize persistent control level to 1 |
| A mapped mode is accepted | Update the control level |
| Disarm or unmapped mode | Retain the last mapped level |
| Application restarts | Initialize again |
| Levels 3 and 4 | Use the same main control-law branches; configured holds determine behavior |
| RTL, Land, Offboard or other unmapped mode | Does not implicitly activate a corresponding controller function |

Read `nav_state`, not `nav_state_user_intention`. PX4 may reject/change a requested mode when required input or estimates are unavailable. The four mapped modes do not require an Offboard heartbeat.

## Arming, kill and actuator authority

The application reads `actuator_armed`, rather than deriving permission from stick positions or requests. Its current logic is:

```matlab
Arm = armed && ~in_esc_calibration_mode;
Kill = manual_lockdown;
ActuatorsArmed = Arm && ~Kill;
```

| Signal / state | Effect |
| --- | --- |
| `Arm` | Enables the flight-control subsystem |
| `ActuatorsArmed` | Gates motor/servo commands and reported thrust demand |
| Kill while still armed | Computation can continue; inhibited controls are NaN |
| Subsequent Commander disarm | Removes `Arm`; X200 has no configured kill-disarm delay |
| Unused actuator channels | NaN; motors use `reversible_flags=0` |
| Driver lockdown / failsafe | PX4 enforces final disarmed/failsafe outputs independently |

`lockdown` and `force_failsafe` are not separate inputs to this model's Arm expression. Flight termination differs from `Kill`. Driver lockdown affects servos as well as motors, so running the controller does not guarantee physically active surfaces.

## GCS messages and RC input

A GCS should send a regular MAVLink heartbeat and use MAVLink 2. Continuous manual input is separate from discrete requests such as arm or change mode. Send manual samples comfortably faster than the configured loss timeout (for example 20 Hz); a heartbeat alone does not keep manual input valid.

### Normalized joystick input

Send `MANUAL_CONTROL` (message 69), addressed to the vehicle system ID. This firmware accepts the following wire values and performs the conversions shown:

| MAVLink field | Wire range | Selected PX4 field |
| --- | --- | --- |
| `x` | [-1000, +1000] | `pitch = x/1000` |
| `y` | [-1000, +1000] | `roll = y/1000` |
| `z` | **[0, 1000]** | `throttle = z/500 - 1` |
| `r` | [-1000, +1000] | `yaw = r/1000` |
| `aux1`, `aux2` | [-1000, +1000] | `aux1/1000`, `aux2/1000` |

| GCS requirement | What to send / configure |
| --- | --- |
| Primary axes | Valid x/y/z/r on every sample; this receiver accepts z only in **[0, 1000]**, even though the common definition permits signed z |
| AUX1 / AUX2 | Set `enabled_extensions` bits 2 and 3; mask **12** enables both |
| AUX3–AUX6 | Extension bits [4, 7]; unused by the model |
| AUX1 inactive / active | Send **-1000 / +1000**, giving model values 0 / 1; wire zero gives 0.5 |
| AUX continuity | Send valid extensions continuously; omission does not retain the previous switch value |
| Input source | `COM_RC_IN_MODE=1` for joystick, or 2 for RC/joystick fallback |
| Buttons | Use explicit PX4 commands or encode model functions on AUX; this model does not consume the `buttons` bitmask |

A GCS must actually transmit the AUX extensions. A button shown in its UI is not automatically an AUX mapping or an arm/kill/mode request. Source selection chooses one stream; it does not blend trainer inputs.

### Raw RC channels

A physical receiver uses PX4's normal RC path. A GCS can supply equivalent channel frames with `RC_CHANNELS_OVERRIDE` (message 70).

| Item | Contract |
| --- | --- |
| PX4 route | `input_rc` → RC calibration, function mapping and switch processing |
| Source selection | Treated as RC; requires an RC-permitted input setting |
| Channel values | Complete calibrated PWM-equivalent frame, typically **[1000, 2000] µs** |
| Functions to include | Primary axes, AUX1/AUX2 and any assigned flight-mode, arm and kill switches |
| Partial overrides | No persistent per-channel merge with a physical receiver; do not combine competing publishers using partial/release semantics |
| `RC_CHANNELS` / `RC_CHANNELS_RAW` | Telemetry, not control-input messages for this route |

### Requests for mode, arm and emergency stop

Use `COMMAND_LONG` with the correct vehicle/component target and inspect `COMMAND_ACK`; then observe the accepted state. `MAV_CMD_DO_SET_MODE` (176) uses `param1=1` (custom mode enabled), `param2` equal to the PX4 main mode, and `param3=0` for these selections:

| Requested mode | PX4 main-mode value (`param2`) | Resulting controller level |
| --- | --- | --- |
| Manual | 1 | 1 |
| Stabilized | 7 | 2 |
| Altitude | 2 | 3 |
| Position | 3 | 4 |

These main-mode values are not `nav_state` values. A GCS can also use `SET_MODE` with PX4's encoded custom mode; Commander converges both paths into `vehicle_status`. An RC flight-mode switch reaches that same accepted state.

| Action | Request / route | Accepted result |
| --- | --- | --- |
| Arm | `MAV_CMD_COMPONENT_ARM_DISARM` (400), `param1=1`, `param2=0` | `actuator_armed.armed`, subject to PX4 checks |
| Normal disarm | Command 400, `param1=0`, `param2=0` | Armed state changes if Commander accepts |
| Emergency force-disarm | Command 400, `param1=0`, `param2=21196` | Forces disarm; does not set `manual_lockdown` |
| Kill / unkill | Configured RC kill switch, including selected raw RC override | Internal KILL/UNKILL request → `actuator_armed.manual_lockdown` |
| Optional kill gesture | Configured kill-stick gesture on selected manual input | Internal KILL request |
| Flight termination | `MAV_CMD_DO_FLIGHTTERMINATION` (185), `param1=1`, while armed | Actual PX4 termination; different from the model's Kill flag |

Normalized joystick messages have no direct kill-switch field. AUX1/AUX2 and the button bitmask do not themselves request arm, kill or flight-mode changes. To obtain the identical RC kill state from a GCS, use the selected RC switch path or a deliberately defined command bridge. Internal `action_request` is uORB, not a standard MAVLink message. Termination is not reversible kill/unkill; command 185 with `param1=0` does not unkill this firmware.

## State feedback and control outputs

The Sensor Outputs subsystem selects/blends PX4 sensor feeds into the controller's attitude, rates, velocity, altitude, position, track and ground-contact bus.

| PX4 input | Controller information |
| --- | --- |
| `vehicle_attitude.q` | Attitude quaternion |
| `vehicle_angular_velocity.xyz` | Body rates |
| `vehicle_acceleration.xyz` | Acceleration |
| `vehicle_local_position` | Horizontal/vertical velocity, local height and position accuracy |
| `vehicle_global_position.lat/lon` | Global position |
| `distance_sensor` | Distance/height feed |
| `airspeed.true_airspeed_m_s` | Airspeed |
| GPS support-package block | Additional GPS feed |

Ground contact is constructed from the model's sensor path, rather than read from `vehicle_land_detected`.

The application publishes these control-side messages with PX4 boot-relative microsecond timestamps. It does not publish competing arming or vehicle-mode state messages.

| Publication | Model contract |
| --- | --- |
| `actuator_motors` | Control indices [0, 3]: lift motors LP1–LP4; control 4: cruise motor MP1. Non-reversible demand [0, +1], computed as `PWM/1000 - 1` from the controller's PWM-equivalent demand. Unused/inhibited controls are NaN. |
| `actuator_servos` | Control indices [0, 6]: left outboard aileron, left flap, right flap, right outboard aileron, left elevator, right elevator, rudder. Demand [-1, +1], computed as `2*(PWM/1000 - 1.5)`. Unused/inhibited controls are NaN. |
| `vehicle_thrust_setpoint` | Land-detector feedback. Body-Z is negative for lift; X/Y are zero in the current publisher. Lift is approximated from collective lift RPM, the static propeller polynomial, four motors and their 15° cant, normalized against weight and hover thrust. Disabled outputs report zero thrust. This is normalized demand, not PWM or Newtons. |
| `vtol_vehicle_status` | Reports MC (3), transition toward FW (1), transition toward MC (2), or FW (4), from TCS mode and departure/arrival status. The function retains state when its automatic-regime inputs are not applicable. |

The motor/servo channel order is logical: hardware PWM/CAN routing is assigned separately to the corresponding output functions. In SITL, the simulator's output-function mapping determines the destination. `offboard_control_mode` and `trajectory_setpoint` are not pilot-input requirements for this interface; implementing Offboard or other navigation modes would require their own reference and supervision contract.

## GCS output, MAVLink and flight logs

| Interface item | Contract |
| --- | --- |
| Producer | Model Post-Processing block |
| uORB output / ULog topic | **`simulink_outbound`**, using `DebugArray` |
| Payload | 58 `float32` values |
| MAVLink message | **`DEBUG_FLOAT_ARRAY` (350)**; no private dialect needed |
| GCS channel identity | **`array_id=100`, `name="outbound"`** |
| Simulink fields to write | `timestamp` and `data` |
| ID/name assignment | PX4 supplies fixed identities when transmitting; original uORB/logged fields are unchanged |

The GCS dispatches by `array_id` and decodes the payload below. Identify logged data by its ULog topic name.

Three additional outbound topics are available for expansion and are currently unused by the X200 model:

| uORB topic | MAVLink `array_id` | MAVLink name |
| --- | --- | --- |
| `simulink_outbound` | 100 | `outbound` |
| `simulink_outbound_1` | 101 | `outbound1` |
| `simulink_outbound_2` | 102 | `outbound2` |
| `simulink_outbound_3` | 103 | `outbound3` |

Each carries its own 58-element array; publishing one does not update the others. Only published samples generate telemetry/log records. These are named topics sharing one structure, not alternative instance numbers. The generic `SimulinkCustomMessage` support-package message is not the GCS telemetry format.

The GCS output layout below uses **zero-based** array indices (add one for MATLAB indexing). Bracketed index intervals include every integer index between their endpoints. Commands and states retain the units indicated by the model bus; angle commands are degrees, measured attitude/track/heading are radians, controller speed is knots, and linear velocities are metres per second.

| `data` indices | Contents, in order |
| --- | --- |
| [0, 2] | EMF `ulat`, `ulon`, `udir` |
| [3, 4] | Collective lift RPM, cruise RPM |
| [5, 10] | Bank command, pitch command, yaw-rate command, vertical-speed command, acceleration command (g), speed command (knots) |
| [11, 16] | TWH, TWV, wing/nacelle command, cruise T/W command, lift T/W command, encoded TCS status |
| [17, 25] | Heading memory, track memory, longitudinal/lateral position errors, altitude memory, lateral/vertical/speed blend factors, encoded hold status |
| 26 | Encoded autoflight status |
| [27, 30] | LNAV cue, VNAV cue, speed cue, navigation status |
| [31, 36] | Navigation latitude/longitude/speed signals, waypoint index, distance to target, target track |
| [37, 49] | North/east velocity, altitude, equivalent airspeed, forward/lateral speed, latitude/longitude, vertical speed, ground contact, controller speed, track, heading |
| 50 | Armed-time accumulator (seconds) |
| [51, 53] | Control level, signal-loss flag, model handoff/source signal |
| [54, 57] | Filtered dLAT, dVER, dDIR, dACC |

Status fields use decimal weights, not bitmasks. Add each listed state multiplied by its decimal weight to the base value.

| Encoding | Base | States, from highest to lowest decimal weight |
| --- | --- | --- |
| TCS | 111111 | TCS mode, takeoff mode, high throttle, low throttle, path priority, speed priority; weights 10^5 through 10^0 |
| Holds | 111 | Direction, position, altitude; weights 10^2 through 10^0 |
| Autoflight | 1111111 | TKO, DXN, NAV, AXN, LDG, FAP, maneuver; weights 10^6 through 10^0 |

Preserve fractional states when decoding transition/abort values.

### Automatic X200 transport setup

| X200 default | Purpose |
| --- | --- |
| `SDLOG_PROFILE=4097` | Standard logging plus Simulink topics |
| `SM_LOG=511` | All custom logging groups |
| Inbound/outbound logging interval: 20 ms | Limits recording to up to 50 Hz |
| `MAV_PROTO_VER=2` | Supports debug-array telemetry |
| `SM_MAV=511` | All custom streams selected on every link/profile |
| Inbound/outbound stream rates: 20 Hz | Target telemetry rate |
| `MAV_0_MODE=15` | Lean Simulink standard-telemetry preset on the primary hardware link |

SITL selects the Simulink preset through startup. Other profiles retain their own standard messages and use the same global `SM_MAV` selection. Set `SM_MAV=0` to disable automatic custom streams; selected streams also need positive rates. Restart links/reboot after changes. Explicit GCS stream requests can override rates.

| Check | What matters |
| --- | --- |
| Applying defaults | Rebuild/restart; saved values take precedence |
| Gazebo variant | Source shared X200 airframe defaults to inherit logging/rate settings |
| Actual rates | Limited by model publication and link/storage capacity |
| Recording | Logger and recording session must be active; topic selection does not enable continuous disarmed recording |
| Missing output | Check masks/rates, MAVLink version, logger state and any custom `logger_topics.txt` replacing the profile |
| Runtime inspection | `listener simulink_outbound`, `mavlink status`, `logger status`; confirm ULog topic and received array ID 100 |

Do not transmit GCS debug arrays with ID 100 back to PX4: the receiver can publish them onto the same output topic. The Simulink preset's standard messages are selected separately through `SM_MAVSTD`; see [setup and streaming controls](README.md).

## Guidance topic ownership

| Platform | Trajectory location selected by the X200 preset |
| --- | --- |
| Hardware | `SMG_TRAJ_DIR=1`: `/fs/microsd/Trajectories/` on the SD card |
| SITL | `SMG_TRAJ_DIR=0`: `./Trajectories/` relative to the PX4 working directory, normally `rootfs` |

Provide the trajectory files in that directory. Presets set defaults; saved values take precedence when switching platforms. See [trajectory setup](README.md#guidance-trajectory-location).

Reserve the following interfaces for `simulink_guidance`; do not use them as extra pilot commands or GCS telemetry channels. Starting the module does not itself arm the aircraft, select a flight mode, or make the X200 controller follow its references. A consuming controller must implement that connection.

| Topic | Guidance role and payload |
| --- | --- |
| `sim_guidance_request` | Input: prepare/start, reset, capture home, start execution, or stop. Results are reported asynchronously. |
| `vehicle_local_position` | Shared PX4 state input when `SMG_IN_TYPE=0`; requires valid, recent local position and yaw. |
| `simulink_inbound` (array ID 200, `inbound`) | Alternative state input when `SMG_IN_TYPE=1`: zero-based `data[18:20]` NED velocity, `[24:27]` quaternion w/x/y/z, `[32:34]` NED acceleration, `[35:37]` NED position. These ranges are inclusive, not Python slices. The X200 GCS output layout is incompatible with this contract. |
| `companion_guidance_outbound` (120, `compg_out`) | Companion-to-PX4 input when `SMG_EN=2`: readiness, finished flag, trajectory time, then absolute NED position and derivatives through snap. |
| `companion_guidance_inbound` (121, `compg_in`) | PX4-to-companion output: execution requests, current reference and derivatives, and measured position. |
| `sim_guidance_status` | Output: prepared/loaded/executing/finished/valid flags. |
| `trajectory_setpoint`, `sim_guidance_trajectory` | Selectable reference outputs: NED position/velocity/acceleration and yaw; the custom trajectory adds jerk/snap and trajectory time. Do not introduce another publisher on a reference topic being used by guidance. |
| `simulink_guidance` (150, `guidance`) | Selectable debug-array output: finished flag followed by four-axis position, velocity, acceleration, jerk and snap. |

| Integration rule | Meaning |
| --- | --- |
| Companion direction | Companion **outbound** is an input to PX4; companion **inbound** is an output from PX4 |
| `simulink_inbound_ctrl` (250, `in_ctrl`) | Available but not consumed by guidance; publishing it does not issue a guidance request |
| Incoming debug-array packets | GCS/companion must supply the correct `array_id` for routing |
| Outgoing debug-array packets | PX4 supplies the fixed ID/name for each named topic |
| Detailed guidance contract | `src/modules/simulink_guidance/README.md` in the installed firmware |

## Source references

| Source | Interface definition |
| --- | --- |
| Saved `S2F_X200_PX4_26b` model | Signal routing and controller payloads |
| `msg/versioned/ManualControlSetpoint.msg`, `VehicleStatus.msg` | Selected input and accepted mode fields |
| `msg/ActuatorArmed.msg`, `msg/DebugArray.msg` | Actuator permission and custom telemetry structures |
| `manual_control`, `mavlink_receiver.cpp`, Commander | Source selection, wire conversions and accepted requests |
| `streams/DEBUG_FLOAT_ARRAY.hpp` | Outgoing channel identity and payload transport |
| `ROMFS/px4fmu_common/init.d/airframes/22000_s2f_x200_simulink` | Shared X200 defaults |

Wire references: [MAVLink manual control](https://mavlink.io/en/services/manual_control.html) and [common messages](https://mavlink.io/en/messages/common.html). The local receiver's supported ranges and behavior define this firmware's interface.
