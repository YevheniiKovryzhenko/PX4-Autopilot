# S2F-X200 controller integration with PX4 v1.16.1-mod

This describes the message contract for the Simulink-controlled S2F-X200
(`SYS_AUTOSTART=22000` on hardware or external-simulator SITL, `22001` in Gazebo).
Stock controllers, the allocator, and the hover-thrust estimator are excluded
from `simulinkctrl`. Commander, EKF2, output drivers, and `land_detector start
vtol` remain responsible for their normal functions.

## What the model publishes

Use PX4's boot-relative microsecond clock for `timestamp` and the corresponding
measurement/sample time for `timestamp_sample` where present. Publish current
state, not a constant value chosen to suppress a failsafe.

| Topic | Responsibility and suggested update rate |
| --- | --- |
| `actuator_motors` | Required at the controller output rate. Channels 0–4 are lift motors 1–4 and cruise motor 5. Normal non-reversible commands are 0–1. Set unused channels to NaN, and `reversible_flags=0` unless reverse thrust is actually configured. |
| `actuator_servos` | Required at the controller output rate. Channels 0–6 follow the documented seven-servo order. Use normalized -1–1, with 0 at center; NaN denotes a disarmed/unused channel. |
| `vehicle_thrust_setpoint` | Required for correct MC/VTOL low-thrust/land detection. Publish continuously, for example at 50 Hz or the control rate. `xyz` is normalized body-FRD thrust, **not Newtons, PWM, or mean ESC command**. In lift-supported hover `xyz[2]` is negative; publish the main topic, not just `vehicle_thrust_setpoint_virtual_mc`. |
| `vtol_vehicle_status` | Required to report actual VTOL mode now that `vtol_att_control` is absent. Publish periodically (for example 20 Hz) and on changes. States are 1 transition-to-FW, 2 transition-to-MC, 3 MC, 4 FW. Report `fixed_wing_system_failure` truthfully. Start in MC. |
| `offboard_control_mode` | Required **if using PX4 Offboard mode**, not merely because the controller is built from Simulink. For direct motor/servo output, use `direct_actuator=true` and the other control-level flags false. Publish continuously (for example 10 Hz), including before requesting Offboard. PX4 checks freshness against `COM_OF_LOSS_T`. |
| `takeoff_status` | Publish if implementing the takeoff state machine that replaces `mc_pos_control`. States are 0 uninitialized, 1 disarmed, 2 spoolup, 3 ready, 4 rampup, 5 flight; include `tilt_limit` in radians. The land detector uses this for ground-effect reporting. It is **not** the signal that directly cancels Commander's preflight auto-disarm timer. |
| `hover_thrust_estimate` | Optional if your controller owns a valid hover-thrust estimator. Set `MPC_USE_HTE=1` only for that arrangement; publish a valid estimate with current timestamps and appropriate variance/status fields. Otherwise use the fixed calibrated `MPC_THR_HOVER`; this airframe defaults `MPC_USE_HTE=0`. |
| `trajectory_setpoint` | Needed for the commanded-descent test when Commander's `vehicle_control_mode.flag_control_climb_rate_enabled` is true. Publish the real NED trajectory; positive `velocity[2]` means down. Unused fields should be NaN. Direct-actuator Offboard does not enable that climb-rate flag, so this is not required just for its land-detection path. |

`vehicle_attitude_setpoint`, `vehicle_rates_setpoint`,
`vehicle_local_position_setpoint`, and `vehicle_torque_setpoint` are also useful
for telemetry, logs, and setpoint comparisons, as in the tested quad model.
They are not substitutes for the actual actuator and land-detector inputs.

### Thrust normalization and hover detection

`MulticopterLandDetector` calculates its throttle signal as
`-vehicle_thrust_setpoint.xyz[2]`. Without this publication, that signal starts
at zero. A stationary hover can then satisfy low-thrust and low-motion tests,
so outputting working motor commands alone is insufficient.

Use the normalized **collective lift demand actually applied by the controller**.
For a controller expressed in body forces, a useful convention is
`xyz = commanded_body_force / reference_max_lift_thrust`, with that reference
consistent with `MPC_THR_HOVER` and the controller's saturation. Do not average
motor PWM/RPM fractions and assume they are equivalent to thrust: the motor
and propeller response is nonlinear. In forward flight, cruise thrust belongs
on body X; lift thrust remains on body negative Z. Set the VTOL state to FW only
when that is physically true, not as a workaround for land detection.

There is no requirement to run the stock hover-thrust estimator. With
`MPC_USE_HTE=0`, calibrate `MPC_THR_HOVER` in the same thrust normalization as the
reported setpoint. The Gazebo approximation sets a nominal value of 0.4; that
number is not a hardware calibration. Relevant land-detector settings include
`MPC_THR_MIN`, `MPC_MANTHR_MIN`, `MPC_LAND_SPEED`, `MPC_LAND_CRWL`,
`LNDMC_TRIG_TIME`, `LNDMC_XY_VEL_MAX`, `LNDMC_Z_VEL_MAX`, `LNDMC_ROT_MAX`, and
`LNDMC_ALT_GND`. Sensor/estimator validity and distance-to-ground availability
also affect detection.

## What the model reads and leaves owned by PX4

Read `actuator_armed` for arming, prearming, lockdown, manual lockdown/kill, and
failsafe state. Read `vehicle_status`, `vehicle_control_mode`, `failsafe_flags`,
and `vehicle_land_detected` to follow the commanded mode and system state.
Continue to read EKF2 state topics and sensor validity as appropriate.

Do not publish competing `actuator_armed`, `vehicle_status`,
`vehicle_control_mode`, `failsafe_flags`, or `vehicle_land_detected` messages.
Use the normal RC/GCS arm action, or a supported `vehicle_command` arm request
and its acknowledgment, so Commander retains the arming checks. A controller
may request arming; it must not manufacture the armed state. Actuator drivers
also apply PX4's arming/lockdown/failsafe gates; do not bypass them through raw
physical PWM writes.

Prearmed servos may move if their output provider allows prearm control;
motor outputs require arming. In the model, gate/reset integrators and motor
commands appropriately on disarm and kill, and account for saved output
disarmed/failsafe values. An unused channel should not inadvertently command
an endpoint.

### Auto-disarm and failsafe behavior

Commander marks takeoff after `vehicle_land_detected.landed` changes from true
to false while armed. Until that happens, the preflight auto-disarm path remains
active (`COM_SPOOLUP_TIME` plus `COM_DISARM_PRFLT`; parameter defaults are 1 s
and 10 s). After takeoff, the normal landed auto-disarm uses `COM_DISARM_LAND`
(default 2 s). A `takeoff_status=FLIGHT` message alone does not change this.
Those are the generic Commander parameter defaults. The X200 airframe overrides
`COM_DISARM_LAND=-1` and `COM_DISARM_PRFLT=-1`, disabling these two timers by
default. It also sets `COM_KILL_DISARM=0` and `COM_LKDOWN_TKO=1`. Other lockdown
and failure-triggered actions remain; see the README's disarming section.
Correct landed detection is still required by other PX4 consumers; do not
hardcode an airborne state to substitute for a working detector.

**Retaining the PX4 failsafe state machine does not retain the excluded flight
controllers.** If Commander selects RTL, Land, Hold, or another recovery mode,
the Simulink controller must implement that requested behavior, or the operating
configuration must use a separately designed recovery mechanism. Removing the
stock controllers means PX4 cannot supply their control laws on demand.
Likewise, direct actuator output providers retain their last received values;
do not assume stopping one publisher automatically gives a complete controller
watchdog. Stop the Offboard heartbeat when the control computation is unhealthy,
monitor `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`, and verify the resulting behavior in
SITL. Mode requests, RC loss, data-link loss, and kill/disarm must all be handled
by the integrated design.

## Findings from the on-disk MATLAB models

The inspected `Jacks_eVTOL_Simulator/systems/HardwareModel.slx` publishes
`vehicle_thrust_setpoint` from `ControlOutput.Force / MaxTotalThrust`, with
current PX4 time, and publishes `offboard_control_mode` from its control-status
flags. It also publishes trajectory, attitude, rate, torque, and local-position
setpoints. Its quad behavior is a useful reference; it does not establish a
VTOL transition-state publisher for the new controller.

The inspected `S2F_X200_PX4_26b.slx` in the X200 simulation folder publishes
`actuator_motors`, `actuator_servos`, and `simulink_outbound`, but does not contain
publishers for `vehicle_thrust_setpoint`, `vtol_vehicle_status`, or
`offboard_control_mode`. Add the relevant interfaces before relying on PX4
land detection and Offboard supervision. The active/unsaved MATLAB model may
differ from this on-disk snapshot.

Also check its final `Send To Motors` and `Send To Servos` conversion blocks:
the inspected wiring applies the -1 bias **before** the 1/2000 gain, giving
`(PWM - 1) / 2000`. For a 1000–2000 us PWM input, that produces approximately
0.4995–0.9995: neither a full-range motor command nor a centered servo command.
For typical endpoint conventions, the expected conversions are:

- Non-reversible motor: `(PWM - 1000) / 1000`, yielding 0–1.
- Centered servo: `(PWM - 1500) / 500`, yielding -1–1.

Confirm the actual upstream units and wiring before changing the model. Match
servo direction and motor command interpretation to the output driver/model;
these equations do not define a thrust/RPM calibration. No MATLAB model has
been edited as part of this integration.

## SITL launch and virtual outputs

For an external MAVLink-connected dynamics simulator:

```bash
cd ~/PX4/v1.16.1-mod
PX4_SYS_AUTOSTART=22000 make px4_sitl_simulinkctrl none
```

Use `px4_sitl_default` instead if that is the minimal profile configured by
MATLAB setup. The external simulator must connect to `simulator_mavlink` and
supply its normal sensor/ground-truth stream; the airframe does not launch a
MATLAB dynamics simulation automatically. SITL virtual `PWM_MAIN` channels 1–5
map to motors 1–5; channels 6–12 map to servos 1–7. This is the virtual simulator
interface, not the hardware pin mapping. Existing saved parameters may override
these defaults.

For the experimental Gazebo model in the Gazebo fork:

```bash
make px4_sitl_simulinkctrl gz_s2f_x200
# For a headless session:
HEADLESS=1 make px4_sitl_simulinkctrl gz_s2f_x200
```

The Gazebo fork owns `models/s2f_x200` and `airframes/22001_gz_s2f_x200`; PX4
registers that launch target and copies the fork-owned script into SITL ROMFS.
Gazebo outputs use `SIM_GZ_EC_FUNC1..5` and `SIM_GZ_SV_FUNC1..7`, not physical
PWM routing. The airframe provides these mappings. Read the model's README
for its normalization and approximation limits.

## Integration checks before hardware use

Record a SITL log while exercising these cases with the actual controller:

1. Disarmed and prearmed: verify `actuator_armed`, motor stop, servo behavior,
   and no competing actuator publishers.
2. Arm and wait on the ground: verify the configured preflight auto-disarm.
3. Take off and hold a stationary hover: verify negative body-Z thrust,
   `vehicle_land_detected.landed=false`, and no false auto-disarm.
4. Transition both ways: verify the reported VTOL state and Commander's vehicle
   type/transition flags follow the physical transition.
5. Land, reduce thrust, and wait: verify land detection and landed auto-disarm.
6. Stop or stall controller updates, lose RC/data link, request recovery modes,
   and use kill/disarm: verify the designed response rather than assuming that
   the absent stock controllers will perform it.

Useful PX4 shell checks are `listener vehicle_thrust_setpoint`,
`listener vehicle_land_detected`, `listener vtol_vehicle_status`,
`listener actuator_armed`, `listener offboard_control_mode`, and `uorb top`.
The generated airframe/model packaging tests do not replace these closed-loop
controller tests.
