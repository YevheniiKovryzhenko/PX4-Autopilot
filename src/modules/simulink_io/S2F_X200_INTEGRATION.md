# S2F-X200: SITL milestone and hardware integration

The updated aircraft model has been exercised in SITL with and without lidar,
including takeoff, VTOL transitions, position flight, landing and disarming.
This establishes controller integration, not hardware flight validation.

## Firmware and build

Use the target board's `simulinkctrl` build and hardware airframe 22000. The
supported hardware configurations already include custom airframes, sensors,
EKF2, land_detector and the Simulink integration. The generated controller must
be built/deployed for the hardware target. No new board/CMake flags are needed
for these sensor and landing changes.

The VehicleIMU first-sample timestamp guard applies to hardware builds as well
as SITL. It prevents integrating elapsed time since boot as one sensor interval;
it does not change estimator or arming thresholds.

## Required controller feedback

- Publish truthful normalized vehicle_thrust_setpoint: upward lift is negative
  body Z. Its normalization must match the live MPC_THR_HOVER parameter. The
  current model reads that parameter rather than hardcoding the SITL value.
- In Altitude/Position, publish the actual commanded NED vertical velocity in
  trajectory_setpoint.velocity[2] (m/s, positive down), with PX4 boot-relative
  timestamps and NaN in unused floating fields.
- Simulink owns trajectory_setpoint. The airframe defaults SMG_OUT_TYPE=2 so
  guidance publishes its custom topic instead. Check saved parameter overrides.
- Read vehicle_land_detected.ground_contact for filtered contact handling.
  Permit actual thrust reduction during deliberate low-speed VTOL descent
  before contact; otherwise the airborne thrust floor can prevent detection.
  Keep normal PX4 thrust/motion checks and landing-confirmation times.
- The current TCS descent-relief threshold is -0.30 m/s upward-positive.
  Keep its magnitude above 1.1*LNDMC_Z_VEL_MAX (currently 0.275 m/s).
- Height/climb feedback uses fresh, valid EKF terrain distance, aligned local
  altitude as fallback, and -vz. Do not duplicate tilt correction or downstream
  height-offset tracking. Holding stale outputs is not a complete failsafe.
- VTOL reporting has a valid disarmed/landed MC reset for the vertical-takeoff
  configuration. Shadow controller stages do not establish flight regime.

## Hardware-specific settings

Do not copy SIM_GZ_* channels, the simulator's 838 rad/s ceiling, nominal hover
normalization of 0.4, simulated calibration, or simulated sensor offsets as
measured hardware values. Verify real actuator functions/directions, hover
normalization, sensor calibration, IMU orientation/position, rangefinder driver,
orientation, lever arm and expected on-ground reading.

The Gazebo lidar preset's EKF2_RNG_POS_X/Y/Z=[0.35,0,0.10] m describes only its
simulated mount in body FRD. Configure the actual hardware installation instead.
Range-aiding/height-reference settings must match available sensors. Validate
estimator-loss handling, touchdown and aborted landings on hardware separately.

The sensor-processing and TCS changes live in the Simulink model, not in PX4's
handwritten controller source. Deploy the current generated controller together
with the updated firmware and verify single ownership of all feedback topics.
