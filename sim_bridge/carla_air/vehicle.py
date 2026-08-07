"""The drone: arming, setpoints, telemetry — with CARLA-Air's quirks handled once.

Two behaviours measured by the conformance suite are wrapped here so no caller has to
remember them:

* **After `reset()` the vehicle does not hold station.** It runs away — a constant
  +7.06 m/s climb that never stops in one configuration (a session reached -1566 m NED
  before anyone noticed), a slow sink in another. The direction varies between sessions;
  that it does not hold does not. `reset()` below therefore always issues a position
  setpoint before returning, and refuses to return until the vehicle is tracking it.
* **`moveToPositionAsync().join()` returns at the target, then the vehicle relaxes
  ~4 m away** and holds there. Waypoint accuracy is metres, not centimetres. Nothing here
  hides that — but `goto()` reports the *measured* pose, never the commanded one, so an
  episode log cannot silently record a position the aircraft was not at.
"""
from __future__ import annotations

import math
import time

import airsim

from . import frames


class Vehicle:
    #: How close the aircraft must end up to the pose `reset()` was given, in metres.
    #:
    #: 1.5, not the 6.0 this shipped with. Measured over 32 resets: at 6 m the loop stopped as
    #: soon as it was inside tolerance and left 1.3-1.7 m on the table; asked for 1 m it
    #: reaches 0.5-0.9 m at every altitude with air beneath it. The looser value was not
    #: buying anything except an earlier exit.
    RESET_TOLERANCE_M = 1.5
    #: How many times `reset()` will re-command the hold before giving up and saying so.
    RESET_ATTEMPTS = 4
    #: Stop retrying once an attempt stops HELPING, rather than burning every attempt against
    #: something unreachable.
    #:
    #: This exists because of a floor, not a miss. Commanded to 3.5 m AGL the aircraft settles
    #: at 0.2 m — on the ground — and it does so identically across 8 resets at both speeds.
    #: Retrying cannot fix that, so a tolerance tight enough to be useful in the air would make
    #: every street-level reset burn four attempts and then report failure. An attempt that
    #: improves the miss by less than this is treated as converged-as-far-as-it-goes: the
    #: caller still gets the real position and the stage log still says how far out it is.
    #: See todo.md D-05 and tests/conformance/p12_reset_altitude.py.
    RESET_MIN_IMPROVEMENT_M = 0.3

    #: How long a ground-truth environment read stays good. Temperature, pressure and density
    #: do not meaningfully move over a flight, and re-reading them every tick is 20% of the
    #: sensor call's cost for no new information.
    ENV_CACHE_S = 5.0

    def __init__(self, client: airsim.MultirotorClient, name: str = "SimpleFlight"):
        self._c = client
        self._name = name
        #: Collisions at or before this sim timestamp belong to a previous episode.
        self._collision_epoch = 0
        self._env_cache = None
        self._env_at = 0.0

    # ---------- lifecycle ----------

    def reset(self, hold_ned=(0.0, 0.0, -40.0), speed=8.0, settle_s=2.0,
              on_stage=None, hard=False):
        """Reset to the start pose and leave the aircraft *holding a setpoint*.

        Never return a vehicle that has only been armed: see the module docstring.

        **PLACED, not flown.** This used to `reset()` and then `moveToPositionAsync` the
        aircraft from the world origin to the start - a real flight of ~200 m, ~20 s of the
        ~30 s a reset cost. `simSetVehiclePose` puts it there directly.

        `reset()` still runs first, and is still what makes this safe rather than merely
        fast: it clears the latched collision flag (without it, one crashed episode would
        mark every later one as collided from its first frame), cancels any command still
        executing from the previous episode, and returns the vehicle to a known disarmed
        state. Teleporting alone does none of that.

        The `moveToPositionAsync` at the end is NOT redundant. The aircraft is already at
        the pose, so it returns almost immediately - but it leaves a live setpoint behind,
        and a drone placed at 55 m with no setpoint simply falls. That invariant is the
        whole point of the module docstring.
        """
        # Optional so this class stays usable without a connection behind it - the 3.10
        # scripts and the tests call reset() directly.
        say = on_stage or (lambda _stage: None)

        n, e, d = (float(v) for v in hold_ned)
        if hard:
            # The old path. Kept because it is the only thing that provably returns the
            # simulator to a known state - but it is pathologically slow on repeat
            # (5.5 s, then >60 s, then a hang; see todo.md E-06), so it is no longer the
            # default. Use it when the simulator is already misbehaving.
            say("sim-reset")
            self._c.reset()
            time.sleep(3.0)
        else:
            # What client.reset() was actually being used for, done explicitly:
            #   * cancel whatever the previous episode left executing,
            #   * drop the aircraft out of API control so the teleport is not fought,
            #   * (the collision flag is handled by `_collision_epoch`, below).
            say("cancelling")
            self._c.cancelLastTask(vehicle_name=self._name)
            self._c.armDisarm(False, self._name)
            self._c.enableApiControl(False, self._name)
            time.sleep(0.3)

        # Teleport BEFORE arming, so SimpleFlight initialises its controller at the pose we
        # want rather than at the origin and then being yanked. `ignore_collision=True`
        # because the target may be inside geometry the collision check would object to, and
        # a scenario start is chosen deliberately.
        say("placing")
        self._c.simSetVehiclePose(
            airsim.Pose(airsim.Vector3r(n, e, d), airsim.Quaternionr(0.0, 0.0, 0.0, 1.0)),
            True, vehicle_name=self._name)
        time.sleep(0.5)

        say("arming")
        self._c.enableApiControl(True, self._name)
        self._c.armDisarm(True, self._name)
        say("holding")
        # An EXPLICIT heading hold, not AirSim's default. The default `yaw_mode` is
        # `YawMode(is_rate=True, yaw_or_rate=0.0)`, which reads as "hold zero" and means the
        # opposite: RATE control commanding zero rate, i.e. do not actively drive yaw at all.
        # The airframe keeps whatever angular momentum it arrives with, and the aircraft that
        # `reset()` claims to have placed at yaw zero is pointing somewhere else by the time
        # the caller sees it.
        #
        # Measured over 10 resets, drift during the settle:
        #     default              worst 65.2 deg, median  0.5 deg
        #     explicit hold        worst  0.9 deg, median  0.3 deg
        # Intermittent, which is what made it survive so long — most resets look fine.
        # See tests/conformance/p11_reset_attitude.py and todo.md D-02.
        #
        # And CONVERGE, rather than trusting one join(). `moveToPositionAsync().join()`
        # returns when SimpleFlight decides it has arrived, which is not the same as being
        # there: measured across ten real episodes commanded to z = -55, four began between
        # 18 and 37 m BELOW it, and the aircraft is always low, never high. The likely path is
        # that it is unpowered through the placement sleep and the arm, falls, and the hold
        # gives up before it has climbed back — which is why street level was the one accurate
        # altitude in the grid, having nowhere to fall to.
        #
        # So: command, check what actually happened, and command again. Bounded, because a
        # reset that cannot converge must fail loudly rather than spin — and `hard=True`
        # exists for a simulator that is genuinely wedged.
        # See tests/conformance/p12_reset_altitude.py and todo.md D-03.
        previous_miss = None
        for attempt in range(self.RESET_ATTEMPTS):
            self._c.moveToPositionAsync(
                n, e, d, speed,
                yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
                vehicle_name=self._name).join()
            time.sleep(settle_s)
            pos = self.state()["position"]
            miss = math.dist(pos, (n, e, d))
            if miss <= self.RESET_TOLERANCE_M:
                if attempt:
                    say(f"converged after {attempt + 1}")
                break
            if (previous_miss is not None
                    and previous_miss - miss < self.RESET_MIN_IMPROVEMENT_M):
                # Not converging. Say what it settled at rather than pretending the remaining
                # attempts might help — they demonstrably do not against a floor.
                say(f"stalled at {miss:.1f} m out; further attempts are not improving it")
                break
            previous_miss = miss
            say(f"re-holding ({miss:.1f} m out)")
        else:
            # Reported, not raised: a start pose that is metres out still flies, and failing
            # the episode outright would be worse than flying it with the error recorded.
            # The caller gets the real position back and can decide.
            say(f"NOT CONVERGED: {miss:.1f} m from the commanded pose")

        # Take the epoch AFTER everything has settled, and after a hard reset in particular:
        # a hard reset restarts sim time, so an epoch captured beforehand would be in the
        # future and would mask every real collision that followed.
        try:
            self._collision_epoch = int(
                self._c.simGetCollisionInfo(vehicle_name=self._name).time_stamp)
        except Exception:                                              # noqa: BLE001
            self._collision_epoch = 0
        say("settled")
        return self.state()

    def takeoff(self, altitude_ned=-30.0, speed=6.0, settle_s=1.0):
        """Arm, take off, and climb to `altitude_ned`, LEFT HOLDING A SETPOINT.

        Needed because `land()` ends with `armDisarm(False)` and `enableApiControl(False)` —
        after a landing the aircraft is inert, and until this existed the only way back into
        the air was `reset()`, which also teleports it to a different place.

        `takeoffAsync` only clears the ground by a couple of metres, so it is followed by a
        climb to the requested altitude at the current x/y. And it returns holding, never
        merely armed — same rule as `reset()`, for the same reason: an armed vehicle with no
        setpoint is the state this project has watched run away.

        `altitude_ned` is NED, not AGL, and the controller clamps NED altitude to
        [15, 120] m. On Town10HD the ground is NED z = +27.45, so -30 is about 57 m of climb.
        """
        self._c.enableApiControl(True, self._name)
        self._c.armDisarm(True, self._name)
        self._c.takeoffAsync(vehicle_name=self._name).join()
        p = self._c.getMultirotorState(vehicle_name=self._name).kinematics_estimated.position
        self._c.moveToPositionAsync(p.x_val, p.y_val, float(altitude_ned), speed,
                                    vehicle_name=self._name).join()
        time.sleep(settle_s)
        return self.state()

    def land(self):
        try:
            self._c.landAsync(vehicle_name=self._name).join()
        finally:
            self._c.armDisarm(False, self._name)
            self._c.enableApiControl(False, self._name)

    # ---------- telemetry ----------

    def state(self):
        s = self._c.getMultirotorState(vehicle_name=self._name)
        k = s.kinematics_estimated
        q = k.orientation
        return {
            "t": time.time(),
            "position": [k.position.x_val, k.position.y_val, k.position.z_val],
            "velocity": [k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val],
            "angular_velocity": [k.angular_velocity.x_val, k.angular_velocity.y_val,
                                 k.angular_velocity.z_val],
            "orientation": [q.w_val, q.x_val, q.y_val, q.z_val],
            "yaw": frames.quat_to_yaw(q.w_val, q.x_val, q.y_val, q.z_val),
            "landed": int(s.landed_state),
            "armed": True,
        }

    def sensors(self):
        """IMU, barometer, magnetometer and GPS in one call.

        AirSim creates these four automatically for a multirotor, so they answer even though
        `settings.json` declares no `Sensors` block — which is why they had been available and
        unpublished since the beginning. LiDAR and the distance sensor are NOT auto-created and
        throw an RPC error until declared; see S-02b.

        One call rather than four: these are four round trips to the same client for data read
        microseconds apart, and the bridge polls them on a timer.

        They carry AirSim's noise models — two IMU reads 0.4 s apart differ by ~1e-1 — so this
        is an instrument stream, not a second copy of ground truth.
        """
        # Five RPC round trips per call is what makes this expensive: at 20 Hz it costs ~28%
        # of the RGB capture rate, because the sidecar is one Python process and msgpack-rpc
        # holds the GIL while it serialises. The environment read is the one that can go —
        # air temperature and density are effectively constant over a flight, so it is cached
        # rather than fetched every tick.
        now = time.time()
        if self._env_cache is None or now - self._env_at > self.ENV_CACHE_S:
            self._env_cache = self._c.simGetGroundTruthEnvironment(vehicle_name=self._name)
            self._env_at = now
        env = self._env_cache

        imu = self._c.getImuData(vehicle_name=self._name)
        baro = self._c.getBarometerData(vehicle_name=self._name)
        mag = self._c.getMagnetometerData(vehicle_name=self._name)
        gps = self._c.getGpsData(vehicle_name=self._name)
        g = gps.gnss.geo_point
        v = gps.gnss.velocity
        return {
            "t": time.time(),
            "imu": {
                # AirSim body frame is already FRD, which is what PX4's SensorCombined wants,
                # so no axis juggling here — unlike the CARLA->NED hop in frames.py.
                "accel": [imu.linear_acceleration.x_val, imu.linear_acceleration.y_val,
                          imu.linear_acceleration.z_val],
                "gyro": [imu.angular_velocity.x_val, imu.angular_velocity.y_val,
                         imu.angular_velocity.z_val],
                "orientation": [imu.orientation.w_val, imu.orientation.x_val,
                                imu.orientation.y_val, imu.orientation.z_val],
                "t": imu.time_stamp,
            },
            "baro": {"altitude": baro.altitude, "pressure": baro.pressure,
                     "qnh": baro.qnh, "t": baro.time_stamp},
            # AirSim reports the field in Gauss, which is also PX4's unit. No conversion.
            "mag": {"field": [mag.magnetic_field_body.x_val, mag.magnetic_field_body.y_val,
                              mag.magnetic_field_body.z_val], "t": mag.time_stamp},
            "gps": {"lat": g.latitude, "lon": g.longitude, "alt": g.altitude,
                    "vel": [v.x_val, v.y_val, v.z_val],
                    "eph": gps.gnss.eph, "epv": gps.gnss.epv,
                    "fix": int(gps.gnss.fix_type), "valid": bool(gps.is_valid),
                    "t": gps.time_stamp},
            # Temperature in KELVIN from AirSim; PX4's VehicleAirData wants Celsius.
            "env": {"temperature_k": env.temperature, "pressure": env.air_pressure,
                    "density": env.air_density, "gravity": env.gravity.z_val},
        }

    def collision(self):
        """Collisions since the last reset, not since the simulator started.

        AirSim LATCHES `has_collided` - it stays true until a full `client.reset()`. That
        made the sim reset load-bearing for scoring: without it, one crashed episode would
        mark every later one as collided from its first frame.

        Comparing timestamps removes that dependency, and is better anyway: the epoch is
        explicit and per-vehicle, rather than a side effect of a global operation that costs
        a minute (see todo.md E-06).
        """
        c = self._c.simGetCollisionInfo(vehicle_name=self._name)
        fresh = bool(c.has_collided) and int(c.time_stamp) > self._collision_epoch
        return {
            "has_collided": fresh,
            "object_name": c.object_name,
            "position": [c.position.x_val, c.position.y_val, c.position.z_val],
            "object_id": int(c.object_id),
            "penetration_m": float(c.penetration_depth),
            "time_stamp": int(c.time_stamp),
        }

    # ---------- control ----------

    def velocity(self, vx: float, vy: float, vz: float, duration: float = 0.5, yaw_deg=None):
        """One velocity setpoint, NED, m/s.

        `duration` must exceed the caller's resend period or the vehicle stalls between
        commands. The control node streams at 10 Hz with duration 0.5 s.
        """
        kw = {"vehicle_name": self._name}
        if yaw_deg is not None:
            kw["yaw_mode"] = airsim.YawMode(False, yaw_deg)
        self._c.moveByVelocityAsync(float(vx), float(vy), float(vz), float(duration), **kw)

    def goto(self, x: float, y: float, z: float, speed: float = 6.0, settle_s: float = 0.0):
        """Blocking position move. Returns the *measured* pose, not the commanded one."""
        self._c.moveToPositionAsync(float(x), float(y), float(z), float(speed),
                                    vehicle_name=self._name).join()
        if settle_s:
            time.sleep(settle_s)
        return self.state()

    def attitude(self, roll: float, pitch: float, yaw: float, z: float,
                 duration: float = 0.2):
        """Hold a roll/pitch/yaw attitude at NED altitude `z` for `duration` seconds.

        `moveByRollPitchYawZAsync` keeps altitude on a Z controller while the three angles are
        commanded directly — the closest thing SimpleFlight has to PX4's attitude mode. The
        alternative, `...ThrottleAsync`, hands altitude to the caller as a normalised throttle,
        which is a different and much sharper knife.

        Not joined: at a 20 Hz command stream, blocking for the duration would let each command
        overrun the next. AirSim replaces the active command on the next call, which is what a
        setpoint stream wants.
        """
        # PITCH AND YAW ARE NEGATED, ROLL IS NOT. Measured, not guessed: commanding through
        # this API and reading the resulting orientation back out of AirSim's own state gives
        #
        #     roll  +12 deg -> +12.0      correct
        #     pitch +15 deg -> -15.0      inverted
        #     yaw   +40 deg -> -40.0      inverted
        #
        # symmetric and exact on both signs, so it is a convention mismatch rather than drift:
        # `moveByRollPitchYawZAsync` does not use the same handedness for pitch and yaw that
        # AirSim's own orientation reporting does. Correcting here means everything above this
        # line - PX4 messages, the ROS graph, the docs - speaks one consistent NED/FRD
        # convention, and the quirk stays contained to the one call that has it.
        self._c.moveByRollPitchYawZAsync(float(roll), -float(pitch), -float(yaw), float(z),
                                         float(duration), vehicle_name=self._name)
        return {"roll": roll, "pitch": pitch, "yaw": yaw, "z": z}

    def yaw(self, deg: float, timeout_s: float = 10.0):
        self._c.rotateToYawAsync(float(deg), timeout_sec=timeout_s, vehicle_name=self._name).join()
        return self.state()

    def hold(self):
        """Stop. Distinct from 'send nothing', which is what makes it run away."""
        s = self.state()
        p = s["position"]
        self._c.moveToPositionAsync(p[0], p[1], p[2], 1.0, vehicle_name=self._name)
        return s

    # ---------- helpers ----------

    def distance_to(self, ned) -> float:
        p = self.state()["position"]
        return math.dist(p, list(ned))
