defmodule Goatbot.Robot do
  use BB, extensions: [BB.Parameter.Store.CubDB.Dsl]

  import BB.Unit

  commands do
    command :arm do
      # A custom handler so that arming lands in the state the robot's
      # attitude calls for rather than always in `:idle`. `arm true` keeps
      # `BB.Safety.arm/1` routing through the command system.
      handler(BB.NSK.Command.Arm)
      allowed_states([:disarmed])
      arm(true)
    end

    command :disarm do
      handler(BB.Command.Disarm)
      allowed_states([:idle, :balancing, :fallen])
    end

    # Disarmed only: taking the operating system out from under a balancing
    # robot drops it on the floor.
    command :poweroff do
      handler(BB.NSK.Command.Poweroff)
      allowed_states([:disarmed])
    end

    command :stand do
      handler(BB.NSK.Command.Stand)
      allowed_states([:idle, :fallen])
    end

    # From `:idle` as well as `:balancing`, because a robot armed while
    # already on its side has fallen over without ever having balanced.
    command :fall do
      handler(BB.NSK.Command.Fall)
      allowed_states([:balancing, :idle])
    end
  end

  topology do
    # The robot is not bolted to anything, so the chain starts at the world
    # and reaches the body through the two ways it can move: across the
    # ground, and leaning about the wheel axis. Neither joint has an
    # actuator, and with no encoders the ground pose stays at identity —
    # only the lean is observable.
    link :world do
      joint :ground do
        type(:planar)

        axis do
        end

        link :ground_contact do
          joint :lean do
            type(:revolute)
            # Pitch about +Y, so leaning forwards is positive. The axis
            # sits a wheel radius above the ground — half of a 43mm wheel,
            # and anything under 32mm leaves the body resting on the floor
            # with the wheels spinning in the air.
            axis(roll: ~u(-90 degree))
            origin(z: ~u(21.5 millimeter))

            limit(
              lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(0 newton_meter),
              velocity: ~u(0 radian_per_second)
            )

            link :base_link do
              # From the CAD model, measured against the wheel axis.
              visual do
                box(x: ~u(20.7 millimeter), y: ~u(99 millimeter), z: ~u(124 millimeter))
                origin(x: ~u(1.75 millimeter), z: ~u(46 millimeter))
              end

              inertial do
                # The height is measured. The fore-aft is not: it is
                # back-derived from the lean the robot actually balances at,
                # so the geometry and the `:balance` setpoint tell the same
                # story. See `mix help bb_nsk.install`.
                origin(x: ~u(2.4 millimeter), z: ~u(45.5 millimeter))

                # Weighed with the wheels off: their mass sits on the axle
                # where it makes no toppling torque, so the body alone is
                # the pendulum.
                mass(~u(138 gram))

                # A uniform box of the body's dimensions about its centre of
                # mass. An approximation — the panel is on the front and the
                # battery is one lump — but not a negligible one: the body's
                # own pitch inertia is a third of the total about the wheel
                # axis.
                inertia(
                  ixx: ~u(2895 gram_square_centimeter),
                  iyy: ~u(1818 gram_square_centimeter),
                  izz: ~u(1176 gram_square_centimeter),
                  ixy: ~u(0 gram_square_centimeter),
                  ixz: ~u(0 gram_square_centimeter),
                  iyz: ~u(0 gram_square_centimeter)
                )
              end

              joint :left_wheel_joint do
                type(:continuous)
                axis(roll: ~u(-90 degree))
                origin(y: ~u(55 millimeter))

                limit(effort: ~u(0.2 newton_meter), velocity: ~u(20 radian_per_second))

                actuator(
                  :left_wheel,
                  {BB.NSK.Wheel,
                   forward_pwm: 3,
                   reverse_pwm: 2,
                   enable_pin: "PE6",
                   deadband: param([:motor, :deadband])}
                )

                link :left_wheel_link do
                  inertial do
                    mass(~u(15 gram))

                    inertia(
                      ixx: ~u(17.33 gram_square_centimeter),
                      iyy: ~u(34.67 gram_square_centimeter),
                      izz: ~u(17.33 gram_square_centimeter),
                      ixy: ~u(0 gram_square_centimeter),
                      ixz: ~u(0 gram_square_centimeter),
                      iyz: ~u(0 gram_square_centimeter)
                    )
                  end
                end
              end

              joint :right_wheel_joint do
                type(:continuous)
                axis(roll: ~u(-90 degree))
                origin(y: ~u(-55 millimeter))

                limit(effort: ~u(0.2 newton_meter), velocity: ~u(20 radian_per_second))

                actuator(
                  :right_wheel,
                  {BB.NSK.Wheel,
                   forward_pwm: 5,
                   reverse_pwm: 4,
                   enable_pin: "PE11",
                   deadband: param([:motor, :deadband])}
                )

                link :right_wheel_link do
                  inertial do
                    mass(~u(15 gram))

                    inertia(
                      ixx: ~u(17.33 gram_square_centimeter),
                      iyy: ~u(34.67 gram_square_centimeter),
                      izz: ~u(17.33 gram_square_centimeter),
                      ixy: ~u(0 gram_square_centimeter),
                      ixz: ~u(0 gram_square_centimeter),
                      iyz: ~u(0 gram_square_centimeter)
                    )
                  end
                end
              end

              # The chip publishes in its own axes and a `sensor` has no origin — the
              # link it hangs off is its frame — so the mounting is links of its own and
              # everything downstream gets the transform from the kinematics.
              #
              # **Two joints rather than one origin carrying both**, because
              # `BB.Math.Transform.from_origin/1` composes the rotation before the
              # translation: one origin with both would put the chip 29.5mm *forward* of
              # the wheel axis instead of above it.
              joint :imu_mount_joint do
                type(:fixed)
                origin(x: ~u(-4.9 millimeter), z: ~u(29.5 millimeter))

                link :imu_mount do
                  joint :imu_joint do
                    type(:fixed)
                    origin(pitch: ~u(90 degree), yaw: ~u(-90 degree))

                    link :imu_link do
                      # The driver stops rather than declining when the chip isn't
                      # there, which on a host would take the supervision tree with it.
                      if Mix.target() != :host do
                        # No magnetometer, so the chip publishes an identity
                        # orientation and this fuses a real one.
                        sensor :imu,
                               {BB.Sensor.BMI323,
                                bus: "i2c-0",
                                address: 0x68,
                                mode: :polling,
                                accelerometer_range: 4,
                                accelerometer_odr: 200,
                                gyroscope_range: 500,
                                gyroscope_odr: 200,
                                publish_rate: param([:sampling, :publish_rate])} do
                          estimator(
                            :orientation,
                            {BB.Estimator.Ahrs.Mahony,
                             kp: param([:ahrs, :kp]), ki: param([:ahrs, :ki])}
                          )
                        end
                      end
                    end
                  end
                end
              end
            end

            # No actuator, but the lean is observable: the IMU hangs off a link below
            # this joint, and what it reports about gravity is exactly this joint's
            # configuration.
            sensor(:lean_angle, BB.NSK.Sensor.Lean)
          end
        end

        # The heading is observable even though the position isn't: the gyro
        # measures yaw rate directly, and integrating it is a real angle where dead
        # reckoning `x` and `y` from wheel commands would be a fiction. So this
        # joint's `theta` means something and its translation still doesn't.
        sensor(:heading, BB.NSK.Sensor.Heading)
      end
    end
  end

  # Robot-level sensors, not attached to any link.
  sensors do
    # Battery voltage and current from the STM32 coprocessor, published as
    # `BB.Message.Sensor.BatteryState` on [:sensor, :power], and its button
    # events on [:buttons]. Idle on the host, where there's no STM32.
    sensor(:power, Goatbot.Sensor.Power)
  end

  # `/root` is the Nerves application data partition and does not exist on a
  # host. The DSL is compiled, so this is decided when the firmware is built.
  #
  # `tmp/` rather than `_build/` on the host — which is what
  # `bb_parameter_store_cubdb` would pick for a project with no Nerves in it
  # — because these are tuned values and `mix clean` should not take them.
  parameter_store_cubdb do
    data_dir(
      if Mix.target() == :host,
        do: "tmp/goatbot_params",
        else: "/root/goatbot_params"
    )
  end

  parameters do
    group :motor do
      # Measured unloaded, so the figure under the robot's own weight will be
      # higher — which is why it is tunable.
      param(:deadband,
        type: :float,
        default: 0.01,
        min: 0.0,
        max: 0.9,
        doc: "Duty below which the motor doesn't turn, as a fraction of full scale"
      )
    end

    group :sampling do
      # This is the balance loop's rate. The robot falls with a 68ms time
      # constant, so samples per fall matter more than they sound — see
      # `mix help bb_nsk.add_imu`.
      #
      # The chip's ODR is 200 Hz, so going above that repeats samples unless the
      # ODR below is raised with it.
      param(:publish_rate,
        type: {:unit, :hertz},
        default: ~u(200 hertz),
        min: ~u(10 hertz),
        max: ~u(400 hertz),
        doc: "How often the IMU publishes, which is the rate the balance loop runs at"
      )
    end

    group :ahrs do
      # The filter sits *inside* the control loop, so its lag is part of the
      # plant the balance gains are tuned against — changing one invalidates the
      # tuning of the other.
      #
      # 0.2 rather than 0.4, from replaying recorded IMU data through both
      # filters: half as quick to mistake a steady acceleration for lean (0.9°
      # of a 5° fake tilt after 1 s, against 1.65°), and the lowest error while
      # being pushed back and forth. Madgwick was worse on both.
      param(:kp,
        type: :float,
        default: 0.2,
        min: 0.0,
        max: 5.0,
        doc:
          "How hard the accelerometer may pull the estimate. Small: it can't tell lean from acceleration"
      )

      # Defaults to zero in the library, so it has to be set. Matters more at a
      # low kp, which leans on the gyro for longer.
      param(:ki,
        type: :float,
        default: 0.005,
        min: 0.0,
        max: 0.5,
        doc: "Gyroscope bias estimation. Zero is proportional-only, which drifts"
      )
    end

    group :balance do
      # Not zero: the centre of mass sits ahead of the wheel axis, so the robot
      # is only in equilibrium leaning back far enough to bring it over them.
      # -2.5 with the current battery: at -3 it ran away backwards, at -2
      # forwards. The velocity loop takes up what's left.
      param(:setpoint,
        type: {:unit, :degree},
        default: ~u(-2.5 degree),
        min: ~u(-30 degree),
        max: ~u(30 degree),
        doc: "The lean the balance controller holds, negative leaning back"
      )

      # 100 with `Goatbot.Balancer`: 150 and up shook (2-5 deg of fast
      # oscillation), and every degree of shake is wheel speed not available for
      # catching the robot. The stock controller's 180 was for the old battery.
      param(:proportional_gain,
        type: :float,
        default: 100.0,
        min: 0.0,
        max: 250.0,
        doc: "Wheel velocity per radian of lean error, in 1/s"
      )

      # 1.5 at a kp of 100. The stock note still stands: much above 4 the
      # derivative term amplifies gyro noise into the command.
      param(:derivative_gain,
        type: :float,
        default: 1.5,
        min: 0.0,
        max: 20.0,
        doc: "Wheel velocity per radian per second of lean rate"
      )

      # Large, and doing the real work. The wheels behave like a speed servo
      # (torque goes with command minus current speed), so holding the body at
      # a lean takes a command that keeps growing while the lean persists. P and
      # D alone can't: the robot ran away within seconds every time. In
      # simulation the threshold was around 450; 400 balances on the robot.
      param(:integral_gain,
        type: :float,
        default: 400.0,
        min: 0.0,
        max: 500.0,
        doc: "Wheel velocity per radian-second of accumulated lean error, in 1/s²"
      )

      # Room for the integral to reach the motors' full speed (0.5 * 400 = 200
      # rad/s, well past the ~31 they can do), so the clamp never binds in normal
      # running and only stops a runaway winding it up without end.
      param(:integral_limit,
        type: :float,
        default: 0.5,
        min: 0.0,
        max: 1.0,
        doc: "Clamp on the accumulated lean error, in radian-seconds"
      )

      # Used by the stock `bb_nsk` balancer only. More than a bias corrector: the
      # commanded velocity is the only estimate of speed this hardware can
      # produce, so this is the only velocity feedback in the system. Without it
      # the robot rocks as it accelerates away.
      param(:trim_gain,
        type: :float,
        default: 0.01,
        min: 0.0,
        max: 1.0,
        doc: "How fast the setpoint chases away a persistent wheel command. Slow on purpose"
      )

      param(:trim_limit,
        type: {:unit, :degree},
        default: ~u(5 degree),
        min: ~u(0 degree),
        max: ~u(15 degree),
        doc: "How far the trim may wander before it is telling you something else is wrong"
      )

      param(:fall_angle,
        type: {:unit, :degree},
        default: ~u(30 degree),
        min: ~u(5 degree),
        max: ~u(80 degree),
        doc: "Lean error past which it gives up and waits to be stood back up"
      )

      # Tight on purpose. Any looser and the robot takes over while still moving
      # in the operator's hand, and then spends its life answering that
      # transient rather than balancing.
      param(:catch_angle,
        type: {:unit, :degree},
        default: ~u(2 degree),
        min: ~u(1 degree),
        max: ~u(30 degree),
        doc: "Lean error within which it will take over again"
      )

      param(:catch_rate,
        type: {:unit, :degree_per_second},
        default: ~u(4 degree_per_second),
        min: ~u(1 degree_per_second),
        max: ~u(200 degree_per_second),
        doc: "How still it must be held before taking over, so it can't grab mid-air"
      )
    end

    # `Goatbot.VelocityController`, which leans the robot against its own
    # velocity. With a large `integral_gain` the wheel command tracks the wheel
    # speed, so the filtered command *is* the velocity and `velocity_damping`
    # does the work. That's Kv * r: 0.0043 is Kv 0.2 lean-radians per m/s.
    group :velocity do
      # Kv 0.2. Kv 0.3 (with a 0.1 s filter) braked harder in simulation, but
      # on the robot it shook itself over within half a second of standing up:
      # the filtered command is the balance loop's own output, so a stiffer,
      # faster version of it feeds that output straight back in.
      param(:velocity_damping,
        type: :float,
        default: 0.0043,
        min: 0.0,
        max: 0.02,
        doc: "Radians of lean per rad/s of filtered wheel command"
      )

      # Off. Integrating the command was right for a balance loop without an
      # integral, where the command meant acceleration; with one, it's double
      # counting.
      param(:velocity_gain,
        type: :float,
        default: 0.0,
        min: 0.0,
        max: 0.05,
        doc: "Radians of lean per unit of integrated wheel command"
      )

      # 0.2. Shorter means less lag and less rolling back after braking in
      # simulation, but 0.05 wobbled at ~5 Hz on the robot and 0.1 (with Kv 0.3)
      # fell over at start-up.
      param(:velocity_filter,
        type: :float,
        default: 0.2,
        min: 0.1,
        max: 10.0,
        doc: "Seconds the wheel command is averaged over"
      )

      param(:velocity_limit,
        type: {:unit, :degree},
        default: ~u(5 degree),
        min: ~u(0 degree),
        max: ~u(15 degree),
        doc: "Largest lean offset it may ask for"
      )

      # Driving: a held throttle sets a target speed the loop leans towards.
      # 0.2 m/s at 0.2 m/s², which drove well on the robot. Simulation
      # reckoned 0.3 at 0.5 m/s² would be fine; on the robot it wasn't, so
      # treat the model as optimistic and step up from here with recordings.
      param(:drive_speed,
        type: :float,
        default: 0.2,
        min: 0.0,
        max: 0.3,
        doc: "Target speed, in m/s, at full throttle"
      )

      param(:drive_accel,
        type: :float,
        default: 0.2,
        min: 0.05,
        max: 1.0,
        doc: "How fast, in m/s², the target speed may change"
      )

      # Letting go should stop it, not let it coast. Not too fast: the drop in
      # target speed becomes a step in the lean target (about 3.4° from 0.3 m/s
      # at 3 m/s²), which kicks the balance loop.
      param(:drive_decel,
        type: :float,
        default: 1.0,
        min: 0.1,
        max: 10.0,
        doc: "How fast, in m/s², the target speed may fall back towards zero"
      )

      # The dead-man: a driver who stops sending stops the robot.
      param(:drive_timeout,
        type: :integer,
        default: 300,
        min: 50,
        max: 2000,
        doc: "Milliseconds a drive message stays good for"
      )

      # Only gates `velocity_gain`'s integration, which is off. Left high so it
      # doesn't hide the shove if that's ever turned back on.
      param(:velocity_ceiling,
        type: :float,
        default: 50.0,
        min: 0.1,
        max: 50.0,
        doc: "Average wheel command, in rad/s, above which it stops integrating"
      )
    end

    group :drive do
      # Parameter names are unique across *every* group, not just within one, so
      # these cannot be `:limit` and `:slew` — `:yaw` already has a `:limit`.
      param(:authority,
        type: {:unit, :degree},
        default: ~u(2 degree),
        min: ~u(0 degree),
        max: ~u(12 degree),
        doc: "Lean a full throttle may spend. Bounds acceleration, not speed"
      )

      param(:ramp,
        type: {:unit, :degree_per_second},
        default: ~u(8 degree_per_second),
        min: ~u(1 degree_per_second),
        max: ~u(90 degree_per_second),
        doc: "How fast the drive lean may change, so a thumb slammed over ramps rather than steps"
      )

      # Four times `ramp`: slow to wind on so a thumb slammed over does not lurch,
      # fast to unwind so letting go stops the robot promptly. If letting go
      # still feels slow, this is the knob.
      param(:release,
        type: {:unit, :degree_per_second},
        default: ~u(32 degree_per_second),
        min: ~u(1 degree_per_second),
        max: ~u(180 degree_per_second),
        doc: "How fast the drive lean returns towards zero, which is how quickly letting go stops"
      )
    end

    group :yaw do
      # Holds a heading, so a wheel finding more grip than the other doesn't
      # quietly turn the robot.
      #
      # The stock code said both signs were confirmed on hardware; on this
      # robot the stock mixing spun it, and `Goatbot.Balancer` mixes the other
      # way round. Confirmed live: a 13° error turned back to within 1° in a
      # second at gain 4.
      # 16, the stock value. Tested live: 8 held within ±1.6° with the calmest
      # yaw rate (±8°/s); 16 holds within ±3.5° but twitchier (±12°/s, peaks of
      # 50°/s), and once oscillated hard enough to be reverted. Lower it if the
      # robot twists back and forth.
      param(:gain,
        type: :float,
        default: 16.0,
        min: 0.0,
        max: 50.0,
        doc: "Differential wheel velocity per radian of heading error, in 1/s"
      )

      # Never swept — part of a tested configuration rather than a tested value.
      param(:damping,
        type: :float,
        default: 0.5,
        min: 0.0,
        max: 20.0,
        doc: "Differential wheel velocity per radian per second of yaw rate"
      )

      # Velocity spent turning is velocity unavailable for staying upright, and
      # straightening up is not worth falling over for. 8, not the stock 3:
      # turning at 2 rad/s takes 5.1 of differential by itself, and the
      # balancer still cuts the differential back whenever it needs the headroom.
      param(:limit,
        type: :float,
        default: 8.0,
        min: 0.0,
        max: 15.0,
        doc: "How much wheel velocity the heading may spend, in rad/s"
      )

      # No magnetometer, so the heading drifts and the held heading must follow
      # it. `Goatbot.YawController` learns the gyro's bias (-0.6°/s on this
      # robot, 36°/min) while it isn't balancing and takes it out, so this only
      # has to absorb what's left: long enough to hold a direction for minutes.
      # The stock controller, without the calibration, needed 10.
      param(:tau,
        type: :float,
        default: 60.0,
        min: 0.5,
        max: 120.0,
        doc: "Seconds for the held heading to follow the measured one"
      )

      # How fast ←/→ in the dashboard turns the robot: about 115°/s.
      param(:turn_rate,
        type: :float,
        default: 2.0,
        min: 0.1,
        max: 4.0,
        doc: "Turn rate, in rad/s, for a full turn request"
      )

      # track / (2 * wheel radius) = 0.11 m / 0.043 m: the wheel-speed
      # difference a turn needs, so it doesn't have to wait for an error.
      param(:feedforward,
        type: :float,
        default: 2.56,
        min: 0.0,
        max: 5.0,
        doc: "Wheel rad/s of differential per rad/s of requested turn"
      )
    end
  end

  controllers do
    controller(
      :pid_balancer,
      {Goatbot.Balancer,
       setpoint: param([:balance, :setpoint]),
       kp: param([:balance, :proportional_gain]),
       ki: param([:balance, :integral_gain]),
       kd: param([:balance, :derivative_gain]),
       integral_limit: param([:balance, :integral_limit]),
       fall_angle: param([:balance, :fall_angle])},
      # No hardware of its own (it only publishes to the wheels, which are
      # swapped for simulated ones), so it can run under SIMULATE too.
      simulation: :start
    )

    controller(
      :velocity_hold,
      {Goatbot.VelocityController,
       gain: param([:velocity, :velocity_gain]),
       filter: param([:velocity, :velocity_filter]),
       limit: param([:velocity, :velocity_limit]),
       ceiling: param([:velocity, :velocity_ceiling]),
       damping: param([:velocity, :velocity_damping]),
       drive_speed: param([:velocity, :drive_speed]),
       drive_accel: param([:velocity, :drive_accel]),
       drive_decel: param([:velocity, :drive_decel]),
       drive_timeout: param([:velocity, :drive_timeout])},
      simulation: :start
    )

    controller(
      :yaw_hold,
      {Goatbot.YawController,
       gain: param([:yaw, :gain]),
       damping: param([:yaw, :damping]),
       limit: param([:yaw, :limit]),
       feedforward: param([:yaw, :feedforward]),
       tau: param([:yaw, :tau]),
       drive_timeout: param([:velocity, :drive_timeout])},
      simulation: :start
    )

    # The stock balancer, swapped out for `Goatbot.Balancer` above. Only one of
    # the two may run — both would be driving the same wheels.
    #
    # Without it, `arm` lands in `:idle` rather than `:balancing`, so hand over
    # with `Goatbot.Robot.stand()`.
    #
    # controller(
    #   :balancer,
    #   {BB.NSK.Balance.Controller,
    #    setpoint: param([:balance, :setpoint]),
    #    proportional_gain: param([:balance, :proportional_gain]),
    #    derivative_gain: param([:balance, :derivative_gain]),
    #    trim_gain: param([:balance, :trim_gain]),
    #    trim_limit: param([:balance, :trim_limit]),
    #    fall_angle: param([:balance, :fall_angle]),
    #    catch_angle: param([:balance, :catch_angle]),
    #    catch_rate: param([:balance, :catch_rate]),
    #    yaw_gain: param([:yaw, :gain]),
    #    yaw_damping: param([:yaw, :damping]),
    #    yaw_limit: param([:yaw, :limit]),
    #    yaw_tau: param([:yaw, :tau]),
    #    drive_limit: param([:drive, :authority]),
    #    drive_slew: param([:drive, :ramp]),
    #    drive_release: param([:drive, :release])}
    # )

    controller(:leds, {BB.NSK.Leds.Controller, brightness: 40})
  end

  states do
    state(:balancing, doc: "Actively holding itself upright")
    state(:fallen, doc: "Past recovering, wheels braked, waiting to be stood back up")
  end
end
