# AlonLib

A Kotlin utility library for FTC (*FIRST* Tech Challenge) robot code: unit-safe wrappers for
motors/servos/sensors, a units system (`Length`, `AngularVelocity`, `Rotation2d` extensions),
PID/feedforward helpers, moving-window filters, a full command-based framework, a mecanum
drivetrain, and a desktop hardware emulator so OpModes can be run and debugged without a robot.

## Table of contents

- [Installing](#installing)
- [Requirements](#requirements)
- [Structure](#structure)
- [Quick start](#quick-start)
- [API reference](#api-reference)
  - [Root — `alonlib`](#root--alonlib)
  - [`units/`](#units)
  - [`math/`](#math)
  - [`math/filters/movingwindowfilters/`](#mathfiltersmovingwindowfilters)
  - [`hardware/`](#hardware)
  - [`drives/mecanumDrive/`](#drivesmecanumdrive)
  - [`commands/`](#commands)
- [Running on a desktop emulator instead of a robot](#running-on-a-desktop-emulator-instead-of-a-robot)
- [Building locally](#building-locally)

## Installing

Add JitPack as a repository and the library as a dependency in your `TeamCode` module.

`settings.gradle`:

```groovy
dependencyResolutionManagement {
    repositories {
        // ...
        maven { url = 'https://jitpack.io' }
    }
}
```

`TeamCode/build.gradle`:

```groovy
dependencies {
    implementation 'com.github.alonHamb:AlonLib:v11.2.1'
}
```

Use a specific tag (recommended, e.g. `v11.2.1`), a commit hash, or `<branch>-SNAPSHOT` to track a
branch directly.

## Requirements

The FTC SDK (`org.firstinspires.ftc:*`, v11.1.0), RoadRunner (`com.acmerobotics.roadrunner:*`), and
FTC Dashboard are bundled as `api` dependencies, so they come along transitively — you don't need to
declare any of them yourself just to use AlonLib's classes. AlonLib's command framework, geometry
types, and PID/feedforward controllers are its own reimplementation (no `org.solverslib:core`
dependency).
If your project also depends on `FtcRobotController` directly (the usual FTC project setup, needed
to actually build/run the robot controller app), Gradle will de-duplicate the shared FTC SDK version
automatically as long as the versions match.

## Structure

Everything lives under `alonlib/src/main/java/org/firstinspires/ftc/teamcode/alonlib`:

- `Logging.kt` — print helpers and telemetry throttling
- `units/` — `Length`, `AngularVelocity`, `Rotation2d` extensions, unit conversions, `Alliance`
- `math/` — PID/feedforward gains, deadband/interpolation helpers, moving-window filters
- `hardware/` — `HaMotor`, `HaServo`, `HaLimelight3A`, `HaPinPoint`
- `drives/` — `HaMecanumDrive`, a mecanum drivetrain subsystem
- `commands/` — a full command-based framework (`Command`, `CommandScheduler`, `Subsystem`,
  composite commands, ...), extensions, factories, and a RoadRunner `Action` → `Command` bridge

`alonlib-emulator/src/main/java` has the desktop-emulator adapters (`EmulatedRobot`, `EmulatedHub`,
`EmuDcMotorEx`, ...) — see [Running on a desktop emulator instead of a robot](#running-on-a-desktop-emulator-instead-of-a-robot),
below.

## Quick start

A minimal `LinearOpMode` wiring up a drivetrain built from `HaMotor`s and a servo through `HaServo`,
using AlonLib's units system throughout instead of raw doubles:

```kotlin
package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.alonlib.drives.mecanumDrive.HaMecanumDrive
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

@TeleOp(name = "Quick Start Teleop")
class QuickStartTeleop : LinearOpMode() {
    override fun runOpMode() {
        val frontLeft = HaMotor(hardwareMap, "front left motor", HaMotor.GoBILDA.RPM_435)
        val frontRight = HaMotor(hardwareMap, "front right motor", HaMotor.GoBILDA.RPM_435)
        val backLeft = HaMotor(hardwareMap, "back left motor", HaMotor.GoBILDA.RPM_435)
        val backRight = HaMotor(hardwareMap, "back right motor", HaMotor.GoBILDA.RPM_435)
        val drive = HaMecanumDrive(frontLeft, frontRight, backLeft, backRight)

        val hood = HaServo(hardwareMap, "hood servo", Data.Servos.Mode.FULL_RANGE, Data.Servos.Type.Speed)

        waitForStart()

        while (opModeIsActive()) {
            drive.driveRobotCentric(
                gamepad1.left_stick_x.toDouble(),
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.right_stick_x.toDouble()
            )

            if (gamepad1.a) hood.position = 30.0.degrees

            frontLeft.update()
            frontRight.update()
            backLeft.update()
            backRight.update()
        }
    }
}
```

Notes:
- `HaMotor.update()` must be called every loop for any of its closed-loop `runMode`s
  (`PositionControl`/`VelocityControl`) or current limiting to actually take effect — see
  [`HaMotor`](#hamotor).
- `HaMecanumDrive` built from four `HaMotor`s drives their `percentOutput` directly, bypassing each
  `HaMotor`'s own PID/current-limiting layer — use it for simple stick-driven teleop, and drive the
  individual `HaMotor`s yourself (`percentOutput`, `velocity`, or `position`) when you need
  closed-loop control per wheel.
- See [`units/`](#units) for why `30.0.degrees` is a `Rotation2d`, not a raw `Double`.

## API reference

Every public class, function, and property in `alonlib`, grouped by package/file. Types not
exhaustively covered here — this library's own `Rotation2d`/`Pose2d`/`Command` (see
`math/geometry`/`commands`), the FTC SDK's `HardwareMap`/`Telemetry`, RoadRunner's `Action` — are
linked out to context instead.

### Root — `alonlib`

`Logging.kt`

| Symbol | Description |
| --- | --- |
| `robotPrint(message: Any?)` | Prints `message` wrapped in `ROBOT PRINT( ... )END` markers, for grepping robot stdout. |
| `robotPrintError(message: Any?)` | Prints `message` wrapped in `ROBOT ERROR: ...END` markers. Used internally by AlonLib for out-of-range arguments and invalid state instead of throwing. |
| `enum TelemetryLevel { Testing, Competition }` | Two telemetry verbosity presets. |
| `TelemetryLevel.transmissionIntervalMs: Int` | `25` for `Testing`, `250` for `Competition` — see `Telemetry.throttleTo`. |
| `Telemetry.throttleTo(level: TelemetryLevel)` | Sets `Telemetry.msTransmissionInterval` to `level.transmissionIntervalMs`. Call once during `init()`; the FTC SDK's default sends every `update()` call immediately, which is a common cause of loop time growing. |

### `units/`

`Units.kt` — semantic type aliases (all plain `Double`/`Int`/`Long` under the hood, for
self-documenting signatures):

| Alias | Underlying type | Meaning |
| --- | --- | --- |
| `Rotations` | `Double` | 2π radians / 360 degrees |
| `PercentOutput` | `Double` | In `[-1.0, 1.0]`, motor/servo output |
| `Volts` | `Double` | Voltage |
| `Amps` | `Int` | Current, in Amperes |
| `Seconds` | `Double` | Time |
| `Mills` | `Long` | Time, in milliseconds |

`Alliance.kt`

| Symbol | Description |
| --- | --- |
| `enum Alliance { Blue, Red }` | Match alliance color. |

`Length.kt` — a unit-safe length, immutable, always backed by meters internally:

| Symbol | Description |
| --- | --- |
| `Length.fromMeters/fromCentimeters/fromMillimeters/fromFeet/fromInches(value: Number)` | Companion factories. |
| `Length(meters, centimeters, millimeters, feet, inches)` | Constructor that sums any combination of unit-labeled arguments (all default `0`). |
| `.asMeters / .asCentimeters / .asMillimeters / .asFeet / .asInches: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double` | Arithmetic, returns a new `Length`. |
| `compareTo` | `Length` implements `Comparable<Length>` (also unlocks `<`, `>`, `..`, etc.). |
| `toString()` | `"Meters($meters)"`. |
| NaN/infinite guard | Assigning a `NaN` or infinite value logs via `robotPrintError` and clamps to `0.0` instead. |

`AngularVelocity.kt` — a unit-safe angular velocity, always backed by RPM internally:

| Symbol | Description |
| --- | --- |
| `AngularVelocity.fromRpm/fromRps/fromRadPs/fromDegPs(value: Double)` | Companion factories. |
| `AngularVelocity.fromMps(mps: Double, wheelRadius: Length)` | Build from a linear wheel speed and wheel radius. |
| `.asRpm / .asRps / .asRadPs / .asDegPs: Double` | Read out in a given unit. |
| `.asMps(wheelRadius: Length): Double` | Convert to linear speed for a wheel of the given radius. |
| `.absoluteValue: AngularVelocity` | `abs()`, preserving the type. |
| `+`, `-`, `* Double`, `/ Double`, `compareTo` | Arithmetic and ordering, same shape as `Length`. |
| NaN/infinite guard | Same behavior as `Length`. |

`Extensions.kt` — numeric literal builders and `Rotation2d`/`Pose2d` helpers built on top of
`Length`/`AngularVelocity`/this library's own geometry types:

| Symbol | Description |
| --- | --- |
| `Number.meters / .centimeters / .millimeters / .feet / .inches: Length` | e.g. `5.inches`. |
| `Number.rpm / .rps / .radPs / .degPs: AngularVelocity` | e.g. `435.rpm`. |
| `Number.degrees / .radians / .rotations: Rotation2d` | e.g. `90.degrees`. |
| `Rotation2d.absoluteValue: Rotation2d` | `abs()` on the degree value, preserving the type. |
| `Rotation2d.rotations: Double` | Degrees ÷ 360. |
| `Rotation2d.normalizedDegrees / .normalizedRadians / .normalizedRotations: Double` | Normalized into a consistent range. |
| `+`, `-`, `* Double`, `/ Double`, `..` (`rangeTo`), `compareTo` on `Rotation2d` | Arithmetic and ordering. |
| `Pose2d.xDistanceTo(other) / .yDistanceTO(other): Double` | Signed axis-aligned distance to another pose. |
| `Pose2d.distanceTo(other) / .horizontalDistanceTo(other): Double` | Euclidean distance (both are the same calculation). |
| `Pose2d.horizontalAngleTo(other): Rotation2d` | Bearing from this pose to another, via `atan`. |

`Conversions.kt` — free functions; every angle/angular-velocity one accepts any `Number`:

| Category | Functions |
| --- | --- |
| Constants | `INCHES_IN_METER`, `DECODE_FIELD_LENGTH`, `DECODE_FIELD_WIDTH` (both `Length.fromMeters(3.585719)`) |
| Angle ↔ angle | `degToRad(deg)`, `radToDeg(rad)` |
| Angular velocity ↔ angular velocity | `rpmToRps`, `rpmToRadPs`, `rpmToDegPs`, `rpsToRpm`, `rpsToRadPs`, `rpsToDegPs`, `radPsToRpm`, `radPsToRps`, `radPsToDegPs`, `degPsToRpm`, `degPsToRps`, `degPsToRadPs` |
| Angular velocity → linear velocity | `rpmToMps(rpm, wheelRadius)`, `radPsToMps(radPs, wheelRadius)`, `degPsToMps(degPs, wheelRadius)` |
| Linear velocity → angular velocity | `mpsToRpm(mps, wheelRadius)`, `mpsToRadPs(mps, wheelRadius)`, `mpsToDegPs(mps, wheelRadius)` |
| Length ↔ length | `metersToInches`, `metersToFeet`, `inchesToMeters`, `inchesToFeet`, `feetToMeters`, `feetToInches` |
| Linear speed ↔ linear speed | `mpsToMMps`, `mpsToCMps`, `mpsToKph`, `mpsToIps`, `mpsToMph`, `mmpsToMps` |
| Field-relative | `matchPoseToAlliance(position: Pose2d, alliance: Alliance): Pose2d` — mirrors a blue-alliance-relative pose onto red (`Alliance.Red` reflects `x` about the field length and adds 180° to heading); `Alliance.Blue` returns `position` unchanged. |

All `...ToMps`/`mpsTo...` conversions log via `robotPrintError` and return `0.0` if `wheelRadius`
isn't positive.

`RoadRunnerConversions.kt` — RoadRunner has its own `Pose2d`/`Rotation2d` geometry types, separate
from the ones AlonLib uses everywhere else (its own `math.geometry.Rotation2d`/`Pose2d`). These
convert between them at the boundary, so hardware wrappers stay on AlonLib's types while
RoadRunner-specific code (drive/localizer/trajectories) uses its own:

| Symbol | Description |
| --- | --- |
| `Rotation2d.toRoadRunner(): RoadRunner Rotation2d` | Via `RoadRunnerRotation2d.exp(radians)`. |
| `RoadRunner Rotation2d.toRotation2d(): Rotation2d` | Via `.log()`. |
| `Pose2d.toRoadRunner(): RoadRunner Pose2d` | |
| `RoadRunner Pose2d.toPose2d(): Pose2d` | |

### `math/`

`LinearInterpolationTable.kt`

| Symbol | Description |
| --- | --- |
| `LinearInterpolationTable(vararg points: Pair<Double, Double>)` | Builds a lookup table from `(input, output)` points (`typealias Point = Pair<Double, Double>`). |
| `.getOutputFor(input: Double): Double` | Linearly interpolates between the two table points bracketing `input`; clamps to the nearest edge segment outside the table's range instead of extrapolating unboundedly-undefined. |
| `.firsts / .seconds: DoubleArray` | All the table's input (`.first`) or output (`.second`) values, in the order given to the constructor. |

`PIDFGains.kt`

| Symbol | Description |
| --- | --- |
| `PIDFGains(kP, kI, kD, kFF, kS, KV, Ka, kIZone)` | All `Double`, all default `0.0`. Bundles a PID controller's gains plus feedforward (`kS`/`KV`/`Ka`) and an integral zone. |
| `.toString()` | `"(kP: .. ,kI: .. ,Kd: .. ,kFF: .. ,kS:.. ,kV: .. ,kA:.. )"`. |
| `PIDController.configPID(gains: PIDFGains)` | Applies `gains.kP/kI/kD` to a `PIDController`'s `p`/`i`/`d`. |

`Operations.kt`

| Symbol | Description |
| --- | --- |
| `simpleDeadband(value: Double, deadband: Double): Double` | Returns `0.0` if `abs(value) < deadband`, else `value` unchanged. Logs and returns `value` if `deadband < 0`. |
| `continuousDeadband(value: Double, deadband: Double): Double` | Like `simpleDeadband`, but remaps the surviving range continuously onto `[0, 1]`/`[-1, 0]` instead of leaving a jump at the deadband boundary — e.g. `continuousDeadband(0.5, 0.1) == 0.4444`. `deadband` must be in `[0, 1]`, `value` in `[-1, 1]`; out-of-range logs and returns `value` unchanged. |
| `clamp(value: Double, min: Double, max: Double): Double` | `value.coerceIn(min, max)`; returns `0.0` if `min > max`. |
| `mapRange(value, startMin, startMax, endMin, endMax): Double` | Linearly remaps `value` from `[startMin, startMax]` to `[endMin, endMax]`. Logs and returns `value` unchanged if either range is inverted/degenerate (`min >= max`). |
| `mapRange(value: Int, startMin: Int, startMax: Int, endMin: Int, endMax: Int): Int` | Same, for integers (delegates to the `Double` overload). |
| `median(collection: Collection<Double>) / (array: Array<Double>) / (array: DoubleArray): Double` | The statistical median. Averages the two middle elements when the size is even (a 2-element collection averages both). |

### `math/filters/movingwindowfilters/`

`MovingWindowFilter.kt` — abstract base for a finite-memory low-pass filter. **Since filters have
memory, use a separate instance per input stream.**

| Symbol | Description |
| --- | --- |
| `abstract var window: Int` | Number of samples included in the calculation. Subclasses validate it (see below). |
| `.calculate(newSample: Double): Double` | Pushes `newSample` in (dropping the oldest sample once `window` is exceeded) and returns the filter's output for the updated sample set. Call once per period. |
| `.reset(newValues: DoubleArray) / (Collection<Double>) / (Array<Double>)` | Clears all previous samples and refills from `newValues` (truncated to `window` if longer, left short if shorter). |
| `.reset(newValue: Double)` | Clears all previous samples and fills the whole window with `newValue`. |
| `.clear()` | Clears all previous samples without refilling. |

| Concrete filter | `calculation` | Notes |
| --- | --- | --- |
| `MovingAverageFilter(window: Int)` | `values.average()` | Setting `window <= 0` logs via `robotPrintError` and forces it to `0`. |
| `MovingMedianFilter(window: Int)` | `median(values)` (see `Operations.median`) | Good for rejecting occasional outliers; same `window <= 0` guard as above. |

### `hardware/`

`Data.kt` — shared enums used by hardware wrappers:

| Symbol | Description |
| --- | --- |
| `Data.Servos.Mode { CR, FULL_RANGE }` | Continuous-rotation vs. positional servo. |
| `Data.Servos.Type { Torque, Speed, SuperSpeed, AxonMax, AxonMini }` | Each entry carries `.range: Rotation2d` and `.maxSpeed: AngularVelocity` (datasheet sweep/no-load speed) — `Torque`/`Speed`/`SuperSpeed` are 300°, `AxonMax`/`AxonMini` are 350°. |

#### `HaMotor`

`hardware/motors/HaMotor.kt` — owns an SDK `DcMotorEx` directly plus its own software PIDF loop
(position/velocity control run in software and written out as voltage, not the motor
controller's own onboard PID) and software current limiting. Implements `HardwareDevice`.

```kotlin
class HaMotor(hardwareMap: HardwareMap, id: String, cpr: Number, rpm: Number, vararg followers: HaMotor)
// or, from a known GoBILDA part:
HaMotor(hardwareMap: HardwareMap, id: String, type: GoBILDA, vararg followers: HaMotor)
```

Requires a `LynxModule` named `"Control Hub"` in the hardware map (bulk-reads position/velocity from
it, and battery voltage for its voltage-based `percentOutput` conversion).

Optional `followers` mirror this motor's `percentOutput` every time it's set (directly, or via
`voltage`/`update()`) — construct each one the way you want it to run (direction, zero-power
behavior, ...) and pass it in here; they never run their own PID. This replaces the old, separate
`MotorGroup`/leader-follower classes.

| Symbol | Description |
| --- | --- |
| `.hub: LynxModule` | The `"Control Hub"` `LynxModule` this motor bulk-reads from. |
| `.motor: DcMotorEx` | The underlying SDK motor — escape hatch, not meant for normal use. |
| `.velocityController` / `.positionController: PIDFController` | The software PID loops backing `RunMode.VelocityControl`/`RunMode.PositionControl`. |
| `.feedForwardController: SimpleMotorFeedforward` | Rebuilt from `pidfGains.kS/KV/Ka` whenever `pidfGains` is set. |
| `.cachingTolerance: Double` | The minimum power delta (default `0.0001`) before `percentOutput`'s setter actually writes to `motor`. |
| `.zeroPowerBehavior: ZeroPowerBehavior` | `FLOAT` (default) or `BRAKE`; forwarded to the underlying motor and every one of `followers`. |
| `.runningDirection: Direction` | `FORWARD`/`REVERSE`; backed by `motor.direction`. |
| `.runMode: RunMode` | `RawPower` (default, `update()` is a no-op), `PositionControl`, or `VelocityControl` — selects what `setPoint`/`update()` do. |
| `.percentOutput: PercentOutput` | Direct `[-1, 1]` power. Clamped to `[minPercentOutput, maxPercentOutput]`, further scaled by `currentLimitScalar`. Refuses to move past a tripped `forwardLimit`/`reverseLimit` (logs instead). Mirrored to every one of `followers` once applied. |
| `.voltage: Double` | Get: `batteryVoltage * percentOutput`. Set: converts a target voltage to `percentOutput` given current battery voltage (floored at 1.0V to avoid divide-by-near-zero). |
| `.current: Double` | Motor current in milliamps. |
| `.currentLimit: Double` | Milliamps; `<= 0.0` disables current limiting entirely (default). |
| `.currentLimitStep: Double` | How much `currentLimitScalar` moves per `update()` call while backing off/recovering (`[0, 1]`, default `0.05`). |
| `.currentLimitScalar: Double` (read-only) | Current derating factor in `[0, 1]`; `1.0` = no derating. |
| `.forwardLimit` / `.reverseLimit: () -> Boolean` | Software limit-switch callbacks, checked by `percentOutput`'s setter only (default `{ false }`). |
| `.position: Rotation2d` | Get: current encoder position. Set: sets the PID `setPoint` (in `PositionControl` mode), clamped to `[minimumPosition, maximumPosition]`. |
| `.velocity: AngularVelocity` | Get: current encoder velocity. Set: sets the PID `setPoint` (in `VelocityControl` mode); `0.rpm` instead directly zeroes `motor.power`. Clamped to `±maxRpm`. |
| `.pidfGains: PIDFGains` | Applying this pushes `kP/kI/kD` into both PID controllers and rebuilds `feedForwardController` from `kS/KV/Ka`. |
| `.setPoint: Double` | The active controller's raw setpoint (degrees in `PositionControl`, RPM in `VelocityControl`); resets both PID controllers first if either has nonzero `i`. Clamped to the relevant min/max. |
| `.error: Double` (read-only) | Active controller's position error; `0.0` in `RawPower`. |
| `.tolerance: Double` | Forwarded to whichever PID controller is active via `setTolerance`. |
| `.inTolerance: Boolean` (read-only) | Active controller's `atSetPoint()`; always `true` in `RawPower`. |
| `.minPercentOutput` / `.maxPercentOutput: Double` | Default `-1.0`/`1.0`; each is coerced to stay on the correct side of the other. |
| `.maximumPosition` / `.minimumPosition: Rotation2d` | Default `±180°`; rejects (logs, doesn't apply) a value that would invert the min/max ordering. |
| `.stop()` | Sets `percentOutput = 0.0`, zeroes `motor.power` directly, and stops every one of `followers` too. |
| `.update()` | **Call every loop.** Runs `limitCurrent()`, then — in `VelocityControl`/`PositionControl` — computes `voltage` from the active PID controller + feedforward + `kFF * sign(error)`. No-op in `RawPower`. |
| `HardwareDevice` overrides | `getManufacturer()` (`Unknown`), `getDeviceName()` (`"HaMotor"`), `getConnectionInfo()` (`""`), `getVersion()` (`1`), `resetDeviceConfigurationForOpMode()` (stops+resets encoder, propagated to `followers`), `close()` (closes the motor, propagated to `followers`). |

#### `HaServo`

`hardware/servos/HaServo.kt` — wraps a `Servo` (force-cast to `ServoImplEx` to set a 500–2500µs PWM
range on construction). Implements `HardwareDevice`.

```kotlin
class HaServo(hardwareMap: HardwareMap, id: String, mode: Data.Servos.Mode, type: Data.Servos.Type)
```

| Symbol | Description |
| --- | --- |
| `.servo: Servo` | The underlying raw servo — escape hatch, not meant for normal use. |
| `.forwardLimit` / `.reverseLimit: () -> Boolean` | Software limit callbacks for `percentOutput` only (default `{ false }`). |
| `.maxPercentOutput` / `.minPercentOutput: Double` | Default `1.0`/`0.0`, each coerced against the other, both within `[0, 1]`. |
| `.percentOutput: Double` | Get: `servo.position`. Set: clamped to `[minPercentOutput, maxPercentOutput]`; refuses to move past a tripped limit (logs instead). |
| `.maxPosition` / `.minPosition: Rotation2d` | Soft position limits **relative to the center of the servo's physical sweep** (0° = centered, not one end) — half of `type.range` in each direction. Defaults to `±type.range / 2`. |
| `.minLimit` / `.maxLimit: Double` | Soft limits in plain degrees **from the low end of the physical sweep** (e.g. straight off a datasheet), applied on top of (not instead of) `minPosition`/`maxPosition`. Default `0.0`/`type.range` (i.e. no extra restriction). |
| `.position: Rotation2d` | `Mode.FULL_RANGE` only (`Mode.CR` logs an error and does nothing). Set: clamps to `[minPosition, maxPosition]`, converts to an absolute physical angle, clamps again to `[minLimit, maxLimit]`, then writes `servo.position` as a `[0, 1]` fraction of `type.range`. |
| `.maxVelocity` / `.minVelocity: AngularVelocity` | Default `type.maxSpeed`/`0.rpm`, each coerced within `[0, type.maxSpeed]`. |
| `.velocity: AngularVelocity` | `Mode.CR` only (`Mode.FULL_RANGE` logs an error and does nothing). Set: maps `[minVelocity, maxVelocity]` onto the servo's raw `[?, 1]` CR range. |
| `.runningDirection: HaMotor.Direction` | `FORWARD`/`REVERSE`; backed by `servo.direction` (inverted, since `HaMotor.Direction` and the FTC SDK's `Servo.Direction` disagree on sense). |
| `.stop()` | `Mode.CR`: sets `percentOutput = 0.0`. `Mode.FULL_RANGE`: no-op (a positional servo has no "stop"). |
| `HardwareDevice` overrides | `getManufacturer()` (`Unknown`), `getDeviceName()` (`"HaServo"`), `getConnectionInfo()` (`""`), `getVersion()` (`1`), `resetDeviceConfigurationForOpMode()` (no-op), `close()` (closes the underlying servo). |

#### `HaLimelight3A`

`hardware/sensors/HaLimelight3A.kt` — thin wrapper over the FTC SDK's `Limelight3A`. Implements
`HardwareDevice`.

```kotlin
class HaLimelight3A(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.isPolling: Boolean` | `limelight.isRunning`. |
| `.currentlyConnected: Boolean` | `limelight.isConnected`. |
| `.status: LLStatus` | Raw device status. |
| `.latestResult: LLResult?` | Most recent vision result. |
| `.detectedTags: List<LLResultTypes.FiducialResult>?` | `latestResult`'s AprilTag detections. |
| `.firstDetectedTag: LLResultTypes.FiducialResult?` | First entry of `detectedTags`. |
| `.firstDetectedTagId: Int?` | `firstDetectedTag?.fiducialId`. |
| `.latestPose2d: Pose2d` | `latestResult`'s MegaTag2 pose (`0` for x/y/heading if there's no result yet). |
| `.pipeLine: Int` | Set switches the active pipeline (`limelight.pipelineSwitch`). |
| `.pollRate: Double` | Get: `limelight.timeSinceLastUpdate`. Set: `limelight.setPollRateHz(Hz.toInt())`. |
| `.reloadCurrentPipeline()` | Reloads the active pipeline's config. |
| `.startPolling()` / `.pausePolling()` / `.stopPolling()` | Lifecycle control over the vision pipeline. |
| `.captureSnapshot(name: String)` | Saves a snapshot on the Limelight. |
| `.deleteAllSnapshots()` | Deletes every saved snapshot. |
| `.deleteSnapShot(name: String)` | Deletes one saved snapshot. |
| `.UpdateMegaTag2RobotHeading(yaw: Rotation2d)` | Feeds the robot's current heading in for MegaTag2 pose fusion. |
| `HardwareDevice` overrides | All forwarded straight through to the underlying `limelight`. |

#### `HaPinPoint`

`hardware/sensors/HaPinPoint.kt` — thin wrapper over goBILDA's `GoBildaPinpointDriver` odometry
computer. Implements `HardwareDevice`.

```kotlin
class HaPinPoint(hardwareMap: HardwareMap, id: String, pod: GoBildaPinpointDriver.GoBildaOdometryPods)
```

| Symbol | Description |
| --- | --- |
| `.driver: GoBildaPinpointDriver` | The underlying driver, for low-level access this wrapper doesn't expose (e.g. a RoadRunner localizer). |
| `.deviceStatus: GoBildaPinpointDriver.DeviceStatus` | Device health/status. |
| `.loopTime: Int` | The Pinpoint's own internal loop time. |
| `.frequency: Double` | The Pinpoint's own internal update frequency. |
| `.encoderXTicks` / `.encoderYTicks: Int` | Raw encoder tick counts. |
| `.position: Pose2d` | Get/set the tracked field pose (meters + degrees). |
| `.positionX` / `.positionY: Length` | Get/set individual axes, in millimeters under the hood. |
| `.heading: Rotation2d` | Get/set the tracked heading, normalized. |
| `.countedHeading: Rotation2d` (read-only) | Unnormalized heading (keeps counting past ±180°). |
| `.xVelocity` / `.yVelocity: Length` (read-only) | Per-axis linear velocity (as a `Length`, i.e. "distance per second"). |
| `.headingVelocity: AngularVelocity` (read-only) | Angular velocity of the tracked heading. |
| `.xOffset` / `.yOffset: Length` (read-only) | Currently configured pod offsets — see `setOffset`. |
| `.setEncoderDirections(xEncoderDirection, yEncoderDirection)` | Configures which way each pod's encoder counts up. |
| `.resetPoseAndIMU()` | Zeroes position to `(0,0,0)` and recalibrates the IMU. **Robot must be stationary**; takes ~0.25s. |
| `.resetIMU()` | Recalibrates just the IMU (position untouched). Same stationary/~0.25s requirement. |
| `.setOffset(xOffset: Length, yOffset: Length)` | Sets the odometry pods' positions relative to the robot's tracking point — see the in-source doc comment for sign conventions. |
| `.update()` | Pulls a fresh reading from the Pinpoint. Call once per loop before reading position/velocity. |
| `HardwareDevice` overrides | Mostly forwarded to `pinPoint`; `getManufacturer()` is hardcoded to `GoBilda`. |

### `drives/mecanumDrive/`

`HaMecanumDrive.kt` — a mecanum drivetrain subsystem extending AlonLib's own `RobotDrive`, driving
`HaMotor.percentOutput` directly (bypassing any individual `HaMotor`'s PID/current-limiting layer).

```kotlin
class HaMecanumDrive(motors: Array<HaMotor>, rightSideMultiplier: Double = -1.0)
// or, from four HaMotors (front-left, front-right, back-left, back-right), right side auto-inverted:
HaMecanumDrive(frontLeft: HaMotor, frontRight: HaMotor, backLeft: HaMotor, backRight: HaMotor)
```

| Symbol | Description |
| --- | --- |
| `.motors: Array<HaMotor>` | The four wheel motors, indexed by `MotorType` (`kFrontLeft`, `kFrontRight`, `kBackLeft`, `kBackRight`). |
| `.rightSideMultiplier: Double` | `-1.0` (inverted, default) or `1.0`. |
| `.isRightSideInverted(): Boolean` | `rightSideMultiplier == -1.0`. |
| `.setRightSideInverted(isInverted: Boolean)` | Sets `rightSideMultiplier` to `-1.0`/`1.0`. |
| `.stop()` | Stops every motor (override of `RobotDrive.stop`). |
| `.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed)` | Robot-relative drive — equivalent to `driveFieldCentric(..., gyroAngle = 0.0)`. |
| `.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, squareInputs: Boolean)` | Same, optionally squaring each input first (finer control near zero) before clamping to `[-1, 1]`. |
| `.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle)` | Field-relative drive: rotates the strafe/forward vector by `-gyroAngle`, computes per-wheel mecanum speeds, normalizes, adds turn, normalizes again, and drives. |
| `.driveFieldCentric(xSpeed, ySpeed, turnSpeed, gyroAngle, squareInputs: Boolean)` | Same, with optional input squaring first. |
| `.driveWithMotorPowers(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed)` | Drives each wheel directly (right-side speeds scaled by `rightSideMultiplier`), all scaled by `RobotDrive.maxOutput`. |

### `commands/`

`Extentions.kt` — infix/extension sugar over this library's own `Command`:

| Symbol | Description |
| --- | --- |
| `Command until condition: () -> Boolean` | `this.until(condition)`. |
| `Command andThen next: Command` / `Command andThen next: () -> Command` | `this.andThen(next)`, or `this.andThen(next())` for the lazy form. |
| `Command finallyDo end: (interrupted: Boolean) -> Unit` | `this.finallyDo(end)`. |
| `Command finallyDo command: Command` | Schedules `command` when this command ends, regardless of interruption. |
| `Command alongWith parallel: Command` | `this.alongWith(parallel)`. |
| `Command raceWith parallel: Command` | `this.raceWith(parallel)`. |
| `Command withTimeout seconds: Double` | `this.withTimeout(seconds)`. |
| `Command withName commandName: String` | `this.withName(commandName)` — for multi-subsystem commands; see `SubsystemBase.withName` for single-subsystem ones. |
| `withName(commandName: String, commandSupplier: () -> CommandBase): Command` | Builds a command from the supplier and names it `commandName`. |
| `SubsystemBase.withName(commandName: String, commandSupplier: () -> CommandBase): Command` | Same, but names it `"$commandName : ${this.name}"` (appends the owning subsystem's name). |

`Factories.kt`

| Symbol | Description |
| --- | --- |
| `wait(duration: Mills): WaitCommand` | A command that finishes after `duration` milliseconds. |
| `waitUntil(until: () -> Boolean): WaitUntilCommand` | A command that finishes once `until()` returns `true`. |
| `instantCommand(toRun: () -> Unit): InstantCommand` | Runs `toRun` once and finishes immediately. |
| `(() -> Unit).asInstantCommand: Command` | Same as `instantCommand`, as an extension property. **Requires no subsystems** — don't use it for an action that needs to claim one. |

`RoadRunnerCommands.kt` — bridges a RoadRunner `Action` (e.g. a trajectory from
`MecanumDrive.actionBuilder(...).build()`) into a `Command`, so it can be scheduled as a
default/triggered command or combined with `SequentialCommandGroup` etc., instead of only being
runnable via `Actions.runBlocking` in a plain `LinearOpMode`:

| Symbol | Description |
| --- | --- |
| `class ActionCommand(action: Action, vararg requirements: Subsystem) : CommandBase` | Each `execute()` runs the action once (every loop, so trajectory following stays accurate) and sends the resulting `TelemetryPacket` to FTC Dashboard. `isFinished()` becomes `true` once `action.run(...)` returns `false`. |
| `Action.asCommand(vararg requirements: Subsystem): Command` | Shorthand for `ActionCommand(this, *requirements)`. |

## Running on a desktop emulator instead of a robot

`alonlib-emulator` lets your OpModes — real, unmodified `LinearOpMode`/`OpMode`/`CommandOpMode`
subclasses, exactly as they'll run on the robot — run against simulated hardware on your desktop
via [ftc-control-hub-emulator](https://github.com/alonHamb/ftc-control-hub-emulator), instead of a
physical REV Control/Expansion Hub. It backs `hardwareMap.get(DcMotorEx::class.java, ...)`,
`hardwareMap.get(Servo::class.java, ...)`, and `hardwareMap.get(LynxModule::class.java, "Control
Hub")` with simulated motor/servo dynamics and a simulated battery, and drives the OpMode lifecycle
(`init`/`start`/`loop`/`stop`) the same way the Driver Station does. All 15 of its own regression
tests (hardware-map wiring, `LynxModule` bulk data/voltage, `HaMotor`/`HaServo` end-to-end, the
`OpMode` lifecycle harness, `HaServo` position-range edge cases, and `EmulatorAutoLauncher`'s config
discovery/OpMode scanning/drive-wheel guessing) pass against the real FTC SDK classes at runtime —
see `alonlib-emulator/src/test`.

**Only ever add this to your TeamCode module's `testImplementation`, never `implementation`** — it
pulls in Mockito and ftc-control-hub-emulator's Swing UI and JNA-based gamepad reading, none of
which belong in the APK that ships to the robot.

### Trying the emulator window in this repo

`:demo` is a small runnable module (plain Kotlin/JVM, kept out of the published JitPack artifacts
so consumers never pull in a Swing entrypoint) that opens the exact same `RunnerShellApp` window
[ftc-control-hub-emulator](https://github.com/alonHamb/ftc-control-hub-emulator)'s own `:demo`
module does — field view, port monitor, telemetry, and gamepad status — wired to a simulated
mecanum drivetrain and one servo. It's the fastest way to see the window working before wiring
`EmulatedRobot` into your own project:

```bash
./gradlew :demo:run
```

Click **Init**, then **Start**, then drive with gamepad1 (left stick to translate, right stick X to
rotate, A/B to open/close the claw) or the keyboard if nothing's plugged in. See
[`demo/src/main/kotlin/.../DemoMain.kt`](demo/src/main/kotlin/org/firstinspires/ftc/teamcode/alonlib/emulator/demo/DemoMain.kt).
`EmulatedRobot.launch()` (below) drives this same window against a real OpMode's hardware map
instead of the hardcoded devices this demo uses.

### Setup

`TeamCode/build.gradle`:

```groovy
android {
    // HardwareMap.get(...) touches a BNO055-vs-BHI260 IMU check that otherwise crashes off-device.
    testOptions {
        unitTests {
            returnDefaultValues = true
        }
    }
}

dependencies {
    // Same tag as AlonLib itself -- both modules are published together.
    testImplementation 'com.github.alonHamb.AlonLib:alonlib-emulator:v11.2.1'
}
```

(JitPack is already a repository if you're using AlonLib itself — see [Installing](#installing),
above.)

### Zero-code: `EmulatorAutoLauncher`

You don't need to write anything — no `EmulatorMain.kt`, no `RobotMap` mirroring, no OpMode
registration. Once the dependency above is added, `EmulatorAutoLauncher` (shipped inside
`alonlib-emulator`, so it's already sitting on your test classpath) does two things automatically,
the same way the real Driver Station does:

1. **Finds your hardware config XML itself.** It looks under `src/main/res/xml/` for the one file
   that declares a `<LynxModule>` — the exact file REV Hardware Client wrote and your project
   already uploads to the Control Hub — and builds simulated hardware straight from it via
   `emulator.config.parseRobotConfigXml`/`buildSimulatedRobot`. Add a device to your config and it
   shows up in the emulator's port monitor next time you launch, with nothing to keep in sync.
2. **Finds your OpModes itself.** It classpath-scans for every `@TeleOp`/`@Autonomous`-annotated
   `OpMode` (skipping `@Disabled` ones), the same annotations the real SDK uses to build the Driver
   Station's OpMode list, and populates the emulator's dropdown from them.

It also makes a best-effort guess at your four mecanum drive motors by name (`frontLeft`,
`front_left_motor`, `left front motor`, ... — `front`/`back`/`rear` × `left`/`right`, order- and
separator-insensitive) so the field-pose view moves; if it can't find all four unambiguously, the
emulator still runs, just without a moving field view.

**Run it:** open `EmulatorAutoLauncher` in your IDE (Android Studio/IntelliJ — `alonlib-emulator`
ships with `withSourcesJar()`, so this works with nothing of your own on disk) and click the gutter
arrow next to `launch()`.

**Don't run it via `./gradlew testDebugUnitTest`** (CLI or an IDE run configuration delegated to
Gradle) — the Android Gradle Plugin forces `-Djava.awt.headless=true` on unit test JVMs, which makes
the emulator window silently never appear; the task just hangs until you kill it, with no error. If
you want a CLI-runnable command instead of an IDE click, add a plain `JavaExec` task (not subject to
that Android-unit-test default) to your `TeamCode/build.gradle`:

```groovy
tasks.register("runEmulator", JavaExec) {
    group = "verification"
    dependsOn("assembleDebugUnitTest") // compiles test sources without running them
    classpath = tasks.named("testDebugUnitTest", Test).get().classpath
    mainClass = "org.firstinspires.ftc.teamcode.alonlib.emulator.EmulatorAutoLauncherKt"
}
```

```bash
./gradlew :TeamCode:runEmulator
```

(Swap `Debug` for your build type/variant name in both places if you're not building the `debug`
variant.)

If your project doesn't fit the zero-code path — multiple hardware config files, OpModes you don't
want auto-discovered, a non-mecanum drivetrain — construct `EmulatedRobot` yourself instead; see the
worked example below.

### API reference — `alonlib-emulator`

| Symbol | Description |
| --- | --- |
| `EmulatorAutoLauncher().launch()` | The zero-code entry point above. Also available as a plain `fun main()` in the same file, for the `JavaExec` task above. |
| `EmulatedHub(hub: HubId, motors: Map<Int, String> = emptyMap(), servos: Map<Int, String> = emptyMap())` | One physical hub's worth of simulated devices, keyed by REV port index — matching how you'd describe a real robot's wiring. `.motors`/`.servos` expose the underlying `SimMotor`/`SimServo`s (for advancing sim time in tests, e.g. `.update(dt)`); `.devices` lists all of them. Only needed if you're wiring `EmulatedRobot` by hand instead of using `EmulatorAutoLauncher`. |
| `buildEmulatedHardwareMap(controlHub: EmulatedHub, expansionHub: EmulatedHub? = null, batteryVoltage: () -> Double): HardwareMap` | Builds a real `HardwareMap` pre-populated with each hand-declared hub's devices, so `hardwareMap.get(DcMotorEx::class.java/Servo::class.java/LynxModule::class.java, ...)` all work exactly as they would against real hardware. |
| `buildEmulatedHardwareMap(simulatedRobot: emulator.config.SimulatedRobot, batteryVoltage: () -> Double): HardwareMap` | Same, but built straight from a `SimulatedRobot` (i.e. `emulator.config.buildSimulatedRobot(parseRobotConfigXml(...))`) instead of hand-declared hubs — what `EmulatorAutoLauncher` uses under the hood. |
| `EmulatedRobot(controlHub: EmulatedHub, expansionHub: EmulatedHub? = null, driveWheels: DriveWheels? = null)` | Ties one or two hand-declared `EmulatedHub`s to the emulator UI and drives whichever OpMode is selected through the real OpMode lifecycle. `.hardwareMap` is the resulting fake `HardwareMap`. `DriveWheels(frontLeft, frontRight, backLeft, backRight)` is optional and only powers the emulator's live field-pose display. |
| `EmulatedRobot(simulatedRobot: emulator.config.SimulatedRobot, driveWheels: DriveWheels? = null)` | Same, but built straight from a `SimulatedRobot` — what `EmulatorAutoLauncher` uses under the hood. |
| `EmulatedRobot.launch(title: String, opModes: Map<String, () -> OpMode>)` | Blocks the calling thread, showing the emulator window, until it's closed. Each map entry is a name shown in the OpMode dropdown → a factory for a fresh instance (matching how the real SDK constructs a new instance on every Init). Same headless-JVM caveat as `EmulatorAutoLauncher` above applies here too. |
| `EmuDcMotorEx(sim: SimMotor) : DcMotorEx` | A `DcMotorEx` backed by a simulated motor — real code that talks to `DcMotor`/`DcMotorEx` directly, or via `HaMotor` (which wraps one), runs unmodified against simulated dynamics. PID/current-alert configuration is accepted but not modeled, since `HaMotor` runs its own software PIDF loop and writes plain power/voltage. |
| `emulatedServo(controller: EmuServoController, port: Int): ServoImplEx` | Builds a genuine `ServoImplEx` for one hub port, needed because `HaServo` unconditionally force-casts `Servo` to `ServoImplEx`. |
| `EmuServoController(portsToSims: Map<Int, SimServo>) : ServoControllerEx` | Backs a hub's worth of simulated servos — the `ServoControllerEx` a real `ServoImplEx` delegates every operation to. |
| `emulatedLynxModule(motorsByPort: Map<Int, SimMotor>, batteryVoltage: () -> Double): LynxModule` | A Mockito-backed `LynxModule` whose bulk-read motor data and input voltage come from simulated motors/battery instead of a real REV hub over USB — `LynxModule` has no way to be constructed directly. |
| `EmuTelemetry() : Telemetry` | A full `Telemetry` implementation that renders into a plain `snapshot(): List<String>` instead of transmitting to a driver station, for the emulator's telemetry panel. |
| `OpModeHarness(opMode: OpMode)` | Drives an `OpMode` (or `LinearOpMode`) through the same lifecycle hooks the real SDK's `OpModeManagerImpl` uses. `.init(hardwareMap)` wires hardware/telemetry/gamepads and starts the OpMode thread; `.start()` transitions Init → Run; `.tick(gamepad1, gamepad2)` pushes fresh gamepad state and runs one event-loop iteration; `.stop()` requests a stop and blocks until the OpMode thread exits; `.crash: Throwable?` surfaces any exception thrown by user code (`null` if none). `.telemetry: EmuTelemetry` is the telemetry instance wired into the OpMode. |

### Advanced: wiring `EmulatedRobot` by hand

Skip this if `EmulatorAutoLauncher` above already covers your project. This is for when it doesn't
— multiple hardware config files, OpModes you don't want auto-discovered, a non-mecanum drivetrain.

Put this in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/EmulatorMain.kt` (a local unit
test, run manually from your IDE's gutter/Run button — same headless-JVM caveat as
`EmulatorAutoLauncher` above, so it's not meant to run as part of `./gradlew test`). This mirrors a
`RobotMap` like:

```kotlin
object RobotMap {
    object Drive {
        const val FRONT_LEFT_MOTOR_ID = "front left motor"   // port 0 on the control hub
        const val FRONT_RIGHT_MOTOR_ID = "front right motor" // port 1 on the control hub
        const val BACK_LEFT_MOTOR_ID = "back left motor"     // port 2 on the control hub
        const val BACK_RIGHT_MOTOR_ID = "back right motor"   // port 3 on the control hub
    }
    object Shooter {
        const val HOOD_SERVO_ID = "hood servo" // servo port 0 on the control hub
    }
}
```

```kotlin
package org.firstinspires.ftc.teamcode

import emulator.hardware.HubId
import org.firstinspires.ftc.teamcode.alonlib.emulator.EmulatedHub
import org.firstinspires.ftc.teamcode.alonlib.emulator.EmulatedRobot
import org.firstinspires.ftc.teamcode.opmodes.teleop.BlueMainTeleop
import org.junit.Test

class EmulatorMain {
    @Test
    fun launch() {
        val controlHub = EmulatedHub(
            HubId.CONTROL,
            motors = mapOf(
                0 to RobotMap.Drive.FRONT_LEFT_MOTOR_ID,
                1 to RobotMap.Drive.FRONT_RIGHT_MOTOR_ID,
                2 to RobotMap.Drive.BACK_LEFT_MOTOR_ID,
                3 to RobotMap.Drive.BACK_RIGHT_MOTOR_ID
            ),
            servos = mapOf(0 to RobotMap.Shooter.HOOD_SERVO_ID)
        )

        EmulatedRobot(
            controlHub,
            driveWheels = EmulatedRobot.DriveWheels(
                frontLeft = controlHub.motors.getValue(0),
                frontRight = controlHub.motors.getValue(1),
                backLeft = controlHub.motors.getValue(2),
                backRight = controlHub.motors.getValue(3)
            )
        ).launch(
            title = "Decode-Robot Emulator",
            opModes = mapOf("Blue Main Teleop" to { BlueMainTeleop() })
        )
    }
}
```

`EmulatedHub`'s port numbers should match your real robot's wiring (or don't -- they only need to
be internally consistent, since nothing physical is plugged in). List every OpMode you want to pick
from in the emulator's dropdown in the `opModes` map; each entry's factory is called fresh on every
Init, matching how the real SDK constructs a new OpMode instance every time too.

### Writing a JUnit test against the emulator directly

You don't need the UI to exercise real OpMode/hardware-wrapper code in a fast, headless test — drive
`buildEmulatedHardwareMap`/`OpModeHarness` directly, the way `alonlib-emulator`'s own test suite
does:

```kotlin
@Test
fun `drivetrain moves forward when commanded`() {
    val controlHub = EmulatedHub(HubId.CONTROL, motors = mapOf(0 to "front left motor"))
    val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 } // simulated battery voltage

    val motor = HaMotor(hardwareMap, "front left motor", HaMotor.GoBILDA.RPM_435)
    motor.percentOutput = 0.5
    motor.update()
    controlHub.motors.getValue(0).update(0.5) // advance simulated dynamics by 0.5s

    assertTrue(motor.position.degrees != 0.0)
}
```

### Known limitations

- **Anything that reaches `AppUtil`/a real Android `Context`** — `FtcDashboard.getInstance()`,
  vision pipelines, most sensor drivers other than `DcMotorEx`/`Servo`/`LynxModule` — isn't
  emulated and will throw or crash, since there's no real Android runtime underneath. Guard those
  calls (e.g. behind a flag) if your OpMode uses them, the way `BlueMainTeleop`/`RedMainTeleop` in a
  Decode-Robot-shaped project need to for `FtcDashboard.getInstance()`.
- I2C sensors (`HaLimelight3A`, `HaPinPoint`, IMUs, ...) aren't backed by anything — only
  `DcMotorEx`/`Servo`/`LynxModule` bulk data and input voltage are simulated.
- `EmulatedHardwareMapImpl` reimplements `HardwareMap.get`/`tryGet` rather than inheriting them
  (see its doc comment for why) and `OpModeHarness.init` reimplements `internalInit()`'s thread
  spawn for the same class of reason (both touch Android-only code with no desktop implementation
  even when merely *called*, regardless of what's registered in the map). Everything else on
  `HardwareMap`/`OpMode` is the SDK's own, real implementation.
- Same physics simplifications as ftc-control-hub-emulator itself: no IMU noise/drift, no wheel
  slip, ~20 Hz refresh instead of a real Driver Station's ~10-20 ms loop.
- `EmulatorAutoLauncher` requires exactly one `res/xml/*.xml` declaring a `<LynxModule>` (errors
  clearly if it finds zero or several) and guesses drive wheels by name — see
  [Advanced: wiring `EmulatedRobot` by hand](#advanced-wiring-emulatedrobot-by-hand) if either
  doesn't fit your project.

## Building locally

```bash
./gradlew :alonlib:assembleRelease :alonlib-emulator:assembleRelease
```

Run the emulator module's own regression tests (headless, no window):

```bash
./gradlew :alonlib-emulator:test
```
