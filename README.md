# AlonLib

A Kotlin utility library for FTC (*FIRST* Tech Challenge) robot code: unit-safe wrappers for
motors/servos/sensors, a full unit-safe measurement system (length, velocity, acceleration, mass,
force, torque, voltage, current, time, percentage, angle), a WPILib-derived math suite (PID/
feedforward control, state-space control, Kalman-filter-family pose estimation, geometry, spline/
trajectory generation, kinematics/odometry for differential/mecanum/swerve drivetrains), a full
command-based framework, several drivebase subsystems (mecanum, differential, H-drive, coaxial
swerve), a pure-pursuit path follower, edge-detected gamepad input, and a desktop hardware emulator
so OpModes can be run and debugged without a robot.

## Table of contents

- [Installing](#installing)
- [Requirements](#requirements)
- [Structure](#structure)
- [Quick start](#quick-start)
- [API reference](#api-reference)
  - [Root — `alonlib`](#root--alonlib)
  - [`units/`](#units)
  - [`math/` and `math/control/`](#math-and-mathcontrol)
  - [`math/estimator/`, `math/filter/`, `math/filters/movingwindowfilters/`](#mathestimator-mathfilter-mathfiltersmovingwindowfilters)
  - [`math/geometry/` and `math/interpolation/`](#mathgeometry-and-mathinterpolation)
  - [`math/kinematics/`](#mathkinematics)
  - [`math/spline/`](#mathspline)
  - [`math/system/`](#mathsystem)
  - [`math/trajectory/` and `math/trajectory/constraint/`](#mathtrajectory-and-mathtrajectoryconstraint)
  - [`hardware/`](#hardware)
  - [`drives/`](#drives)
  - [`gamepad/`](#gamepad)
  - [`p2p/`](#p2p)
  - [`purepursuit/`](#purepursuit)
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
    implementation 'com.github.alonHamb:AlonLib:<version>'
}
```

Replace `<version>` with the [latest release tag](https://github.com/alonHamb/AlonLib/tags) (recommended),
a commit hash, or `<branch>-SNAPSHOT` to track a branch directly.

## Requirements

The FTC SDK (`org.firstinspires.ftc:*`, v11.1.0), RoadRunner (`com.acmerobotics.roadrunner:*`
core/actions v1.0.1, `ftc` v0.1.25), and FTC Dashboard (v0.5.1) are bundled as `api` dependencies,
so they come along transitively — you don't need to declare any of them yourself just to use
AlonLib's classes. EJML (`org.ejml:ejml-simple`, backing the `math/system` state-space/matrix code)
is bundled as `implementation`, so it doesn't leak onto your classpath unless you touch that
package. AlonLib's command framework, geometry types, and PID/feedforward/state-space controllers
are its own reimplementation (no `org.solverslib:core` dependency).
If your project also depends on `FtcRobotController` directly (the usual FTC project setup, needed
to actually build/run the robot controller app), Gradle will de-duplicate the shared FTC SDK version
automatically as long as the versions match.

## Structure

Everything lives under `alonlib/src/main/java/org/firstinspires/ftc/teamcode/alonlib`:

- `Logging.kt` — print helpers and telemetry throttling
- `units/` — unit-safe wrapper classes (`Length`, `LinearVelocity`, `LinearAcceleration`,
  `AngularVelocity`, `AngularAcceleration`, `Mass`, `Force`, `Torque`, `Voltage`, `Current`, `Time`,
  `Percentage`), numeric-literal builders, unit conversions, `Alliance`
- `math/` — PID/feedforward gains and controllers (`math/control/`), Kalman-filter-family pose
  estimators (`math/estimator/`), signal filters/debouncers (`math/filter/`,
  `math/filters/movingwindowfilters/`), geometry types (`math/geometry/`), generic interpolation
  (`math/interpolation/`), drivetrain kinematics/odometry (`math/kinematics/`), spline generation
  (`math/spline/`), state-space control plumbing (`math/system/`), and trajectory generation
  (`math/trajectory/`)
- `hardware/` — `HaMotor`, `HaServo`, `HaCRServo`, and ~20 sensor wrappers (`HaIMU`, `HaColorSensor`,
  `HaDistanceSensor`, `HaLimelight3A`, `HaPinPoint`, `HaOTOS`, `HaOctoQuad`, `HaHuskyLens`, ...)
- `drives/` — `HaMecanumDrive`, `MecanumDrive`, `DifferentialDrive`, `HDrive`, and a coaxial swerve
  drivetrain (`drives/swerve/coaxial/`)
- `gamepad/` — `GamepadEx` and edge-detected button/trigger readers
- `p2p/` — `P2PController`, a point-to-point field-centric drive controller
- `purepursuit/` — a pure-pursuit path follower (`Path`, `Waypoint`, ...)
- `commands/` — a full command-based framework (`Command`, `CommandScheduler`, `Subsystem`,
  composite commands, trajectory-following commands, button bindings), extensions, factories, and a
  RoadRunner `Action` → `Command` bridge

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
        val frontLeft = HaMotor(hardwareMap, "front left motor", Data.Motors.GoBILDA.RPM_435)
        val frontRight = HaMotor(hardwareMap, "front right motor", Data.Motors.GoBILDA.RPM_435)
        val backLeft = HaMotor(hardwareMap, "back left motor", Data.Motors.GoBILDA.RPM_435)
        val backRight = HaMotor(hardwareMap, "back right motor", Data.Motors.GoBILDA.RPM_435)
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
  (`POSITION_CONTROL`/`VELOCITY_CONTROL`) or current limiting to actually take effect — see
  [`HaMotor`](#hamotor).
- `HaMecanumDrive` built from four `HaMotor`s drives their `percentOutput` directly, bypassing each
  `HaMotor`'s own PID/current-limiting layer — use it for simple stick-driven teleop, and drive the
  individual `HaMotor`s yourself (`percentOutput`, `velocity`, or `position`) when you need
  closed-loop control per wheel.
- See [`units/`](#units) for why `30.0.degrees` is a `Rotation2d`, not a raw `Double`.

## API reference

Every public class, function, and property in `alonlib`, grouped by package/file. Types not
exhaustively covered here — this library's own `Rotation2d`/`Pose2d`/`Command` are still documented
below in their own sections; the FTC SDK's `HardwareMap`/`Telemetry` and RoadRunner's `Action` are
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

There is no `Units.kt` in this package anymore (its old `Rotations`/`PercentOutput`/`Volts`/`Amps`/
`Seconds`/`Mills` type aliases have been replaced by the full unit-safe wrapper classes below).
Every wrapper class in this package (`Length`, `LinearVelocity`, `LinearAcceleration`,
`AngularVelocity`, `AngularAcceleration`, `Mass`, `Force`, `Torque`, `Voltage`, `Current`, `Time`,
`Percentage`) follows the same shape unless noted otherwise: a `private` backing `Double` field in
the class's base unit, a primary constructor `ClassName(value: Number, unit: ClassName.Unit)`, a
nested `enum class Unit`, `fromX(value: Number)` companion factories for every unit, `asX` read-out
properties, `Comparable<ClassName>` (so `<`, `>`, etc. all work), a `toString()` that prints the
base-unit value, and arithmetic operators `+`, `-`, `* Double`, `/ Double`, `unaryMinus()` that all
stay in the base unit and construct a new instance via the `fromX` factory. Assigning the backing
field a `NaN` or infinite value logs via `robotPrintError` (`"<ClassName> is NaN."` /
`"<ClassName> is infinite."`) and clamps it to `0.0` instead of throwing.

`Alliance.kt`

| Symbol | Description |
| --- | --- |
| `enum class Alliance { Blue, Red }` | Match alliance color. |

`Length.kt` — a unit-safe length, always backed by meters internally. Deviates from the shared
shape described above: no `unaryMinus`, and `times`/`div` take another `Length` (not a `Double`):

| Symbol | Description |
| --- | --- |
| `Length(length: Number, lengthUnit: Length.Unit)` | Primary constructor. |
| `enum Length.Unit { Meters, Centimeters, Millimeters, Feet, Inches }` | |
| `Length.fromMeters/fromCentimeters/fromMillimeters/fromFeet/fromInches(value: Number)` | Companion factories. |
| `.asMeters / .asCentimeters / .asMillimeters / .asFeet / .asInches: Double` | Read out in a given unit. |
| `+`, `-` | Arithmetic, returns a new `Length` in meters. |
| `* (other: Length)`, `/ (other: Length)` | Unlike every other class in this package, `times`/`div` take a `Length` operand, not a `Double` — `meters * other.meters` / `meters / other.meters`, wrapped back into a `Length`. There is no scalar `* Double`/`/ Double` overload and no `unaryMinus`. |
| `compareTo` | Orders by meters. |
| `toString()` | `"Meters($meters)"`. |
| NaN/infinite guard | Logs via `robotPrintError` and clamps to `0.0`, per the shared behavior above. |

`LinearVelocity.kt` — a unit-safe linear (straight-line) velocity, always backed by meters per
second internally:

| Symbol | Description |
| --- | --- |
| `LinearVelocity(velocity: Number, velocityUnit: LinearVelocity.Unit)` | Primary constructor. |
| `enum LinearVelocity.Unit { MetersPerSecond, FeetPerSecond, InchesPerSecond, CentimetersPerSecond, MillimetersPerSecond, KilometersPerHour, MilesPerHour }` | |
| `LinearVelocity.fromMetersPerSecond/fromFeetPerSecond/fromInchesPerSecond/fromCentimetersPerSecond/fromMillimetersPerSecond/fromKilometersPerHour/fromMilesPerHour(value: Number)` | Companion factories. |
| `.asMetersPerSecond / .asFeetPerSecond / .asInchesPerSecond / .asCentimetersPerSecond / .asMillimetersPerSecond / .asKilometersPerHour / .asMilesPerHour: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on meters per second. |
| `toString()` | `"MetersPerSecond($metersPerSecond)"`. |
| NaN/infinite guard | Shared behavior. |

`LinearAcceleration.kt` — a unit-safe linear acceleration, always backed by meters per second
squared internally:

| Symbol | Description |
| --- | --- |
| `LinearAcceleration(acceleration: Number, accelerationUnit: LinearAcceleration.Unit)` | Primary constructor. |
| `enum LinearAcceleration.Unit { MetersPerSecondSquared, FeetPerSecondSquared, InchesPerSecondSquared, Gs }` | |
| `LinearAcceleration.STANDARD_GRAVITY: Double` | `const val`, `9.80665` (meters per second squared) — also reused by `Force.kt` and `Torque.kt` for their gravity-based unit conversions. |
| `LinearAcceleration.fromMetersPerSecondSquared/fromFeetPerSecondSquared/fromInchesPerSecondSquared/fromGs(value: Number)` | Companion factories. |
| `.asMetersPerSecondSquared / .asFeetPerSecondSquared / .asInchesPerSecondSquared / .asGs: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on meters per second squared. |
| `toString()` | `"MetersPerSecondSquared($metersPerSecondSquared)"`. |
| NaN/infinite guard | Shared behavior. |

`AngularVelocity.kt` — a unit-safe angular velocity, always backed by RPM internally. Deviates
from the shared shape: the backing field (`rpm`) is a **public** settable property (not a private
one behind `asRpm`), it additionally has `absoluteValue`, and it has no `unaryMinus`:

| Symbol | Description |
| --- | --- |
| `AngularVelocity(velocity: Double, velocityUnit: AngularVelocity.Unit)` | Primary constructor. |
| `enum AngularVelocity.Unit { Rpm, Rps, RadPs, DegPs }` | |
| `var rpm: Double` | The backing property itself, public and directly settable (unlike other classes in this package, whose base-unit field is private). Setting it to `NaN`/infinite logs via `robotPrintError` and clamps to `0.0`. |
| `.asRpm / .asRps / .asRadPs / .asDegPs: Double` | Read out in a given unit. |
| `.asMps(wheelRadius: Length): Double` | Convert to linear speed for a wheel of the given radius, via `rpmToMps`. |
| `.absoluteValue: AngularVelocity` | `abs()` of the RPM value, preserving the type. |
| `+`, `-`, `* Double`, `/ Double`, `compareTo` | Shared shape, operating on RPM. No `unaryMinus`. |
| `toString()` | `"RPM($rpm)"`. |
| `AngularVelocity.fromRpm/fromRps/fromRadPs/fromDegPs(value: Double)` | Companion factories. Note: these take `Double`, not `Number`, unlike every other `fromX` factory in this package. |
| `AngularVelocity.fromMps(mps: Double, wheelRadius: Length): AngularVelocity` | Build from a linear wheel speed and wheel radius, via `mpsToRpm`. |
| NaN/infinite guard | Shared behavior (see `rpm` above). |

`AngularAcceleration.kt` — a unit-safe angular acceleration, always backed by radians per second
squared internally:

| Symbol | Description |
| --- | --- |
| `AngularAcceleration(acceleration: Number, accelerationUnit: AngularAcceleration.Unit)` | Primary constructor. |
| `enum AngularAcceleration.Unit { RadiansPerSecondSquared, DegreesPerSecondSquared, RotationsPerSecondSquared, RpmPerSecond }` | `RpmPerSecond` is a common motor-spec unit. |
| `AngularAcceleration.fromRadiansPerSecondSquared/fromDegreesPerSecondSquared/fromRotationsPerSecondSquared/fromRpmPerSecond(value: Number)` | Companion factories. |
| `.asRadiansPerSecondSquared / .asDegreesPerSecondSquared / .asRotationsPerSecondSquared / .asRpmPerSecond: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on radians per second squared. |
| `toString()` | `"RadiansPerSecondSquared($radiansPerSecondSquared)"`. |
| NaN/infinite guard | Shared behavior. |

`Mass.kt` — a unit-safe mass, always backed by kilograms internally:

| Symbol | Description |
| --- | --- |
| `Mass(mass: Number, massUnit: Mass.Unit)` | Primary constructor. |
| `enum Mass.Unit { Kilograms, Grams, Pounds, Ounces }` | |
| `Mass.KILOGRAMS_PER_POUND: Double` | `const val`, `0.45359237` — also reused by `Force.kt` and `Torque.kt` for their pound-based unit conversions. |
| `Mass.fromKilograms/fromGrams/fromPounds/fromOunces(value: Number)` | Companion factories. |
| `.asKilograms / .asGrams / .asPounds / .asOunces: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on kilograms. |
| `toString()` | `"Kilograms($kilograms)"`. |
| NaN/infinite guard | Shared behavior. |

`Force.kt` — a unit-safe force, always backed by newtons internally. Pounds-force and
kilograms-force are converted through `Mass.KILOGRAMS_PER_POUND` and
`LinearAcceleration.STANDARD_GRAVITY`:

| Symbol | Description |
| --- | --- |
| `Force(force: Number, forceUnit: Force.Unit)` | Primary constructor. |
| `enum Force.Unit { Newtons, PoundsForce, KilogramsForce, Dynes }` | |
| `Force.fromNewtons/fromPoundsForce/fromKilogramsForce/fromDynes(value: Number)` | Companion factories. |
| `.asNewtons / .asPoundsForce / .asKilogramsForce / .asDynes: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on newtons. |
| `toString()` | `"Newtons($newtons)"`. |
| NaN/infinite guard | Shared behavior. |

`Torque.kt` — a unit-safe torque, always backed by newton-meters internally. Ounce-inches and
kilogram-centimeters are common servo-spec units:

| Symbol | Description |
| --- | --- |
| `Torque(torque: Number, torqueUnit: Torque.Unit)` | Primary constructor. |
| `enum Torque.Unit { NewtonMeters, PoundFeet, PoundInches, OunceInches, KilogramCentimeters }` | |
| `Torque.NEWTON_METERS_PER_POUND_FOOT: Double` | `const val`, `Mass.KILOGRAMS_PER_POUND * LinearAcceleration.STANDARD_GRAVITY * 0.3048`. Used to derive `PoundFeet`/`PoundInches`/`OunceInches`; `KilogramCentimeters` instead uses `LinearAcceleration.STANDARD_GRAVITY` directly. |
| `Torque.fromNewtonMeters/fromPoundFeet/fromPoundInches/fromOunceInches/fromKilogramCentimeters(value: Number)` | Companion factories. |
| `.asNewtonMeters / .asPoundFeet / .asPoundInches / .asOunceInches / .asKilogramCentimeters: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on newton-meters. |
| `toString()` | `"NewtonMeters($newtonMeters)"`. |
| NaN/infinite guard | Shared behavior. |

`Voltage.kt` — a unit-safe voltage, always backed by volts internally:

| Symbol | Description |
| --- | --- |
| `Voltage(voltage: Number, voltageUnit: Voltage.Unit)` | Primary constructor. |
| `enum Voltage.Unit { Volts, Millivolts, Microvolts, Kilovolts }` | |
| `Voltage.fromVolts/fromMillivolts/fromMicrovolts/fromKilovolts(value: Number)` | Companion factories. |
| `.asVolts / .asMillivolts / .asMicrovolts / .asKilovolts: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on volts. |
| `toString()` | `"Volts($volts)"`. |
| NaN/infinite guard | Shared behavior. |

`Current.kt` — a unit-safe electrical current, always backed by amps internally:

| Symbol | Description |
| --- | --- |
| `Current(current: Number, currentUnit: Current.Unit)` | Primary constructor. |
| `enum Current.Unit { Amps, Milliamps, Microamps, Kiloamps }` | |
| `Current.fromAmps/fromMilliamps/fromMicroamps/fromKiloamps(value: Number)` | Companion factories. |
| `.asAmps / .asMilliamps / .asMicroamps / .asKiloamps: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on amps. |
| `toString()` | `"Amps($amps)"`. |
| NaN/infinite guard | Shared behavior. |

`Time.kt` — a unit-safe duration, always backed by seconds internally:

| Symbol | Description |
| --- | --- |
| `Time(time: Number, timeUnit: Time.Unit)` | Primary constructor. |
| `enum Time.Unit { Seconds, Milliseconds, Microseconds, Nanoseconds, Minutes, Hours }` | |
| `Time.fromSeconds/fromMilliseconds/fromMicroseconds/fromNanoseconds/fromMinutes/fromHours(value: Number)` | Companion factories. |
| `.asSeconds / .asMilliseconds / .asMicroseconds / .asNanoseconds / .asMinutes / .asHours: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on seconds. |
| `toString()` | `"Seconds($seconds)"`. |
| NaN/infinite guard | Shared behavior. |

`Percentage.kt` — a unit-safe percentage/ratio (e.g. motor percent output), always backed by a
`0.0..1.0`-style fraction internally. Per its own doc comment, **this class does not clamp to any
range itself** — callers (e.g. `HaMotor`) decide what range is valid and must clamp explicitly:

| Symbol | Description |
| --- | --- |
| `Percentage(percentage: Number, percentageUnit: Percentage.Unit)` | Primary constructor. |
| `enum Percentage.Unit { Fraction, Percent, Permille, BasisPoints }` | `Fraction` is `0.0..1.0 == 0%..100%`; `Permille` is parts per thousand (`0..1000`); `BasisPoints` is parts per ten-thousand (`0..10000`). |
| `Percentage.fromFraction/fromPercent/fromPermille/fromBasisPoints(value: Number)` | Companion factories. |
| `.asFraction / .asPercent / .asPermille / .asBasisPoints: Double` | Read out in a given unit. |
| `+`, `-`, `* Double`, `/ Double`, `unaryMinus`, `compareTo` | Shared shape, operating on the fraction. |
| `.coerceIn(min: Percentage, max: Percentage): Percentage` | Clamps the fraction into `[min.asFraction, max.asFraction]`. The only explicit clamping helper on this class — plain arithmetic and construction never clamp. |
| `toString()` | `"Percentage($asPercent%)"` (note: reports the percent value, not the fraction). |
| NaN/infinite guard | Shared behavior. |

`Extensions.kt` — numeric literal builders for every unit class above, plus `Rotation2d`/`Pose2d`
helpers built on top of this library's own `math/geometry` types:

| Symbol | Description |
| --- | --- |
| `Number.meters / .centimeters / .millimeters / .feet / .inches: Length` | e.g. `5.inches`. |
| `Number.rpm / .rps / .radPs / .degPs: AngularVelocity` | e.g. `435.rpm`. |
| `Number.metersPerSecond / .feetPerSecond / .inchesPerSecond / .centimetersPerSecond / .millimetersPerSecond / .kilometersPerHour / .milesPerHour: LinearVelocity` | |
| `Number.metersPerSecondSquared / .feetPerSecondSquared / .inchesPerSecondSquared / .gs: LinearAcceleration` | |
| `Number.radPs2 / .degPs2 / .rps2 / .rpmPerSecond: AngularAcceleration` | |
| `Number.kilograms / .grams / .pounds / .ounces: Mass` | |
| `Number.newtons / .poundsForce / .kilogramsForce / .dynes: Force` | |
| `Number.newtonMeters / .poundFeet / .poundInches / .ounceInches / .kilogramCentimeters: Torque` | |
| `Number.volts / .millivolts / .microvolts / .kilovolts: Voltage` | |
| `Number.amps / .milliamps / .microamps / .kiloamps: Current` | |
| `Number.seconds / .milliseconds / .microseconds / .nanoseconds / .minutes / .hours: Time` | |
| `Number.fraction / .percent / .permille / .basisPoints: Percentage` | |
| `Number.degrees / .radians / .rotations: Rotation2d` | e.g. `90.degrees`. `.radians` passes straight into `Rotation2d`'s primary (radian) constructor; `.degrees`/`.rotations` go through `Rotation2d.fromDegrees`. |
| `Rotation2d.absoluteValue: Rotation2d` | `abs()` on the degree value, preserving the type. |
| `Rotation2d.rotations: Double` | Degrees ÷ 360. |
| `Rotation2d.normalizedDegrees: Double` | Wrapped into `[0, 360)` — e.g. a `Rotation2d` built from 190° (which `Rotation2d` itself canonicalizes to -170°, since its own domain is `(-180, 180]`) reads back as `190.0` here. |
| `Rotation2d.normalizedRadians: Double` | Wrapped into `[0, 2π)`. |
| `Rotation2d.normalizedRotations: Double` | `normalizedDegrees / 360.0`, i.e. wrapped into `[0, 1)`. |
| `Rotation2d.times(ratio: Double) / .div(ratio: Double): Rotation2d` | Scales the degree value. |
| `Rotation2d.rangeTo(that: Rotation2d): ClosedRange<Double>` | Builds a range over the two rotations' **degree** values (not over `Rotation2d` itself). |
| `Rotation2d.compareTo(other: Rotation2d): Int` | Orders by degrees; also unlocks `<`, `>`. |
| `Pose2d.xDistanceTo(other: Pose2d): Double` | `other.x - this.x`, signed. |
| `Pose2d.yDistanceTO(other: Pose2d): Double` | `other.y - this.y`, signed. Note the capitalization: `TO`, not `To` — this is the actual method name, not a typo in this document. |
| `Pose2d.distanceTo(other: Pose2d): Double` | Euclidean distance via `xDistanceTo`/`yDistanceTO`. |
| `Pose2d.horizontalDistanceTo(other: Pose2d): Double` | Identical calculation to `distanceTo` — both exist as separate functions. |
| `Pose2d.horizontalAngleTo(other: Pose2d): Rotation2d` | Bearing from this pose to another, via `atan(Δy / Δx)`. |

`Conversions.kt` — free functions; every angle/angular-velocity/length conversion accepts any
`Number`:

| Category | Functions |
| --- | --- |
| Constants | `INCHES_IN_METER = 39.3700787402`; `DECODE_FIELD_LENGTH = Length.fromMeters(3.585719)`; `DECODE_FIELD_WIDTH = Length.fromMeters(3.585719)` — currently defined with the *same* numeric value as `DECODE_FIELD_LENGTH` (i.e. `DECODE_FIELD_WIDTH` is not independently set to the field's actual width). |
| Angle ↔ angle | `degToRad(deg)`, `radToDeg(rad)` |
| Angular velocity ↔ angular velocity | `rpmToRps`, `rpmToRadPs`, `rpmToDegPs`, `rpsToRpm`, `rpsToRadPs`, `rpsToDegPs`, `radPsToRpm`, `radPsToRps`, `radPsToDegPs`, `degPsToRpm`, `degPsToRps`, `degPsToRadPs` |
| Angular velocity → linear velocity | `rpmToMps(rpm, wheelRadius)`, `radPsToMps(radPs, wheelRadius)` (delegates through `rpmToMps`), `degPsToMps(degPs, wheelRadius)` (delegates through `rpmToMps`) |
| Linear velocity → angular velocity | `mpsToRpm(mps, wheelRadius)`, `mpsToRadPs(mps, wheelRadius)` (delegates through `mpsToRpm`), `mpsToDegPs(mps, wheelRadius)` (delegates through `mpsToRpm`) |
| Length ↔ length | `metersToInches`, `metersToFeet`, `inchesToMeters`, `inchesToFeet`, `feetToMeters`, `feetToInches` |
| Linear speed ↔ linear speed | `mpsToMMps`, `mpsToCMps`, `mpsToKph`, `mpsToIps`, `mpsToMph`, `mmpsToMps` (all one-way unit-scaling helpers; separate from, and unrelated to, `LinearVelocity`) |
| Field-relative | `matchPoseToAlliance(position: Pose2d, alliance: Alliance): Pose2d` — `position` is assumed to already be blue-alliance-relative. `Alliance.Blue` returns `position` unchanged; `Alliance.Red` mirrors `x` about `DECODE_FIELD_LENGTH` (`DECODE_FIELD_LENGTH.asMeters - position.x`) and adds 180° to the heading. |

`rpmToMps`/`mpsToRpm` (and, transitively, `radPsToMps`/`degPsToMps`/`mpsToRadPs`/`mpsToDegPs`,
which all delegate through them) log via `robotPrintError("wheelRadius is negative")` and return
`0.0` if `wheelRadius.asMeters` is not strictly positive (zero included).

`RoadRunnerConversions.kt` — RoadRunner has its own `Pose2d`/`Rotation2d` geometry types, separate
from the ones AlonLib uses everywhere else (its own `math.geometry.Rotation2d`/`Pose2d`). These
convert between them at the boundary, so hardware wrappers stay on AlonLib's types while
RoadRunner-specific code (drive/localizer/trajectories) uses its own:

| Symbol | Description |
| --- | --- |
| `Rotation2d.toRoadRunner(): RoadRunner Rotation2d` | Via `RoadRunnerRotation2d.exp(radians)`. |
| `RoadRunner Rotation2d.toRotation2d(): Rotation2d` | Via `Rotation2d.fromRadians(this.log())`. |
| `Pose2d.toRoadRunner(): RoadRunner Pose2d` | `RoadRunnerPose2d(x, y, rotation.radians)`. |
| `RoadRunner Pose2d.toPose2d(): Pose2d` | `Pose2d(Translation2d(position.x, position.y), Rotation2d(heading.log()))`. |

### `math/`

`LinearInterpolationTable.kt`

| Symbol | Description |
| --- | --- |
| `LinearInterpolationTable(vararg points: Point)` | Builds a lookup table from `(input, output)` points (`typealias Point = Pair<Double, Double>`). |
| `.getOutputFor(input: Double): Double` | Linearly interpolates between the two table points bracketing `input`; clamps to the nearest edge segment instead of extrapolating when `input` is outside the table's range. |
| `.firsts / .seconds: DoubleArray` | All the table's input (`.first`) or output (`.second`) values, in the order given to the constructor. |

`PIDFGains.kt`

| Symbol | Description |
| --- | --- |
| `PIDFGains(kP, kI, kD, kFF, kS, KV, Ka, kIZone)` | All `Double`, all default `0.0` (`@JvmOverloads`). Bundles a PID controller's gains plus feedforward (`kS`/`KV`/`Ka`) and an integral zone. `kIZone`: if the absolute error exceeds it, the integral accumulator is cleared — a feature motor controllers have but WPILib's `PIDController` doesn't. |
| `.toString()` | `"(kP: .. ,kI: .. ,Kd: .. ,kFF: .. ,kS:.. ,kV: .. ,kA:.. )"`. |
| `PIDController.configPID(gains: PIDFGains)` | Extension function. Applies `gains.kP/kI/kD` to a `PIDController`'s `p`/`i`/`d` (does not touch `kFF`/`kS`/`KV`/`Ka`/`kIZone`). |

`Operations.kt`

| Symbol | Description |
| --- | --- |
| `simpleDeadband(value: Double, deadband: Double): Double` | Returns `0.0` if `abs(value) < deadband`, else `value` unchanged. Logs and returns `value` if `deadband < 0`. |
| `continuousDeadband(value: Double, deadband: Double): Double` | Like `simpleDeadband`, but remaps the surviving range continuously onto `[0, 1]`/`[-1, 0]` instead of leaving a jump at the deadband boundary — e.g. `continuousDeadband(0.5, 0.1) == 0.4444`. `deadband` must be in `[0, 1]`, `value` in `[-1, 1]`; out-of-range logs and returns `value` unchanged. |
| `clamp(value: Double, min: Double, max: Double): Double` | `value.coerceIn(min, max)`; returns `0.0` (does not log) if `min > max`. |
| `mapRange(value, startMin, startMax, endMin, endMax): Double` | Linearly remaps `value` from `[startMin, startMax]` to `[endMin, endMax]`. Logs and returns `value` unchanged if either range is inverted/degenerate (`min >= max`). |
| `mapRange(value: Int, startMin: Int, startMax: Int, endMin: Int, endMax: Int): Int` | Same, for integers (delegates to the `Double` overload). |
| `interpolate(startValue: Double, endValue: Double, t: Double): Double` | Linear interpolation; `t = 0` returns `startValue`, `t = 1` returns `endValue`. Not clamped — `t` outside `[0, 1]` extrapolates. |
| `angleModulus(angle: Double): Double` | Wraps `angle` (radians) to `[-pi, pi]`. Implemented via `inputModulus`. |
| `inputModulus(value: Double, minimumInput: Double, maximumInput: Double): Double` | Wraps `value` into `[minimumInput, maximumInput]`; useful for continuous quantities like angles. Logs and returns `value` unchanged if `minimumInput >= maximumInput`. |
| `isNear(expected: Double, value: Double, tolerance: Double): Boolean` | True if `abs(expected - value) < tolerance`. Logs and returns `false` if `tolerance < 0`. |
| `isNear(expected, value, tolerance, minimumInput, maximumInput): Boolean` | Same, but wraps the error through `inputModulus` around `[minimumInput, maximumInput]` first, so e.g. 179° and -179° read as 2° apart instead of 358°. |
| `median(collection: Collection<Double>) / (array: Array<Double>) / (array: DoubleArray): Double` | The statistical median. Averages the two middle elements when the size is even (a 2-element collection averages both). |

### `math/control/`

WPILib-style feedback controllers and feedforwards, ported for FTC. Feedforward classes
(`ArmFeedforward`, `ElevatorFeedforward`, `SimpleMotorFeedforward`) follow the standard `kS`
(static friction) / `kV` (velocity) / `kA` (acceleration) / `kG` (gravity) control-theory
convention, with a `calculateWithVelocities` exact-discretization overload alongside the
continuous `calculate`.

`PIDFController.kt`

| Symbol | Description |
| --- | --- |
| `open class PIDFController(kp, ki, kd, kf, sp = 0.0, pv = 0.0)` | `u(t) = kP*e(t) + kI*∫e(t')dt' + kD*e'(t) + kF*r(t)`. Timing is wall-clock (`System.nanoTime()`), matching SolversLib's `PIDFController` — call `calculate(measurement)` every loop and it derives `dt` itself, unlike WPILib's `PIDController` which takes an explicit period. |
| `PIDFController(coefficients: PIDFCoefficients)` | Secondary constructor from an FTC SDK `PIDFCoefficients` (`f` included, `kIZone` ignored). |
| `enum IntegrationBehavior { NONE, CLEAR_AT_SETPOINT }` | `CLEAR_AT_SETPOINT` clears the accumulated integral once the controller reaches the setpoint within tolerance; `NONE` only clamps to `IntegrationControl`'s bounds. |
| `class IntegrationControl(integrationBehavior, decayFactor = 1.0, minIntegral = -1.0, maxIntegral = 1.0)` | Governs `totalError`: it's clamped to `[minIntegral, maxIntegral]` every step, multiplied by `decayFactor` whenever its sign disagrees with the current position error, and optionally cleared per `integrationBehavior`. `.setIntegrationBounds(min, max)` sets both bounds at once. |
| `var integrationControl: IntegrationControl` | Defaults to a fresh `IntegrationControl()`. |
| `var p, i, d, f: Double` | Gains; directly settable. |
| `val totalError: Double` | Accumulated integral (protected setter). |
| `var setPoint: Double` | Setting it immediately recomputes `positionError`/`velocityError` against the current `measuredValue`. |
| `val measuredValue, positionError, velocityError: Double` | Updated by `calculate`. |
| `val period: Double` | Wall-clock seconds since the previous `calculate` call. |
| `var toleranceP = 0.05, toleranceV = Double.POSITIVE_INFINITY` | Set via `setTolerance(positionTolerance, velocityTolerance = Double.POSITIVE_INFINITY)`. |
| `fun atSetPoint(): Boolean` | `abs(positionError) < toleranceP && abs(velocityError) < toleranceV`. |
| `var minOutput = 0.0, maxOutput = Double.POSITIVE_INFINITY` | Bound the *magnitude* of `calculate`'s output while not `atSetPoint()` (`minOutput`'s setter takes `abs(value)`); once at the setpoint the bound is not applied. |
| `var openF = 0.0` | A basic open-loop feedforward, sign-matched to `positionError`, added on top of every `calculate` call before the output-bound clamp. |
| `fun setPIDF(kp, ki, kd, kf)` / `fun setCoefficients(coefficients: PIDFCoefficients)` | Bulk gain setters. |
| `val coefficients: DoubleArray` | `[p, i, d, f]`. |
| `fun clearTotalError()` | Zeroes `totalError`. |
| `protected open fun calculateOutput(pv: Double): Double` | Core PIDF math: updates `period`/`positionError`/`velocityError`/`totalError` (applying the `IntegrationControl` clamp/decay/clear), and returns `p * proportionalTerm(positionError) + i * totalError + d * velocityError + f * setPoint`. Override point for subclasses that change the update loop (e.g. `CascadeController`). |
| `protected open fun proportionalTerm(error: Double): Double` | Defaults to `error` (linear P). Override to reshape the P term — `SquIDFController` sign-preserving-square-roots it. |
| `fun calculate(pv: Double): Double` | Runs `calculateOutput`, adds `sign(positionError) * openF`, then returns the raw sum if `atSetPoint()`, else clamps its magnitude to `[minOutput, maxOutput]` (sign-preserved). |
| `fun calculate(pv: Double, sp: Double): Double` | Sets `setPoint = sp`, then `calculate(pv)`. |
| `fun calculate(): Double` | `calculate(measuredValue)` — recomputes using the last-seen measurement. |
| `open fun reset()` | Zeroes `prevError`, `lastTimeStamp`, `totalError`. |

`PIDController.kt`

| Symbol | Description |
| --- | --- |
| `open class PIDController(kp, ki, kd, sp = 0.0, pv = 0.0) : PIDFController(kp, ki, kd, 0.0, sp, pv)` | A `PIDFController` with `f` fixed at `0.0`. |
| `fun setPID(kp, ki, kd)` | `setPIDF(kp, ki, kd, 0.0)`. |

`SquIDFController.kt`

| Symbol | Description |
| --- | --- |
| `class SquIDFController(kp, ki, kd, kf, sp = 0.0, pv = 0.0) : PIDFController(...)` | `u(t) = kP*sign(e)*sqrt(\|e\|) + kI*∫e(t')dt' + kD*e'(t) + kF*r(t)`. Overrides `proportionalTerm` to sign-preserving-square-root the error, flattening the P response far from the setpoint (a large error no longer produces a disproportionately large correction) while keeping the same direction and zero-crossing as linear P. |
| `SquIDFController(coefficients: PIDFCoefficients)` | Secondary constructor, same pattern as `PIDFController`. |

`CascadeController.kt`

| Symbol | Description |
| --- | --- |
| `class CascadeController(primary: PIDFController, secondary: PIDFController) : PIDFController(0.0, 0.0, 0.0, 0.0)` | A cascaded position -> velocity controller: `primary` converts position error into a velocity setpoint, which `secondary` chases. Gives a smoother, velocity-limited approach to a position setpoint than a single position PID. `primary`/`secondary` are driven internally — don't also call their own `calculate` independently. |
| `val measuredVelocity: Double` | Estimated by finite-differencing the measured position over `period` each `calculateOutput` call (skipped, i.e. left at its previous value, if `period` is ~0). |
| `fun setSetPoints(positionSetPoint: Double, velocitySetPoint: Double = 0.0)` | Sets both the outer position setpoint and inner velocity setpoint, and recomputes `positionError`/`velocityError` immediately. |
| `override fun calculateOutput(pv: Double): Double` | Updates `measuredVelocity`/errors from `pv`, runs `primary.calculate(pv, setPoint)` to get a velocity goal, then returns `secondary.calculate(measuredVelocity, velocityGoal + velocitySetPoint)`. |
| `override fun reset()` | Resets base state plus `measuredVelocity`, `primary`, and `secondary`. |

`BangBangController.kt`

| Symbol | Description |
| --- | --- |
| `class BangBangController(var tolerance: Double = Double.POSITIVE_INFINITY)` | The simplest possible controller: outputs `1.0` if `measurement < setPoint`, else `0.0`. Extremely aggressive; works well for velocity control of high-inertia mechanisms (flywheels), poorly for almost anything else. *Asymmetric* — never commands negative output, so it can only stop pushing an overspeeding mechanism, not slow it down; set motor controllers to coast (not brake) before using it. |
| `val setPoint, measurement: Double` | Last values passed to `calculate` (private setters). |
| `val error: Double` | `setPoint - measurement`. |
| `fun atSetPoint(): Boolean` | `abs(error) < tolerance`. |
| `fun calculate(measurement: Double, setPoint: Double): Double` | Updates `measurement`/`setPoint`, returns `1.0` or `0.0`. |
| `fun calculate(measurement: Double): Double` | Same, against the last-set `setPoint`. |

`ProfiledPIDController.kt`

| Symbol | Description |
| --- | --- |
| `class ProfiledPIDController(kp, ki, kd, var constraints: TrapezoidProfile.Constraints)` | A `PIDController` whose setpoint is constrained by a `TrapezoidProfile` instead of being tracked directly. Call `reset` before first use to avoid a spurious jump from `(0, 0)`. |
| `var goal, setpoint: TrapezoidProfile.State` | `goal` is the unprofiled target; `setpoint` is the profile's current output, fed to the internal PID each `calculate`. |
| `fun setPID(kp, ki, kd)` / `var p, i, d: Double` | Delegate to the internal `PIDController`. |
| `val period: Double` | Delegates to the internal controller's `period`. |
| `fun setGoal(goal: TrapezoidProfile.State)` / `fun setGoal(goal: Double)` | The `Double` overload sets a zero-velocity goal state. |
| `val atGoal: Boolean` | `atSetpoint && goal == setpoint`. |
| `val atSetpoint: Boolean` | The internal controller's `atSetPoint()`. |
| `fun setTolerance(positionTolerance, velocityTolerance = Double.POSITIVE_INFINITY)` | Delegates to the internal controller. |
| `val positionError, velocityError: Double` | Delegate to the internal controller. |
| `fun calculate(measurement: Double): Double` | Advances a new `TrapezoidProfile(constraints)` by `period` from `setpoint` towards `goal`, then feeds the profiled position to the internal PID as its setpoint. |
| `fun calculate(measurement, goal: TrapezoidProfile.State \| Double): Double` | Sets the goal, then `calculate(measurement)`. |
| `fun calculate(measurement, goal: TrapezoidProfile.State, constraints: TrapezoidProfile.Constraints): Double` | Also replaces `constraints` first. |
| `fun reset()` | Resets the internal PID's previous error/integral; does *not* reset `setpoint`/`goal`. |
| `fun reset(measurement: TrapezoidProfile.State)` / `fun reset(measuredPosition: Double, measuredVelocity: Double = 0.0)` | Also resets `setpoint` to the given state, so the profile continues from the actual measurement instead of jumping from `(0, 0)`. |

`TrapezoidProfile.kt`

| Symbol | Description |
| --- | --- |
| `class TrapezoidProfile(private val constraints: Constraints)` | A trapezoid-shaped velocity profile: accelerate at `maxAcceleration` up to `maxVelocity`, cruise, then decelerate into the goal. Typical usage: keep a `State` across loop iterations, and each loop call `state = profile.calculate(dt, state, goal)` — the unprofiled `goal` is free to change between calls. |
| `data class Constraints(val maxVelocity: Double, val maxAcceleration: Double)` | |
| `data class State(val position: Double = 0.0, val velocity: Double = 0.0)` | |
| `fun calculate(t: Double, current: State, goal: State): State` | The position/velocity at time `t` after `current`, profiled towards `goal`. Handles a goal behind the start (direction-flips internally) and nonzero initial/final velocities (treats the profile as if it began/ended at zero velocity, then trims). Also (re)computes `endAccel`/`endFullSpeed`/`endDecel` and caches `current` for use by `timeLeftUntil`/`totalTime`. Returns `goal` once `t` is past the profile's end. |
| `fun timeLeftUntil(target: Double): Double` | How much longer, from the state passed to the most recent `calculate`, until `target` (a position) is reached. Returns `0.0` if already within `1e-6` of it. |
| `fun totalTime(): Double` | The total duration of the profile computed by the most recent `calculate` (i.e. `endDecel`). |
| `fun isFinished(t: Double): Boolean` | `t >= totalTime()`. |

`SimpleMotorFeedforward.kt`

| Symbol | Description |
| --- | --- |
| `class SimpleMotorFeedforward(var ks, var kv, var ka = 0.0, val dt = 0.020)` | Feedforward for a simple permanent-magnet DC motor: `u = ks*sign(v) + kv*v + ka*a`. Units are whatever `kv`/`ka` were tuned in — radians for angular systems, meters for linear ones. |
| `fun calculate(velocity: Double, acceleration: Double = 0.0): Double` | Continuous-control feedforward at the given velocity/acceleration. |
| `fun calculateWithVelocities(currentVelocity: Double, nextVelocity: Double): Double` | Exact discrete-control feedforward stepping from `currentVelocity` to `nextVelocity` over `dt`; falls back to the simple `ks*sign(nextVelocity) + kv*nextVelocity` formula when `ka < 1e-9`. Inaccurate right where velocity crosses zero. |
| `fun maxAchievableVelocity(maxVoltage, acceleration): Double` / `fun minAchievableVelocity(maxVoltage, acceleration): Double` | Largest/smallest velocity achievable at a given acceleration without exceeding `maxVoltage`. |
| `fun maxAchievableAcceleration(maxVoltage, velocity): Double` / `fun minAchievableAcceleration(maxVoltage, velocity): Double` | Largest/smallest acceleration achievable at a given velocity without exceeding `maxVoltage` (`min` delegates to `max` with `-maxVoltage`). |

`ArmFeedforward.kt`

| Symbol | Description |
| --- | --- |
| `class ArmFeedforward(var ks, var kg, var kv, var ka = 0.0, val dt = 0.020)` | Feedforward for an arm (a motor fighting gravity on a beam pivoted at an angle): `u = ks*sign(v) + kg*cos(position) + kv*v + ka*a`. `position` is measured from horizontal (0 = arm parallel to the floor) — offset your encoder if it doesn't already follow that convention. |
| `fun calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared = 0.0): Double` | Continuous-control feedforward. |
| `fun maxAchievableVelocity(maxVoltage, angle, acceleration): Double` / `fun minAchievableVelocity(...): Double` | Largest/smallest velocity achievable at a given angle/acceleration without exceeding `maxVoltage`. |
| `fun maxAchievableAcceleration(maxVoltage, angle, velocity): Double` / `fun minAchievableAcceleration(...): Double` | Largest/smallest acceleration achievable at a given angle/velocity without exceeding `maxVoltage`. |

Note: unlike `ElevatorFeedforward`/`SimpleMotorFeedforward`, `ArmFeedforward` has no `calculateWithVelocities` discrete-stepping overload.

`ElevatorFeedforward.kt`

| Symbol | Description |
| --- | --- |
| `class ElevatorFeedforward(var ks, var kg, var kv, var ka = 0.0, val dt = 0.020)` | Feedforward for an elevator (a motor fighting gravity in a straight line): `u = ks*sign(v) + kg + kv*v + ka*a`. |
| `fun calculate(velocity, acceleration = 0.0): Double` | Continuous-control feedforward. |
| `fun calculateWithVelocities(currentVelocity, nextVelocity): Double` | Exact discrete-control feedforward over `dt`; falls back to `ks*sign(nextVelocity) + kg + kv*nextVelocity` when `ka < 1e-9`. Inaccurate right where velocity crosses zero. |
| `fun maxAchievableVelocity(maxVoltage, acceleration): Double` / `fun minAchievableVelocity(...): Double` | Largest/smallest velocity achievable at a given acceleration without exceeding `maxVoltage`. |
| `fun maxAchievableAcceleration(maxVoltage, velocity): Double` / `fun minAchievableAcceleration(...): Double` | Largest/smallest acceleration achievable at a given velocity without exceeding `maxVoltage`. |

`DifferentialDriveWheelVoltages.kt`

| Symbol | Description |
| --- | --- |
| `data class DifferentialDriveWheelVoltages(val left: Double = 0.0, val right: Double = 0.0)` | The per-side feedforward voltages computed by `DifferentialDriveFeedforward`. |

`DifferentialDriveFeedforward.kt`

| Symbol | Description |
| --- | --- |
| `class DifferentialDriveFeedforward(kVLinear, kALinear, kVAngular, kAAngular, trackwidthMeters)` | Computes per-side feedforward voltages for a differential drivetrain from its SysId-characterized kV/kA gains. This constructor takes `kVAngular`/`kAAngular` in per-(radians/sec) terms and converts them internally via `trackwidthMeters` (multiplied by `2.0 / trackwidthMeters`). |
| `class DifferentialDriveFeedforward(kVLinear, kALinear, kVAngular, kAAngular)` | Same, but `kVAngular`/`kAAngular` are already per-(meters/sec), matching the drivetrain's linear units — no trackwidth conversion. Internally builds a `LinearSystem` via `Models.identifyDrivetrainSystem`. |
| `val kVLinear, kALinear, kVAngular, kAAngular: Double` | The (possibly converted) gains, exposed read-only. |
| `fun calculate(currentLeftVelocity, nextLeftVelocity, currentRightVelocity, nextRightVelocity, dtSeconds): DifferentialDriveWheelVoltages` | The feedforward voltages to go from the current per-side velocities to the next ones over `dtSeconds`, via an internal `LinearPlantInversionFeedforward` on the drivetrain plant. |

`LinearPlantInversionFeedforward.kt`

| Symbol | Description |
| --- | --- |
| `class LinearPlantInversionFeedforward(plant: LinearSystem, dtSeconds: Double)` | A plant-inversion model-based feedforward: `u_ff = B⁺(r_{k+1} - A*r_k)`, where `B⁺` is the pseudoinverse of `B`. This constructor discretizes `plant.a`/`plant.b` over `dtSeconds`. |
| `class LinearPlantInversionFeedforward(a: Matrix, b: Matrix, dtSeconds: Double)` | Same, from raw continuous-time system matrices (discretized internally via `Discretization.discretizeAB`). |
| `val r: Matrix` | The current stored reference state (private setter, updated by `calculate`). |
| `val uff: Matrix` | The most recently computed feedforward input (private setter). |
| `fun reset(initialState: Matrix)` | Resets `r` to `initialState` and zeroes `uff`. |
| `fun reset()` | Resets `r` and `uff` to zero. |
| `fun calculate(nextR: Matrix): Matrix` | `calculate(r, nextR)`, continuing from the internally stored current reference. |
| `fun calculate(r: Matrix, nextR: Matrix): Matrix` | The feedforward for going from reference `r` (timestep k) to `nextR` (timestep k+1): `uff = B⁺(nextR - A*r)`; also updates the stored `r` to `nextR`. |

`LinearQuadraticRegulator.kt`

| Symbol | Description |
| --- | --- |
| `class LinearQuadraticRegulator` | The feedback control law `u = K(r - x)` that minimizes `sum((xᵀQx + uᵀRu) * dt)` subject to `x' = Ax + Bu`. See *Controls Engineering in the FIRST Robotics Competition* (https://file.tavsys.net/control/controls-engineering-in-frc.pdf) for the derivation. Note: WPILib's raw-`Matrix` overload of the per-tolerance constructor is intentionally omitted here (it would erase to the same JVM signature as the cost-matrix constructor without WPILib's separate `Vector` type) — call `StateSpaceUtil.makeCostMatrix` yourself instead if you need that path. |
| `LinearQuadraticRegulator(plant: LinearSystem, qElms: Matrix, rElms: Matrix, dtSeconds: Double)` | Builds per-state/per-input cost matrices from the diagonal elements `qElms`/`rElms` via `StateSpaceUtil.makeCostMatrix`, then delegates to the raw-matrix constructor using `plant.a`/`plant.b`. |
| `LinearQuadraticRegulator(a: Matrix, b: Matrix, q: Matrix, r: Matrix, dtSeconds: Double)` | Discretizes `a`/`b`, solves the discrete algebraic Riccati equation (`DARE.solve`) for `s`, and computes `K = (BᵀSB + R)⁻¹BᵀSA`. |
| `LinearQuadraticRegulator(a: Matrix, b: Matrix, q: Matrix, r: Matrix, n: Matrix, dtSeconds: Double)` | Same, with a state-input cross-term cost matrix `n`: `K = (BᵀSB + R)⁻¹(BᵀSA + Nᵀ)`. |
| `val k: Matrix` | The computed feedback gain matrix. |
| `val r: Matrix` | The current reference state (private setter). |
| `val u: Matrix` | The most recently computed control input (private setter). |
| `fun reset()` | Zeroes `r` and `u`. |
| `fun calculate(x: Matrix): Matrix` | `u = K * (r - x)` for the current state `x`, tracking the existing `r`. |
| `fun calculate(x: Matrix, nextR: Matrix): Matrix` | Sets `r = nextR`, then `calculate(x)`. |

`RamseteController.kt`

| Symbol | Description |
| --- | --- |
| `class RamseteController(private val b: Double = 2.0, private val zeta: Double = 0.7)` | A nonlinear time-varying feedback controller that drives a unicycle-model (differential drive) robot along a trajectory using its *global* pose, rather than per-wheel PID alone — so it can still converge after the robot has drifted off the path. Named for the Italian acronym ("Robotica Articolata e Mobile per i SErvizi e le TEcnologie") of the paper it's from; see section 8.2.2 of *Controls Engineering in the FIRST Robotics Competition*. `b` and `zeta` are the controller's tuning gains (convergence aggressiveness / damping). |
| `val poseError: Pose2d` | The most recent tracking error (private setter), default `Pose2d.kZero`. |
| `var poseTolerance: Pose2d` | Per-axis (x, y, rotation) tolerance used by `atReference()`. |
| `var enabled: Boolean = true` | When `false`, `calculate` bypasses feedback and returns the reference velocities unchanged (pure feedforward passthrough). |
| `fun atReference(): Boolean` | True if `poseError`'s x/y/rotation are each within `poseTolerance`'s corresponding component. |
| `fun calculate(currentPose: Pose2d, poseRef: Pose2d, linearVelocityRefMeters: Double, angularVelocityRefRadiansPerSecond: Double): ChassisSpeeds` | The chassis speeds that drive `currentPose` towards `poseRef`, tracking the given reference velocities, via `k = 2*zeta*sqrt(omegaRef² + b*vRef²)`, `v_cmd = vRef*cos(eTheta) + k*eX`, `omega_cmd = omegaRef + k*eTheta + b*vRef*sinc(eTheta)*eY`. |
| `fun calculate(currentPose: Pose2d, desiredState: Trajectory.State): ChassisSpeeds` | Convenience overload pulling pose/linear-velocity/angular-velocity (`velocity * curvature`) from a `Trajectory.State`. |

### `math/estimator/`

Kalman-filter-family state estimators and drivetrain pose estimators (WPILib-derived math, ported
onto this library's own `Matrix`/`Pose2d`/`Rotation2d`/kinematics types). All filters here are
stateful — construct one instance per system being estimated, and call `predict`/`correct` (or
`update`/`updateWithTime`) once per control-loop period.

`AngleStatistics.kt` — vector arithmetic (`-`/`+`/mean) for a state or measurement vector that has
one angle component, so [UnscentedKalmanFilter]/[ExtendedKalmanFilter] wrap it correctly across the
±π boundary instead of subtracting/averaging it like a plain linear quantity:

| Symbol | Description |
| --- | --- |
| `AngleStatistics.angleResidual(a: Matrix, b: Matrix, angleStateIdx: Int): Matrix` | `a - b`, with row `angleStateIdx` wrapped to `(-pi, pi]` via `angleModulus`. |
| `AngleStatistics.angleResidual(angleStateIdx: Int): (Matrix, Matrix) -> Matrix` | Curried form — pass directly as an EKF/UKF `residualFuncX`/`residualFuncY`. |
| `AngleStatistics.angleAdd(a: Matrix, b: Matrix, angleStateIdx: Int): Matrix` | `a + b`, same angle-wrapping on `angleStateIdx`. |
| `AngleStatistics.angleAdd(angleStateIdx: Int): (Matrix, Matrix) -> Matrix` | Curried form — pass as `addFuncX`. |
| `AngleStatistics.angleMean(sigmas: Matrix, wm: Matrix, angleStateIdx: Int): Matrix` | Weighted mean of `sigmas`' columns; row `angleStateIdx` is averaged circularly (`atan2` of the weighted mean sin/cos) instead of arithmetically. |
| `AngleStatistics.angleMean(angleStateIdx: Int): (Matrix, Matrix) -> Matrix` | Curried form — pass as a UKF `meanFuncX`/`meanFuncY`. |

`KalmanTypeFilter.kt` — the common interface implemented by [KalmanFilter], [ExtendedKalmanFilter],
and [UnscentedKalmanFilter], letting [KalmanFilterLatencyCompensator] drive any of them
interchangeably:

| Symbol | Description |
| --- | --- |
| `var p: Matrix` | The error covariance matrix. |
| `var xHat: Matrix` | The current state estimate. |
| `reset()` | Resets `xHat`/`p` to their initial values. |
| `predict(u: Matrix, dtSeconds: Double)` | Projects the state estimate forward by `dtSeconds` under control input `u`. |
| `correct(u: Matrix, y: Matrix)` | Corrects the state estimate given measurement `y` (for the same `u` passed to `predict`). |

`KalmanFilter.kt` — a linear Kalman filter: fuses a [`LinearSystem`][LinearSystem] plant model's
predictions with noisy measurements, weighted by the steady-state Kalman gain (solved once, via a
discrete algebraic Riccati equation, at construction time):

```kotlin
class KalmanFilter(
    states: Int,
    plant: LinearSystem,
    stateStdDevs: Matrix,    // process-noise std devs, one per state
    measurementStdDevs: Matrix, // measurement-noise std devs, one per output
    dtSeconds: Double,
) : KalmanTypeFilter
```

| Symbol | Description |
| --- | --- |
| `xHat`, `p`, `reset()`, `predict(u, dtSeconds)`, `correct(u, y)` | `KalmanTypeFilter` implementation. `predict` advances `xHat` via `plant.calculateX` and propagates `p` (`P = APAᵀ + Q`, with `A`/`Q` re-discretized for the given `dtSeconds`). `correct` applies the closed-form (non-adaptive) gain in Joseph form, for numerical stability. |
| `correct(u: Matrix, y: Matrix, r: Matrix)` | As `correct(u, y)`, but for a one-off measurement noise covariance `r` different from the one this filter was built with. |

`ExtendedKalmanFilter.kt` — a Kalman filter for nonlinear plant/measurement models
`x' = f(x, u)` / `y = h(x, u)`: linearizes both around the current state estimate via a numerical
Jacobian at every `predict`/`correct` call, then applies the linear Kalman filter equations:

```kotlin
class ExtendedKalmanFilter(
    states: Int, inputs: Int, outputs: Int,
    f: (Matrix, Matrix) -> Matrix,   // (x, u) -> x'
    h: (Matrix, Matrix) -> Matrix,   // (x, u) -> y
    stateStdDevs: Matrix,
    measurementStdDevs: Matrix,
    residualFuncY: (Matrix, Matrix) -> Matrix = Matrix::minus,
    addFuncX: (Matrix, Matrix) -> Matrix = Matrix::plus,
    dtSeconds: Double,
) : KalmanTypeFilter
```

| Symbol | Description |
| --- | --- |
| `xHat`, `p`, `reset()`, `predict(u, dtSeconds)`, `correct(u, y)` | `KalmanTypeFilter` implementation. `predict` integrates `xHat` forward via RK4 on `f`, and propagates `p` using `f`'s Jacobian at the current state, discretized for `dtSeconds`. `correct` linearizes `h` at the current state and applies the Kalman update in Joseph form. |
| `predict(u: Matrix, f: (Matrix, Matrix) -> Matrix, dtSeconds: Double)` | As `predict(u, dtSeconds)`, but linearizing/integrating a different (e.g. simplified) dynamics function instead of the constructor's `f`. |
| `correct(u: Matrix, y: Matrix, r: Matrix)` | As `correct(u, y)`, but for a one-off measurement noise covariance `r`. |
| `correct(rows: Int, u: Matrix, y: Matrix, h: (Matrix, Matrix) -> Matrix, r: Matrix, residualFuncY = Matrix::minus, addFuncX = Matrix::plus)` | As `correct(u, y)`, but for a different measurement vector shape/function entirely — lets one filter mix measurements from several sensors, each with its own `h`, row count, and `r`. |

Note: unlike upstream WPILib (which only seeds the initial `p` via a Riccati solve when `(A, C)` is
detectable, checked through a JNI call this port doesn't have), this port always attempts the
Riccati solve at construction.

`UnscentedKalmanFilter.kt` — a Kalman filter for the same nonlinear `x' = f(x, u)` / `y = h(x, u)`
models as `ExtendedKalmanFilter`, but propagating the error covariance using sigma points
([MerweScaledSigmaPoints]) instead of a linearization — more accurate for strongly nonlinear models,
at higher cost per step:

```kotlin
class UnscentedKalmanFilter(
    states: Int, outputs: Int,
    f: (Matrix, Matrix) -> Matrix,
    h: (Matrix, Matrix) -> Matrix,
    stateStdDevs: Matrix,
    measurementStdDevs: Matrix,
    meanFuncX: (Matrix, Matrix) -> Matrix = ::weightedMean,
    meanFuncY: (Matrix, Matrix) -> Matrix = ::weightedMean,
    residualFuncX: (Matrix, Matrix) -> Matrix = Matrix::minus,
    residualFuncY: (Matrix, Matrix) -> Matrix = Matrix::minus,
    addFuncX: (Matrix, Matrix) -> Matrix = Matrix::plus,
    dtSeconds: Double,
) : KalmanTypeFilter
```

| Symbol | Description |
| --- | --- |
| `xHat`, `p`, `reset()`, `predict(u, dtSeconds)`, `correct(u, y)` | `KalmanTypeFilter` implementation. `predict` draws sigma points from the current `(xHat, p)`, integrates each through `f` via RK4, and recombines them (via `meanFuncX`/`residualFuncX`) into the new `xHat`/`p`. `correct` passes the same sigma points through `h` and applies the Kalman update in the measurement space. Pass `meanFuncX`/`meanFuncY`/`residualFuncX`/`residualFuncY`/`addFuncX` overrides (e.g. from [AngleStatistics]) when a state or measurement component is an angle. |
| `correct(u: Matrix, y: Matrix, r: Matrix)` | As `correct(u, y)`, but for a one-off measurement noise covariance `r`. |
| `correct(rows, u, y, h, r, meanFuncY, residualFuncY, residualFuncX, addFuncX)` | As `correct(u, y)`, but for a different measurement vector shape/function — mixes in a different sensor's `h`/rows/`r`/functions for one call. |

Note: upstream WPILib implements this as a square-root-form filter (tracking a Cholesky factor
instead of the covariance directly, via QR decomposition and rank-1 updates) for better numerical
conditioning; this port uses the classical covariance-form UKF instead, built on this port's
existing `Matrix` inverse/solve/multiply primitives — mathematically equivalent, but without the
square-root form's extra numerical robustness.

`MerweScaledSigmaPoints.kt` — generates the `2*states+1` sigma points and weights `UnscentedKalmanFilter`
uses, per Van der Merwe's 2004 dissertation:

```kotlin
class MerweScaledSigmaPoints(states: Int, alpha: Double = 1e-3, beta: Double = 2.0, kappa: Int = 3 - states)
```

| Symbol | Description |
| --- | --- |
| `.numSigmas: Int` | `2 * states + 1`. |
| `.wm: Matrix` | Weights (`numSigmas x 1`) for computing the sigma points' mean. |
| `.wc: Matrix` | Weights (`numSigmas x 1`) for computing the sigma points' covariance. |
| `.sigmaPoints(x: Matrix, p: Matrix): Matrix` | The `states x numSigmas` sigma points around mean `x` with covariance `p` (via a Cholesky factor of `p`, scaled by `alpha`/`kappa`). |

`KalmanFilterLatencyCompensator.kt` — incorporates a time-delayed global measurement (e.g. vision,
which typically arrives a frame or two late) into a `KalmanTypeFilter`'s estimate: rewinds the
filter to its recorded state as of the measurement's timestamp, applies the correction there, then
replays every input/local measurement recorded since to catch back up to the present. Stateful — it
buffers up to 300 past observer snapshots (oldest dropped once exceeded), so use one instance per
filter it's compensating.

| Symbol | Description |
| --- | --- |
| `reset()` | Clears the snapshot buffer. |
| `addObserverState(observer: KalmanTypeFilter, u: Matrix, localY: Matrix, timestampSeconds: Double)` | Records `observer`'s current `xHat`/`p` plus the input/local measurement used, timestamped. Call this every loop, right after your own `predict`/`correct`. |
| `applyPastGlobalMeasurement(observer: KalmanTypeFilter, nominalDtSeconds: Double, y: Matrix, globalMeasurementCorrect: (Matrix, Matrix) -> Unit, timestampSeconds: Double)` | Binary-searches the buffer for the snapshot closest to `timestampSeconds`, rewinds `observer` to it, replays `predict`/`correct` for every snapshot from there to now (applying `globalMeasurementCorrect(u, y)` at the rewind point), and leaves `observer` at the caught-up present state. No-op if the buffer is empty. |

`PoseEstimator.kt` — wraps an `Odometry<WheelPositions>` to fuse latency-compensated global pose
measurements (e.g. vision) with wheel/gyro odometry. A drop-in replacement for `Odometry` — behaves
identically to it as long as `addVisionMeasurement` is never called. This is the shared base for the
three drivetrain-specific estimators below; use one of those rather than this directly.

```kotlin
open class PoseEstimator<WheelPositions>(
    odometry: Odometry<WheelPositions>,
    stateStdDevs: Matrix,          // [x, y, theta] std devs — trust in odometry
    visionMeasurementStdDevs: Matrix, // [x, y, theta] std devs — trust in vision
)
```

| Symbol | Description |
| --- | --- |
| `.estimatedPosition: Pose2d` (read-only) | The current fused pose estimate. |
| `.setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix)` | Recomputes the closed-form vision correction gain — call this to change how much future `addVisionMeasurement` calls are trusted (e.g. as distance to a vision target changes). |
| `.resetPosition(gyroAngle: Rotation2d, wheelPositions: WheelPositions, pose: Pose2d)` | Resets the underlying odometry to `pose` given the current gyro/wheel readings, and clears all buffered odometry/vision history. |
| `.resetPose(pose: Pose2d)` / `.resetTranslation(translation: Translation2d)` / `.resetRotation(rotation: Rotation2d)` | Reset just the pose / just translation / just rotation; each also clears buffered odometry/vision history. |
| `.sampleAt(timestampSeconds: Double): Pose2d?` | The estimated pose at `timestampSeconds` (vision-compensated if a vision update at/before that time exists), clamped into the buffered time range. `null` only if there's no odometry history yet. |
| `.addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double)` | Corrects the estimate toward a global measurement taken at `timestampSeconds`. Can be called as infrequently as needed as long as `updateWithTime`/`update` is still called every loop. Silently dropped if `timestampSeconds` is older than the ~1.5s odometry history buffer, or if no odometry sample exists yet. For stability, prefer feeding in measurements already within roughly a meter of the current estimate. |
| `.addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix)` | As above, also calling `setVisionMeasurementStdDevs` first (so it applies to this and future calls). |
| `.update(gyroAngle: Rotation2d, wheelPositions: WheelPositions): Pose2d` | Integrates the latest odometry reading, timestamped with the current wall-clock time (`System.nanoTime()`). **Call once per loop.** |
| `.updateWithTime(currentTimeSeconds: Double, gyroAngle: Rotation2d, wheelPositions: WheelPositions): Pose2d` | As `update`, with an explicit timestamp — use this instead of `update` when you need it on the same clock as your vision measurements. |

`DifferentialDrivePoseEstimator.kt` — `PoseEstimator` for a differential (tank) drivetrain:

```kotlin
class DifferentialDrivePoseEstimator(
    kinematics: DifferentialDriveKinematics, // accepted for API-shape parity, not actually used
    gyroAngle: Rotation2d,
    leftDistanceMeters: Double, rightDistanceMeters: Double,
    initialPose: Pose2d,
    stateStdDevs: Matrix, visionMeasurementStdDevs: Matrix,
) : PoseEstimator<DifferentialDriveWheelPositions>
```

| Symbol | Description |
| --- | --- |
| `.update(gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double): Pose2d` | Convenience overload of `PoseEstimator.update` taking raw wheel distances instead of a `DifferentialDriveWheelPositions`. |
| `.updateWithTime(currentTimeSeconds: Double, gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double): Pose2d` | Same, for `updateWithTime`. |

Note: `kinematics` isn't actually used — `DifferentialDriveOdometry` (which this wraps) tracks
translation from raw wheel-distance averages and takes rotation from the gyro, neither of which
depends on the trackwidth. Kept only for constructor-shape parity with the other two estimators.

`MecanumDrivePoseEstimator.kt` — `PoseEstimator` for a mecanum drivetrain; adds no members beyond
the base class:

```kotlin
class MecanumDrivePoseEstimator(
    kinematics: MecanumDriveKinematics,
    gyroAngle: Rotation2d,
    wheelPositions: MecanumDriveWheelPositions,
    initialPose: Pose2d,
    stateStdDevs: Matrix, visionMeasurementStdDevs: Matrix,
) : PoseEstimator<MecanumDriveWheelPositions>
```

`SwerveDrivePoseEstimator.kt` — `PoseEstimator` for a swerve drivetrain; adds no members beyond the
base class:

```kotlin
class SwerveDrivePoseEstimator(
    kinematics: SwerveDriveKinematics,
    gyroAngle: Rotation2d,
    modulePositions: Array<SwerveModulePosition>,
    initialPose: Pose2d,
    stateStdDevs: Matrix, visionMeasurementStdDevs: Matrix,
) : PoseEstimator<Array<SwerveModulePosition>>
```

### `math/filter/`

Single-value signal-processing filters and debouncers — simpler, standalone tools that don't depend
on the estimator/`Matrix` machinery above (`LinearFilter`'s constructor is the one exception, using
`Matrix` internally to solve for finite-difference coefficients). **All of these are stateful — use
a separate instance per input stream/signal.**

`DoubleCircularBuffer.kt` — a fixed-capacity ring buffer of `Double`s, growing from the front. Backs
`LinearFilter`'s and `MedianFilter`'s tap/sample history; index `0` is always the most recently
added element:

```kotlin
class DoubleCircularBuffer(capacity: Int)
```

| Symbol | Description |
| --- | --- |
| `.size: Int` (read-only) | Current element count (`<= capacity`). |
| `.addFirst(value: Double)` | Pushes `value` to the front, evicting the oldest element once at capacity. No-op if `capacity == 0`. |
| `operator get(index: Int): Double` | The element `index` slots back from the front (`0` = most recent); doesn't remove it. Throws if `index !in 0 until size`. |
| `.removeLast(): Double` | Removes and returns the oldest element (the back of the buffer). Throws if empty. |
| `.getFirst(): Double` | The most recently added element. Throws if empty. |
| `.clear()` | Empties the buffer. |

`LinearFilter.kt` — a general linear FIR/IIR digital filter:
`y[n] = (b0*x[n] + ... + bP*x[n-P]) - (a0*y[n-1] + ... + aQ*y[n-Q])`. `calculate` must be called on a
known, regular period — filter gains are inherently a function of sample rate. Prefer the companion
factories below over the raw-gains constructor unless hand-deriving your own filter:

```kotlin
class LinearFilter(feedforwardGains: DoubleArray, feedbackGains: DoubleArray)
```

| Symbol | Description |
| --- | --- |
| `.lastValue: Double` (read-only) | The most recent `calculate` output (`0.0` before the first call). |
| `.calculate(input: Double): Double` | Pushes `input` through the filter and returns the new output. Call once per period. |
| `.reset()` | Clears input/output history (does not change `lastValue`'s underlying state — a subsequent `calculate` starts from empty history). |
| `.reset(inputBuffer: DoubleArray, outputBuffer: DoubleArray)` | Resets then seeds the input/output history (most-recent-first) from the given buffers. Logs via `robotPrintError` and leaves the filter cleared (not seeded) if either buffer's size doesn't match this filter's tap counts. |
| `LinearFilter.singlePoleIIR(timeConstant: Double, period: Double): LinearFilter` | One-pole IIR low-pass: `y[n] = (1-gain)x[n] + gain*y[n-1]`, `gain = e^(-period/timeConstant)`. Stable for any positive `timeConstant`. `timeConstant = 1/(2*pi*f)` for cutoff frequency `f` (Hz). |
| `LinearFilter.highPass(timeConstant: Double, period: Double): LinearFilter` | First-order high-pass: `y[n] = gain(x[n]-x[n-1]) + gain*y[n-1]`, same `gain` formula. Attenuates frequencies below the cutoff. |
| `LinearFilter.movingAverage(taps: Int): LinearFilter` | A `taps`-tap FIR moving average, always stable. `taps <= 0` logs via `robotPrintError` and returns a pass-through (no-op) filter instead. |
| `LinearFilter.finiteDifference(derivative: Int, stencil: IntArray, period: Double): LinearFilter` | Approximates the `derivative`-th derivative of the input, sampled at `stencil` points (`0` = current sample, `-1` = previous, etc.; avoid positive stencil points for real-time/streaming use — those need future samples). Logs and returns a pass-through filter if `derivative < 1`, `stencil` is empty, or `derivative >= stencil.size`. |
| `LinearFilter.backwardFiniteDifference(derivative: Int, samples: Int, period: Double): LinearFilter` | `finiteDifference` using `samples` consecutive past+current samples as the stencil (purely backward-looking, safe for streaming use). |

`MedianFilter.kt` — a moving-window median filter. Unlike `LinearFilter.movingAverage`, a median is
robust to occasional extreme outliers (e.g. a bad vision/LIDAR/ultrasonic reading) instead of
averaging them in:

```kotlin
class MedianFilter(windowSize: Int)
```

| Symbol | Description |
| --- | --- |
| `.calculate(next: Double): Double` | Pushes `next` in (dropping the oldest sample once `windowSize` is exceeded) and returns the median of the current window. Call once per period. |
| `.lastValue(): Double` | The most recently pushed raw sample (not the filtered/median output). |
| `.reset()` | Clears all previous samples. |

`SlewRateLimiter.kt` — limits how fast a value can change, in units/second (e.g. ramping a voltage
or setpoint). For limiting a *position* rather than a rate, prefer
`math.control.TrapezoidProfile` instead:

```kotlin
class SlewRateLimiter(
    positiveRateLimit: Double,
    negativeRateLimit: Double = -positiveRateLimit,
    initialValue: Double = 0.0,
)
```

| Symbol | Description |
| --- | --- |
| `.lastValue: Double` (read-only) | The limiter's current output. |
| `.calculate(input: Double): Double` | Moves `lastValue` toward `input`, clamped to how far it can move given the elapsed wall-clock time (`System.nanoTime()`) since the previous call and the positive/negative rate limits. Call every loop with the desired target value. |
| `.reset(value: Double)` | Snaps `lastValue` to `value` immediately, bypassing the rate limit. |

`Debouncer.kt` — requires a boolean input to hold steady away from its baseline for
`debounceTimeSeconds` before the debounced output follows it (filters out brief flickers, e.g. a
bouncy limit switch):

```kotlin
class Debouncer(var debounceTimeSeconds: Double, var type: DebounceType = DebounceType.RISING)
enum class DebounceType { RISING, FALLING, BOTH }
```

| Symbol | Description |
| --- | --- |
| `.calculate(input: Boolean): Boolean` | Returns the debounced output for `input`. `RISING` only debounces `false -> true` transitions (a `true -> false` change passes through immediately); `FALLING` is the mirror image; `BOTH` debounces every transition. Call every loop with the raw input. |

`GenericDebouncer.kt` — a `Debouncer` for arbitrary types, not just `Boolean` (e.g. a color sensor's
detected color, or which AprilTag is currently visible). Requires `input` to hold steady (by
`equals`) for `debounceMillis` before `state` follows it:

```kotlin
class GenericDebouncer<T>(var debounceMillis: Double, initial: T)
```

| Symbol | Description |
| --- | --- |
| `.state: T` (read-only) | The current debounced value. |
| `.calculate(input: T): T` | Feeds `input` in and returns the (possibly still-debouncing) `state`. Call every loop. |
| `.reset(newState: T)` | Sets `state` (and resets the debounce timer) to `newState` immediately — for when you know the value changed and don't want to wait out the debounce. |

### `math/filters/movingwindowfilters/`

An older, simpler moving-window filter pair, predating `math/filter/`. **`MovingAverageFilter` here
duplicates `LinearFilter.movingAverage` and `MovingMedianFilter` here duplicates `MedianFilter`,
above** — same core behavior (finite-memory moving average/median), different API shape (an
inheritance hierarchy over a `LinkedList<Double>` here, vs. a `DoubleCircularBuffer`-backed
standalone class there). Prefer `math/filter/`'s `LinearFilter`/`MedianFilter` for new code; this
package exists for backward compatibility with existing call sites.

`MovingWindowFilter.kt` — abstract base for a finite-memory low-pass filter. **Since filters have
memory, use a separate instance per input stream.**

| Symbol | Description |
| --- | --- |
| `abstract var window: Int` | Number of samples included in the calculation. Subclasses validate it via their own setter (see below) — assigning through a constructor argument, rather than the `window = ...` property setter, bypasses that validation (Kotlin property initializers write the backing field directly). |
| `.calculate(newSample: Double): Double` | Pushes `newSample` in (dropping the oldest sample once `window` is exceeded) and returns the filter's output for the updated sample set. If fewer samples than `window` have been provided so far, the calculation simply uses fewer samples. Call once per period. |
| `.reset(newValues: DoubleArray)` / `(Collection<Double>)` / `(Array<Double>)` | Clears all previous samples and refills from `newValues` (only the first `window` are kept if longer; left short if shorter). |
| `.reset(newValue: Double)` | Clears all previous samples and fills the whole window with `newValue`. |
| `.clear()` | Clears all previous samples without refilling. |

| Concrete filter | `calculation` | Notes |
| --- | --- | --- |
| `MovingAverageFilter(window: Int)` (`open class`) | `values.average()` | Setting `.window` to a non-positive value logs via `robotPrintError` and clamps it to `0` instead; the constructor argument itself is not validated. |
| `MovingMedianFilter(window: Int)` | `median(values)` (see `math.median`) | Good for rejecting occasional outliers; same `window <= 0` setter guard as above. |

### `math/geometry/`

Ported from WPILib's geometry package (`Translation`/`Rotation`/`Pose`/`Transform`/`Twist`, 2D and
3D), plus a small coordinate-system-conversion layer. This is a separate implementation from
RoadRunner's own `Rotation2d`/`Pose2d` — RoadRunner-specific code (trajectories, drive/localizer)
uses RoadRunner's types, and `units/RoadRunnerConversions.kt` converts at the boundary; everything
else in AlonLib (`HaMotor`, `HaServo`, `HaPinPoint`, the units system, ...) uses these types
throughout. All linear units are meters, all angular units radians unless a symbol says otherwise.

`Rotation2d.kt` — a 2D rotation, stored internally as `(cos, sin)` rather than a raw angle so
composing rotations (`rotateBy`) is a cheap multiply instead of another trig call:

| Symbol | Description |
| --- | --- |
| `Rotation2d(radians: Double = 0.0)` | From an angle. |
| `Rotation2d(x: Double, y: Double)` | From the angle of the vector `(x, y)`, e.g. a joystick direction. Returns `0.0` if the vector is ~zero-length instead of `NaN`. |
| `.radians` / `.degrees` / `.rotations: Double` | Read out in a given unit. |
| `.cos` / `.sin` / `.tan: Double` | |
| `+`, `-`, unary `-`, `* Double`, `/ Double` | `+`/`-` compose via `rotateBy` (with the RHS negated for `-`); the rest scale `radians`. |
| `.rotateBy(other: Rotation2d): Rotation2d` | Composes this rotation with `other`. Result stays normalized to `(-180°, 180°]` instead of drifting across repeated compositions (e.g. many odometry ticks). |
| `.interpolate(endValue: Rotation2d, t: Double): Rotation2d` | `Interpolatable` — lerps via `this + (endValue - this) * t`. |
| `Rotation2d.kZero` / `.kPi` | `0°` / `180°`. |
| `Rotation2d.fromDegrees/fromRadians/fromRotations(value: Double)` | Companion factories. |

`Rotation3d.kt` — a 3D rotation backed by a `Quaternion`. Unlike `Rotation2d`, 3D rotations don't
commute — `rotateBy` applies extrinsically (around the fixed global axes), `relativeTo` applies
intrinsically (from the other rotation's own frame):

| Symbol | Description |
| --- | --- |
| `Rotation3d()` | Identity. |
| `Rotation3d(q: Quaternion)` | From a quaternion (normalized on construction). |
| `Rotation3d(roll: Double, pitch: Double, yaw: Double)` | From extrinsic roll/pitch/yaw, applied in that order around the fixed global axes (not the body frame). |
| `Rotation3d(rotationVector: Translation3d)` | From an axis-angle rotation vector (axis direction × angle, in radians, as the vector's own magnitude). |
| `Rotation3d(axis: Translation3d, angleRadians: Double)` | From an explicit (not-necessarily-normalized) axis and angle. |
| `Rotation3d(rotation2d: Rotation2d)` | From a 2D rotation in the X-Y plane (pure yaw). |
| `.quaternion: Quaternion` | |
| `.x` / `.y` / `.z: Double` | Extracted roll/pitch/yaw. |
| `.axis: Translation3d` / `.angle: Double` | The axis-angle representation. |
| `.inverse(): Rotation3d` | |
| `.times(scalar: Double)` / `.div(scalar: Double): Rotation3d` | **Not operators** — call `.times()`/`.div()` directly. Implemented as `kZero.interpolate(this, scalar)` (slerp from identity), so this is not simple angle scaling for large `scalar`. |
| `.rotateBy(other: Rotation3d): Rotation3d` | Extrinsic composition (`other.quaternion * this.quaternion`). |
| `.relativeTo(other: Rotation3d): Rotation3d` | Intrinsic — this rotation re-expressed relative to `other`'s orientation. |
| `.integrate(rollRate: Double, pitchRate: Double, yawRate: Double, dtSeconds: Double): Rotation3d` | Projects this rotation forward by constant body-frame angular rates over `dtSeconds`. |
| `.toMatrix(): Array<DoubleArray>` | Row-major 3×3 rotation matrix. |
| `.toVector(): Translation3d` | Axis-angle rotation vector (SO(3) log). |
| `.toRotation2d(): Rotation2d` | This rotation's yaw component, projected into the X-Y plane. |
| `.interpolate(endValue: Rotation3d, t: Double): Rotation3d` | Slerp, shortest-path (negates the delta quaternion when needed). |
| `Rotation3d.kZero` | Identity. |
| `Rotation3d.fromRadians/fromDegrees(roll, pitch, yaw): Rotation3d` | |
| `Rotation3d.fromRotationMatrix(rotationMatrix: Array<DoubleArray>): Rotation3d` | Via Shepperd's method. Logs via `robotPrintError` and returns `kZero` instead of throwing if the matrix isn't special-orthogonal (not a valid rotation). |
| `Rotation3d.fromVectorToVector(initial: Translation3d, last: Translation3d): Rotation3d` | The rotation that carries `initial` onto `last` (any two nonzero vectors). Handles the antiparallel case (180° about an arbitrary orthogonal axis). |

`Quaternion.kt` — backs `Rotation3d`; not normally used directly. Supports general (non-unit)
`exp`/`log`/`pow`, which `Rotation3d.interpolate`'s slerp is built on:

| Symbol | Description |
| --- | --- |
| `Quaternion(w: Double = 1.0, x: Double = 0.0, y: Double = 0.0, z: Double = 0.0)` | |
| `+`, `-`, `* Double`, `/ Double` | Componentwise. |
| `* Quaternion` | Hamilton product. |
| `.conjugate() / .inverse() / .norm() / .normalize(): Quaternion` (except `.norm()`, `Double`) | |
| `.dot(other: Quaternion): Double` | |
| `.pow(t: Double): Quaternion` | `exp(t * log(q))`. |
| `.exp(): Quaternion` / `.log(): Quaternion` | Matrix exponential/logarithm; inverses of each other. |
| `.toRotationVector(): Translation3d` | The axis-angle vector (SO(3) log) this unit quaternion represents. |
| `Quaternion.fromRotationVector(rvec: Translation3d): Quaternion` | The 𝖘𝖔(3) exp of an axis-angle rotation vector. |

`Translation2d.kt` — a point in a 2D coordinate frame (as opposed to `Vector2d`, a free vector):

| Symbol | Description |
| --- | --- |
| `Translation2d(x: Double = 0.0, y: Double = 0.0)` | |
| `Translation2d(distance: Double, angle: Rotation2d)` | Polar: `distance` at `angle` from the origin. |
| `.x` / `.y: Double`, `.norm: Double`, `.angle: Rotation2d` | |
| `.getDistance(other: Translation2d): Double` | Euclidean distance. |
| `.rotateBy(other: Rotation2d): Translation2d` | Around the origin. |
| `.rotateAround(other: Translation2d, rotation: Rotation2d): Translation2d` | Around an arbitrary point. |
| `+`, `-`, unary `-`, `* Double`, `/ Double` | |
| `.interpolate(endValue: Translation2d, t: Double): Translation2d` | Componentwise lerp. |
| `Translation2d.kZero` | |

`Translation3d.kt` — a point in 3D space:

| Symbol | Description |
| --- | --- |
| `Translation3d(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0)` | |
| `Translation3d(distance: Double, angle: Rotation3d)` | Polar: `distance` at `angle` from the origin. |
| `Translation3d(translation: Translation2d)` | From a 2D translation, `z = 0`. |
| `.x` / `.y` / `.z: Double`, `.norm` / `.squaredNorm: Double` | |
| `.getDistance(other) / .getSquaredDistance(other): Double` | |
| `.rotateBy(other: Rotation3d): Translation3d` | Around the origin, via the quaternion sandwich product. |
| `.rotateAround(other: Translation3d, rotation: Rotation3d): Translation3d` | Around an arbitrary point. |
| `.dot(other: Translation3d) / .cross(other: Translation3d)` | |
| `.toTranslation2d(): Translation2d` | Projected into the X-Y plane. |
| `+`, `-`, unary `-`, `* Double`, `/ Double` | |
| `.nearest(translations: Collection<Translation3d>): Translation3d` | Closest by `getDistance`. |
| `.interpolate(endValue: Translation3d, t: Double): Translation3d` | Componentwise lerp. |
| `Translation3d.kZero` | |

`Vector2d.kt` — a free vector in 2D (as opposed to `Translation2d`, a point). Kept as a separate
type (matching SolversLib's own split) because drivebase code wants vector algebra (dot/project/
normalize) that doesn't make sense for a point:

| Symbol | Description |
| --- | --- |
| `Vector2d(x: Double = 0.0, y: Double = 0.0)` | |
| `Vector2d(pose: Pose2d)` | From a pose's `(x, y)`. |
| `.rotateBy(angleRadians: Double): Vector2d` | Counter-clockwise. Takes a raw radians `Double`, unlike `Translation2d.rotateBy`'s `Rotation2d`. |
| `.angle(): Double` | Via `atan2(y, x)`, radians. |
| `+`, `-`, unary `-`, `* Double`, `/ Double` | |
| `.dot(other: Vector2d): Double`, `.magnitude(): Double` | |
| `.scale(scalar: Double): Vector2d` | Same as `* scalar`. |
| `.normalize(): Vector2d` | Unit vector in this direction. |
| `.scalarProject(other: Vector2d): Double` | Scalar projection of this vector onto `other`. |
| `.project(other: Vector2d): Vector2d` | Vector projection of this vector onto `other`. |

`Pose2d.kt` — a robot pose (position + heading):

| Symbol | Description |
| --- | --- |
| `Pose2d(translation: Translation2d = kZero, rotation: Rotation2d = kZero)` | |
| `Pose2d(x: Double, y: Double, rotation: Rotation2d)` | |
| `.translation: Translation2d`, `.rotation: Rotation2d`, `.x` / `.y: Double` | |
| `.rotateBy(other: Rotation2d): Pose2d` | Rotates both translation and rotation around the origin. |
| `.transformBy(other: Transform2d): Pose2d` | Applies a relative transform (expressed in this pose's rotated frame). |
| `+ Transform2d` | Same as `transformBy`. |
| `- Pose2d` | Returns the `Transform2d` from `other` to this pose. |
| `* Double` / `/ Double` | |
| `.relativeTo(other: Pose2d): Pose2d` | This pose re-expressed relative to `other` instead of the field/origin frame. |
| `.exp(twist: Twist2d): Pose2d` | Integrates a constant-curvature `Twist2d` forward from this pose — e.g. one odometry tick's worth of wheel motion — more accurately than naively adding `(dx, dy, dtheta)` straight onto the pose. |
| `.log(end: Pose2d): Twist2d` | The `Twist2d` that `exp` would integrate from this pose to reach `end`. Inverse of `exp`. |
| `.interpolate(endValue: Pose2d, t: Double): Pose2d` | Via `log`/`exp` — i.e. constant-curvature interpolation, not independent lerp/slerp of translation and rotation (contrast `Pose3d.interpolate`). |
| `Pose2d.kZero` | |

`Pose3d.kt` — a pose in 3D space:

| Symbol | Description |
| --- | --- |
| `Pose3d(translation: Translation3d = kZero, rotation: Rotation3d = kZero)` | |
| `Pose3d(x: Double, y: Double, z: Double, rotation: Rotation3d)` | |
| `Pose3d(pose: Pose2d)` | From a 2D pose in the X-Y plane (`z = 0`, pure yaw). |
| `.translation: Translation3d`, `.rotation: Rotation3d`, `.x` / `.y` / `.z: Double` | |
| `+ Transform3d` | Same as `transformBy`. |
| `- Pose3d` | Returns the `Transform3d` from the other pose to this one. |
| `.times(scalar: Double)` / `.div(scalar: Double): Pose3d` | **Not operators**, unlike `Pose2d`. |
| `.rotateBy(other: Rotation3d): Pose3d` | Rotates translation and rotation both around the origin, extrinsically. |
| `.transformBy(other: Transform3d): Pose3d` | Applies a relative/intrinsic transform expressed in this pose's own frame. |
| `.relativeTo(other: Pose3d): Pose3d` | This pose re-expressed relative to `other`. |
| `.rotateAround(point: Translation3d, rotation: Rotation3d): Pose3d` | Rotates this pose around an arbitrary global-frame point. |
| `.toPose2d(): Pose2d` | Projected into the X-Y plane. |
| `.nearest(poses: Collection<Pose3d>): Pose3d?` | Closest by translation distance, ties broken by rotation angle. |
| `.interpolate(endValue: Pose3d, t: Double): Pose3d` | Interpolates translation and rotation **independently** (lerp + slerp) — unlike `Pose2d.interpolate`'s twist-based approach, since WPILib's SE(3) pose-exponential has no portable pure-Java reference to port (see `Twist3d`). |
| `Pose3d.kZero` | |

`Transform2d.kt` — a relative transformation (translation + rotation) applied to a `Pose2d` via
`transformBy`/`+`. Unlike `Pose2d`, it's relative: its translation is expressed in the *starting*
pose's rotated frame, not the field frame:

| Symbol | Description |
| --- | --- |
| `Transform2d(translation: Translation2d = kZero, rotation: Rotation2d = kZero)` | |
| `Transform2d(x: Double, y: Double, rotation: Rotation2d)` | |
| `Transform2d(initial: Pose2d, last: Pose2d)` | The relative transform that carries `initial` to `last`. |
| `.x` / `.y: Double` | |
| `.inverse(): Transform2d` | |
| `+ Transform2d` | Composes two transforms. |
| `* Double` / `/ Double` | |

`Transform3d.kt` — the 3D counterpart, applied intrinsically (relative to the starting pose's own
frame) via `Pose3d.transformBy`/`+`:

| Symbol | Description |
| --- | --- |
| `Transform3d(translation: Translation3d = kZero, rotation: Rotation3d = kZero)` | |
| `Transform3d(x: Double, y: Double, z: Double, rotation: Rotation3d)` | |
| `Transform3d(initial: Pose3d, last: Pose3d)` | The transform that carries `initial` to `last`. |
| `Transform3d(transform: Transform2d)` | From a 2D transform in the X-Y plane. |
| `.x` / `.y` / `.z: Double` | |
| `.times(scalar: Double)` / `.div(scalar: Double): Transform3d` | **Not operators**, unlike `Transform2d`. |
| `+ Transform3d` | Composes two transforms. |
| `.inverse(): Transform3d` | |
| `Transform3d.kZero` | |

`Twist2d.kt`

| Symbol | Description |
| --- | --- |
| `data class Twist2d(val dx: Double = 0.0, val dy: Double = 0.0, val dtheta: Double = 0.0)` | A small planar movement along a constant-curvature arc: `dx`/`dy` meters forward/sideways, `dtheta` radians of rotation. What `Pose2d.exp`/`.log` integrate to/from — odometry accumulates one of these per loop instead of naively adding `(dx, dy, dtheta)` onto the pose, which keeps odometry accurate through a turn instead of only at the sampled instants. |

`Twist3d.kt`

| Symbol | Description |
| --- | --- |
| `data class Twist3d(val dx: Double = 0.0, val dy: Double = 0.0, val dz: Double = 0.0, val rx: Double = 0.0, val ry: Double = 0.0, val rz: Double = 0.0)` | Linear motion (`dx`/`dy`/`dz`, meters) plus an axis-angle rotation vector (`rx`/`ry`/`rz`, radians). Unlike `Twist2d`, **not** wired to a constant-curvature `Pose3d.exp`/`.log` pair — upstream WPILib's SE(3) pose-exponential is a native (JNI) helper with no portable pure-Java reference to port, so `Pose3d.interpolate` interpolates translation/rotation independently instead. This type exists for API parity and for code that just wants to carry `(dx, dy, dz, rx, ry, rz)` around. |

`CoordinateAxis.kt` — one axis of a coordinate system, always expressed as a normalized direction
within this library's own North-West-Up (NWU) frame (the convention every other type on this page
uses natively: +X north, +Y west, +Z up):

| Symbol | Description |
| --- | --- |
| `CoordinateAxis(x: Double, y: Double, z: Double)` | A custom axis, from a raw (not-necessarily-normalized) direction in the NWU frame. |
| `CoordinateAxis.N() / .S() / .E() / .W() / .U() / .D()` | The six cardinal directions in the NWU frame (+X/-X/-Y/+Y/+Z/-Z respectively), as pre-built singletons — the building blocks for `CoordinateSystem`'s companion factories. |

`CoordinateSystem.kt` — converts `Translation3d`/`Rotation3d`/`Pose3d`/`Transform3d` between
different axis conventions. Exists for interfacing with something (a sensor, an external library,
a dataset) that reports in a different handedness/axis convention than this library's native NWU —
day-to-day AlonLib code doesn't need it, since everything already speaks NWU:

| Symbol | Description |
| --- | --- |
| `CoordinateSystem(positiveX: CoordinateAxis, positiveY: CoordinateAxis, positiveZ: CoordinateAxis)` | Defines a system by its three basis axes, each expressed in NWU. Logs via `robotPrintError` (doesn't throw) if the three axes don't form a right-handed system. |
| `CoordinateSystem.NWU()` | North-West-Up — this library's own convention, the identity conversion. |
| `CoordinateSystem.EDN()` | East-Down-North. |
| `CoordinateSystem.NED()` | North-East-Down. |
| `CoordinateSystem.convert(translation: Translation3d, from: CoordinateSystem, to: CoordinateSystem): Translation3d` | |
| `CoordinateSystem.convert(rotation: Rotation3d, from: CoordinateSystem, to: CoordinateSystem): Rotation3d` | |
| `CoordinateSystem.convert(pose: Pose3d, from: CoordinateSystem, to: CoordinateSystem): Pose3d` | |
| `CoordinateSystem.convert(transform: Transform3d, from: CoordinateSystem, to: CoordinateSystem): Transform3d` | |

`Geometry2dTo3d.kt` — extension-function bridges from the 2D geometry types to their 3D
counterparts (`z = 0`, pure yaw), complementing the `Xyz3d(xyz2d)` constructors those 3D types
already expose directly:

| Symbol | Description |
| --- | --- |
| `Rotation2d.toRotation3d(): Rotation3d` | |
| `Translation2d.toTranslation3d(): Translation3d` | |
| `Transform2d.toTransform3d(): Transform3d` | |
| `Pose2d.toPose3d(): Pose3d` | |

### `math/interpolation/`

Generic interpolation building blocks — used by the geometry types above (most implement
`Interpolatable`), by lookup tables built at runtime, and by `TimeInterpolatableBuffer` for
latency-compensated pose history.

`Interpolatable.kt`

| Symbol | Description |
| --- | --- |
| `interface Interpolatable<T> { fun interpolate(endValue: T, t: Double): T }` | Implemented by `Rotation2d`/`Rotation3d`/`Pose2d`/`Pose3d`/`Translation2d`/`Translation3d` and others (e.g. kinematics wheel-position types) so they can be dropped straight into an `InterpolatingTreeMap` or `TimeInterpolatableBuffer` without a separate `Interpolator`. `t = 0` returns this value, `t = 1` returns `endValue`. |

`Interpolator.kt`

| Symbol | Description |
| --- | --- |
| `fun interface Interpolator<T> { fun interpolate(startValue: T, endValue: T, t: Double): T }` | Interpolates between two `T`s, for types that don't implement `Interpolatable` themselves (e.g. plain `Double`, or a third-party type). |
| `Interpolator.forDouble: Interpolator<Double>` | Backed by `math.interpolate` (unclamped lerp). |
| `Interpolator.forInterpolatable<T : Interpolatable<T>>(): Interpolator<T>` | Delegates to the type's own `Interpolatable.interpolate`. |

`InverseInterpolator.kt` — the inverse operation: given two bracketing values and a query value,
find how far along the query sits:

| Symbol | Description |
| --- | --- |
| `fun interface InverseInterpolator<T> { fun inverseInterpolate(startValue: T, endValue: T, q: T): Double }` | Returns the fraction `t` in `[0, 1]` at which `q` sits between `startValue` and `endValue`. Used by `InterpolatingTreeMap` to place a lookup key between its two bracketing entries. |
| `InverseInterpolator.forDouble: InverseInterpolator<Double>` | Clamped to `[0, 1]`; returns `0.0` if the `[startValue, endValue]` range is empty or inverted (`endValue <= startValue`) instead of dividing by a non-positive range. |

`InterpolatingTreeMap.kt` — a sorted, mutable lookup table that interpolates between its two
nearest entries for a key without an exact match. Unlike `math.LinearInterpolationTable` (fixed at
construction, `Double -> Double` only), this can be `put` into at any time and works with any
`K`/`V` pair that has interpolators for it (e.g. a `Rotation2d`-keyed table of `Pose2d`s):

| Symbol | Description |
| --- | --- |
| `open class InterpolatingTreeMap<K : Comparable<K>, V>(keyInterpolator: InverseInterpolator<K>, valueInterpolator: Interpolator<V>)` | Backed by a `java.util.TreeMap<K, V>`. |
| `.put(key: K, value: V)` | Inserts/overwrites. |
| `.get(key: K): V?` | Exact match returns that value directly; otherwise interpolates between the nearest surrounding entries via `keyInterpolator`/`valueInterpolator`. A key outside the table's range clamps to the nearest edge value instead of extrapolating. Returns `null` only if the table is empty. |
| `.clear()` | |
| `.size: Int`, `.isEmpty: Boolean` | |

`InterpolatingDoubleTreeMap.kt`

| Symbol | Description |
| --- | --- |
| `class InterpolatingDoubleTreeMap : InterpolatingTreeMap<Double, Double>` | Pre-wired with `InverseInterpolator.forDouble`/`Interpolator.forDouble`, for the common case of a `Double -> Double` lookup table built up at runtime (e.g. shooter RPM by measured distance). |

`TimeInterpolatableBuffer.kt` — a rolling window of timestamped samples, for estimating a past
value (e.g. a robot pose) at an arbitrary time within that window. This is what a pose estimator
uses for latency-compensated vision fusion: odometry poses are recorded into the buffer every
loop, and when a vision measurement arrives late (camera capture-to-processing latency), the
buffer reconstructs the odometry pose at the moment the image was actually captured — instead of
naively fusing the measurement against the *current* pose — so the correction can be applied at
the right point in the pose history and replayed forward:

| Symbol | Description |
| --- | --- |
| `TimeInterpolatableBuffer<T>(interpolator: Interpolator<T>, historySizeSeconds: Double)` | |
| `.internalBuffer: TreeMap<Double, T>` | Raw backing map, sorted by timestamp — exposed for replaying samples in order. |
| `.addSample(timeSeconds: Double, sample: T)` | Records `sample`, evicting entries older than `historySizeSeconds` relative to `timeSeconds`. |
| `.getSample(timeSeconds: Double): T?` | Exact match, or interpolated between the nearest surrounding samples; clamps to the nearest edge sample outside the buffer's time range. `null` only if the buffer is empty. |
| `.clear()` | |
| `TimeInterpolatableBuffer.createBuffer<T : Interpolatable<T>>(historySizeSeconds: Double): TimeInterpolatableBuffer<T>` | Convenience factory using `Interpolator.forInterpolatable<T>()`. |

### `math/kinematics/`

A WPILib-style kinematics/odometry layer: converts between a chassis-level [`ChassisSpeeds`](#chassisspeedskt)
and each drivetrain shape's own wheel speeds/positions, and tracks field pose from those readings
over time. Two odometry families live here side by side: gyro-driven [`Odometry`](#odometrykt) (this
package's `DifferentialDriveOdometry`/`MecanumDriveOdometry`/`SwerveDriveOdometry`, matching
upstream WPILib) and gyro-free dead-wheel odometry ([`DeadWheelOdometryBase`](#deadwheelodometrybasekt)'s
`DifferentialOdometry`/`HolonomicOdometry`, ported from SolversLib). All distances/speeds are in
meters/meters-per-second unless noted otherwise.

#### `ChassisSpeeds.kt`

A robot chassis's velocity: `vx`/`vy` in meters/sec, `omega` in radians/sec.

| Symbol | Description |
| --- | --- |
| `ChassisSpeeds(vx: Double = 0.0, vy: Double = 0.0, omega: Double = 0.0)` | Mutable `var` fields. Similar shape to `Twist2d` but a different meaning — a `Twist2d` is a pose *delta*, this is a *velocity*. A non-holonomic (differential) drivetrain should never have nonzero `vy`; a holonomic one (mecanum, swerve) usually has all three. |
| `.toTwist2d(dtSeconds: Double): Twist2d` | `Twist2d(vx*dt, vy*dt, omega*dt)`. |
| `+`, `-`, unary `-`, `* Double`, `/ Double` | Componentwise arithmetic, returns a new `ChassisSpeeds`. |
| `equals`/`hashCode`/`toString()` | Value semantics; `"ChassisSpeeds(vx=.. m/s, vy=.. m/s, omega=.. rad/s)"`. |
| `ChassisSpeeds.discretize(vx, vy, omega, dtSeconds): ChassisSpeeds` (companion) | Converts continuous-time speeds into the discrete-time speeds that, applied for one `dtSeconds` step, move the robot exactly `vx*dt`/`vy*dt`/`omega*dt` — compensates for the translational skew a holonomic drivetrain gets from translating and rotating at once. Scaling the result down afterwards (e.g. desaturating swerve module speeds) reintroduces a skew this doesn't account for. |
| `ChassisSpeeds.discretize(continuousSpeeds: ChassisSpeeds, dtSeconds: Double): ChassisSpeeds` | Same, taking a `ChassisSpeeds`. |
| `ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotAngle: Rotation2d): ChassisSpeeds` | Field-relative speeds (facing `robotAngle`) → robot-relative. |
| `ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds: ChassisSpeeds, robotAngle: Rotation2d): ChassisSpeeds` | Same, taking a `ChassisSpeeds`. |
| `ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, robotAngle: Rotation2d): ChassisSpeeds` | Robot-relative speeds → field-relative. |
| `ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds: ChassisSpeeds, robotAngle: Rotation2d): ChassisSpeeds` | Same, taking a `ChassisSpeeds`. |

#### `Kinematics.kt`

`interface Kinematics<WheelSpeeds, WheelPositions>` — converts between `ChassisSpeeds` and a
drivetrain's own wheel-speed/wheel-position shape (left/right for differential, four corners for
mecanum/swerve). Implemented by `DifferentialDriveKinematics`, `MecanumDriveKinematics`,
`SwerveDriveKinematics`; backs the generic `Odometry` base class.

| Symbol | Description |
| --- | --- |
| `toChassisSpeeds(wheelSpeeds: WheelSpeeds): ChassisSpeeds` | Forward kinematics. |
| `toWheelSpeeds(chassisSpeeds: ChassisSpeeds): WheelSpeeds` | Inverse kinematics. |
| `toTwist2d(start: WheelPositions, end: WheelPositions): Twist2d` | The pose delta between `start`/`end`'s wheel positions, for one `Odometry.update` tick. |
| `interpolate(startValue: WheelPositions, endValue: WheelPositions, t: Double): WheelPositions` | Interpolates wheel positions, used when sampling between odometry updates. |

#### `Odometry.kt`

`open class Odometry<WheelPositions>` — tracks a robot's field pose over time from a gyro angle
plus wheel encoder readings, given a `Kinematics<*, WheelPositions>`. Use one of the
drivetrain-specific subclasses (`DifferentialDriveOdometry`, `MecanumDriveOdometry`,
`SwerveDriveOdometry`) rather than this directly. **The gyro angle is trusted over the
kinematics-derived rotation** (encoders alone can't distinguish wheel slip from an actual turn),
while translation comes from the kinematics.

| Symbol | Description |
| --- | --- |
| `Odometry(kinematics: Kinematics<*, WheelPositions>, gyroAngle: Rotation2d, wheelPositions: WheelPositions, initialPose: Pose2d = Pose2d.kZero)` | |
| `.pose: Pose2d` (read-only) | The current tracked field pose. |
| `.resetPosition(gyroAngle: Rotation2d, wheelPositions: WheelPositions, pose: Pose2d)` | Resets the tracked pose and the encoder/gyro baselines it's measured from. |
| `.resetPose(pose: Pose2d)` | Resets the pose, adjusting the gyro offset so future gyro readings stay consistent. |
| `.resetTranslation(translation: Translation2d)` | Overwrites just the translation component of `pose`. |
| `.resetRotation(rotation: Rotation2d)` | Overwrites just the rotation component of `pose`, adjusting the gyro offset. |
| `.update(gyroAngle: Rotation2d, wheelPositions: WheelPositions): Pose2d` | Integrates the latest reading into `pose` (kinematics-derived translation + trusted gyro rotation) and returns it. |

#### `DeadWheelOdometryBase.kt`

`abstract class DeadWheelOdometryBase(initialPose: Pose2d, val trackWidth: Double = 18.0)` — base
for dead-wheel-only odometry (`DifferentialOdometry`, `HolonomicOdometry`) that computes heading
purely from encoder deltas, with no gyro input. Distinct from `Odometry` (which *is* gyro-driven,
matching WPILib's per-wheel-kinematics odometry) — this is SolversLib's simpler
dead-wheel-pod-oriented design, kept under its own name to avoid colliding with `Odometry`.
`trackWidth` defaults to `18.0` (not meters-assumed — pass whatever unit your encoder deltas are
already in; it just needs to match).

| Symbol | Description |
| --- | --- |
| `.pose: Pose2d` (read-only outside subclasses) | The current tracked field pose. |
| `abstract .updatePose()` | Recomputes `pose` from the latest live sensor readings (the constructor lambdas of subclasses). |
| `abstract .updatePose(newPose: Pose2d)` | Resets `pose` outright and clears the subclass's running encoder deltas. |
| `.rotatePose(byRadians: Double)` | Offsets `pose`'s heading by `byRadians` without moving its translation. |

#### `DifferentialOdometry.kt`

`class DifferentialOdometry(trackWidth: Double, initialPose: Pose2d = Pose2d(), left: (() -> Double)? = null, right: (() -> Double)? = null) : DeadWheelOdometryBase` —
dead-wheel odometry for a two-encoder (left/right) setup, no gyro; heading is derived purely from
the left/right encoder delta and `trackWidth`. Pass `left`/`right` to have `updatePose()` (no args)
pull live readings itself each loop, or leave them unset and drive `updatePosition` directly with
your own readings.

| Symbol | Description |
| --- | --- |
| `.updatePose(newPose: Pose2d)` | Resets `pose` and zeroes the running left/right encoder baselines. |
| `.updatePose()` | Pulls fresh readings from the constructor's `left`/`right` lambdas and updates `pose`; no-op if either wasn't supplied. |
| `.updatePosition(leftEncoderPos: Double, rightEncoderPos: Double): Pose2d` | Integrates one left/right reading pair into `pose` and returns it. |

#### `HolonomicOdometry.kt`

`class HolonomicOdometry(trackWidth: Double, centerWheelOffset: Double, initialPose: Pose2d = Pose2d(), left: (() -> Double)? = null, right: (() -> Double)? = null, horizontal: (() -> Double)? = null) : DeadWheelOdometryBase` —
dead-wheel odometry for a three-dead-wheel (left/right/horizontal) setup, no gyro; heading comes
from the left/right delta and `trackWidth`, and the horizontal (strafe) wheel is corrected for
`centerWheelOffset` (its distance from the robot's rotation center) so rotation alone isn't
misread as sideways drift. Pass `left`/`right`/`horizontal` to have `updatePose()` (no args) pull
live readings itself each loop, or leave them unset and drive `update` directly.

| Symbol | Description |
| --- | --- |
| `.updatePose(newPose: Pose2d)` | Resets `pose` and zeroes the running left/right/horizontal encoder baselines. |
| `.updatePose()` | Pulls fresh readings from the constructor's `left`/`right`/`horizontal` lambdas and updates `pose`; no-op if any is unset. |
| `.update(leftEncoderPos: Double, rightEncoderPos: Double, horizontalEncoderPos: Double): Pose2d` | Integrates one left/right/horizontal reading triple into `pose` and returns it. |

#### `DifferentialDriveKinematics.kt` / `DifferentialDriveWheelSpeeds.kt` / `DifferentialDriveWheelPositions.kt` / `DifferentialDriveOdometry.kt`

`DifferentialDriveKinematics(val trackWidthMeters: Double) : Kinematics<DifferentialDriveWheelSpeeds, DifferentialDriveWheelPositions>` —
converts between `ChassisSpeeds` and per-side wheel speeds/positions for a tank/differential drive.

| Symbol | Description |
| --- | --- |
| `.toChassisSpeeds(wheelSpeeds: DifferentialDriveWheelSpeeds): ChassisSpeeds` | `vx = (left+right)/2`, `vy = 0`, `omega = (right-left)/trackWidthMeters`. |
| `.toWheelSpeeds(chassisSpeeds: ChassisSpeeds): DifferentialDriveWheelSpeeds` | `left = vx - trackWidth/2*omega`, `right = vx + trackWidth/2*omega`. |
| `.toTwist2d(start, end): Twist2d` | Forward kinematics from a `DifferentialDriveWheelPositions` delta. |
| `.toTwist2d(leftDistanceMeters: Double, rightDistanceMeters: Double): Twist2d` | Forward kinematics from per-side distance deltas directly, for odometry. |
| `.interpolate(startValue, endValue, t: Double): DifferentialDriveWheelPositions` | Delegates to `DifferentialDriveWheelPositions.interpolate`. |
| `data class DifferentialDriveWheelPositions(val left: Double = 0.0, val right: Double = 0.0) : Interpolatable<...>` | Cumulative per-side encoder distance, meters. |
| `class DifferentialDriveWheelSpeeds(var left: Double = 0.0, var right: Double = 0.0)` | Per-side wheel speed, meters/second. `.desaturate(attainableMaxSpeed: Double)` scales both down (preserving ratio) if either exceeds it in magnitude. `+`, `-`, unary `-`, `* Double`, `/ Double`, `equals`/`hashCode`/`toString()`. |
| `class DifferentialDriveOdometry(gyroAngle: Rotation2d, leftDistanceMeters: Double, rightDistanceMeters: Double, initialPose: Pose2d = Pose2d.kZero) : Odometry<DifferentialDriveWheelPositions>` | Tracks field pose from a gyro angle plus left/right encoder distances. **Zero both encoders before constructing** (or before any subsequent `resetPosition` call). `.resetPosition(gyroAngle, leftDistanceMeters, rightDistanceMeters, pose)` / `.update(gyroAngle, leftDistanceMeters, rightDistanceMeters)` are per-side convenience overloads of the `Odometry` base's `WheelPositions`-typed methods. |

#### `MecanumDriveKinematics.kt` / `MecanumDriveWheelSpeeds.kt` / `MecanumDriveWheelPositions.kt` / `MecanumDriveMotorVoltages.kt` / `MecanumDriveOdometry.kt`

`MecanumDriveKinematics(val frontLeftWheel: Translation2d, val frontRightWheel: Translation2d, val rearLeftWheel: Translation2d, val rearRightWheel: Translation2d) : Kinematics<MecanumDriveWheelSpeeds, MecanumDriveWheelPositions>` —
converts between `ChassisSpeeds` and four-wheel speeds/positions given each wheel's location
relative to the robot center. Inverse kinematics (chassis → wheels) is a matrix multiply; forward
kinematics (wheels → chassis) is overdetermined (4 equations, 3 unknowns), solved via the
Moore-Penrose `Matrix.pseudoInverse` for a least-squares fit.

| Symbol | Description |
| --- | --- |
| `.toWheelSpeeds(chassisSpeeds: ChassisSpeeds, centerOfRotation: Translation2d): MecanumDriveWheelSpeeds` | Inverse kinematics pivoting around `centerOfRotation` instead of the physical center (e.g. a robot corner). Recomputes the inverse-kinematics matrix only when `centerOfRotation` changes. |
| `.toWheelSpeeds(chassisSpeeds: ChassisSpeeds): MecanumDriveWheelSpeeds` | Same, with `centerOfRotation = Translation2d.kZero`. |
| `.toChassisSpeeds(wheelSpeeds: MecanumDriveWheelSpeeds): ChassisSpeeds` | Least-squares forward kinematics. |
| `.toTwist2d(start, end): Twist2d` | Forward kinematics from a `MecanumDriveWheelPositions` delta. |
| `.toTwist2d(wheelDeltas: MecanumDriveWheelPositions): Twist2d` | Forward kinematics from per-wheel distance deltas directly, for odometry. |
| `.interpolate(...)` | Delegates to `MecanumDriveWheelPositions.interpolate`. |
| `data class MecanumDriveWheelPositions(frontLeft, frontRight, rearLeft, rearRight: Double = 0.0) : Interpolatable<...>` | Cumulative per-wheel encoder distance, meters. |
| `class MecanumDriveWheelSpeeds(var frontLeft, frontRight, rearLeft, rearRight: Double = 0.0)` | Per-wheel speed, meters/second. `.desaturate(attainableMaxSpeed: Double)` scales all four down (preserving ratios) if any exceeds it. `+`, `-`, unary `-`, `* Double`, `/ Double`, `equals`/`hashCode`/`toString()`. |
| `data class MecanumDriveMotorVoltages(frontLeft, frontRight, rearLeft, rearRight: Double = 0.0)` | Plain voltage-per-wheel bundle, volts. |
| `class MecanumDriveOdometry(kinematics: MecanumDriveKinematics, gyroAngle: Rotation2d, wheelPositions: MecanumDriveWheelPositions, initialPose: Pose2d = Pose2d.kZero) : Odometry<MecanumDriveWheelPositions>` | Tracks field pose from a gyro angle plus four wheel encoder distances — a thin `Odometry` specialization with no extra methods of its own. |

#### `MecanumOdoKinematics.kt` / `OdoWheelSpeeds.kt`

`MecanumOdoKinematics(frontLeftWheel: Translation2d, frontRightWheel: Translation2d, rearLeftWheel: Translation2d, rearRightWheel: Translation2d, auxDistance: Double, wheelbaseWidth: Double)` —
inverse kinematics from `ChassisSpeeds` to four mecanum wheel speeds, like
`MecanumDriveKinematics`, but its matching **forward** kinematics reads from dead-wheel
`OdoWheelSpeeds` (left/right/center) instead of the four drive wheels — for robots whose odometry
pods are physically separate from their drive wheels. `auxDistance` is the center (strafe) pod's
offset from the rotation center.

| Symbol | Description |
| --- | --- |
| `.toWheelSpeeds(chassisSpeeds: ChassisSpeeds, centerOfRotation: Translation2d = Translation2d.kZero): MecanumDriveWheelSpeeds` | Inverse kinematics, optionally pivoting around a non-center point. |
| `.toChassisSpeeds(wheelSpeeds: OdoWheelSpeeds): ChassisSpeeds` | Forward kinematics from the dead-wheel pods: `omega = (right-left)/(wheelbaseWidth)`, `vy` corrected by `auxDistance * omega`. |
| `class OdoWheelSpeeds(var left, right, center: Double = 0.0)` | Dead-wheel speeds for a `MecanumOdoKinematics` setup, meters/second. `.normalize(attainableMaxSpeed: Double)` scales all three down (preserving ratios, based on `max(abs(left), abs(right))`) if that exceeds it. `toString()` only (no arithmetic operators, unlike the other `*WheelSpeeds` types). |

#### `SwerveDriveKinematics.kt` / `SwerveModuleState.kt` / `SwerveModulePosition.kt` / `SwerveDriveOdometry.kt`

`SwerveDriveKinematics(vararg val moduleTranslations: Translation2d) : Kinematics<Array<SwerveModuleState>, Array<SwerveModulePosition>>` —
converts between `ChassisSpeeds` and per-module states/positions for an arbitrary number (2+) of
swerve modules at arbitrary locations, via the same pseudoinverse-based least-squares approach as
`MecanumDriveKinematics`. **Module order is whatever order `moduleTranslations` were passed in** —
every array this class takes or returns (states, positions, headings) must use that same order.
Fewer than 2 modules logs via `robotPrintError` (constructs anyway).

| Symbol | Description |
| --- | --- |
| `.resetHeadings(vararg headings: Rotation2d)` | Sets the module headings used when a zero chassis speed is commanded (modules then hold their last angle instead of snapping to 0°). Logs and no-ops if the count doesn't match the module count. |
| `.toSwerveModuleStates(chassisSpeeds: ChassisSpeeds, centerOfRotation: Translation2d = Translation2d.kZero): Array<SwerveModuleState>` | Inverse kinematics. A chassis speed of exactly zero returns every module at zero speed holding its last-commanded heading, rather than snapping to 0°. Recomputes the inverse matrix only when `centerOfRotation` changes. |
| `.toWheelSpeeds(chassisSpeeds: ChassisSpeeds): Array<SwerveModuleState>` | `Kinematics` override, `= toSwerveModuleStates(chassisSpeeds)`. |
| `.toChassisSpeeds(wheelSpeeds: Array<SwerveModuleState>): ChassisSpeeds` | Least-squares forward kinematics. Logs and returns a zero `ChassisSpeeds` if the array length doesn't match the module count. |
| `.toTwist2d(vararg moduleDeltas: SwerveModulePosition): Twist2d` | Forward kinematics from per-module distance deltas directly, for odometry. Same length-mismatch guard. |
| `.toTwist2d(start, end): Twist2d` | `Kinematics` override, diffs two `Array<SwerveModulePosition>` and delegates to the vararg overload. |
| `.interpolate(startValue, endValue, t: Double): Array<SwerveModulePosition>` | Per-module `SwerveModulePosition.interpolate`. Logs and returns `startValue` unchanged on a length mismatch. |
| `SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates: Array<SwerveModuleState>, attainableMaxSpeed: Double)` (companion) | Scales every module's speed down (preserving ratios) if any exceeds `attainableMaxSpeed`; mutates `moduleStates` in place. |
| `SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, desiredChassisSpeed: ChassisSpeeds, attainableMaxModuleSpeed: Double, attainableMaxTranslationalSpeed: Double, attainableMaxRotationalVelocity: Double)` (companion) | Like the two-arg overload, but backs off translation/rotation together (not just per-module) so the commanded motion's *shape* is preserved — avoids joystick-edge saturation distorting the direction of travel. Mutates in place. |
| `class SwerveModuleState(var speedMetersPerSecond: Double = 0.0, var angle: Rotation2d = Rotation2d.kZero) : Comparable<SwerveModuleState>` | One module's commanded/measured speed + steering angle. `.optimize(currentAngle: Rotation2d)`: minimizes steering change to reach `angle` by driving backwards instead if the delta exceeds 90° — a module never turns more than 90°. `.cosineScale(currentAngle: Rotation2d)`: scales `speedMetersPerSecond` by `cos(angle - currentAngle)` so a module still turning towards its target doesn't drive perpendicular to where it should. `compareTo` orders by speed. Companion `SwerveModuleState.optimize(desiredState, currentAngle): SwerveModuleState` is the pure (non-mutating) form. |
| `class SwerveModulePosition(var distanceMeters: Double = 0.0, var angle: Rotation2d = Rotation2d.kZero) : Comparable<...>, Interpolatable<...>` | One module's cumulative encoder distance + steering angle. `.copy()`, `.interpolate(endValue, t)`, `compareTo` by distance, value `equals`/`hashCode`/`toString()`. |
| `class SwerveDriveOdometry(kinematics: SwerveDriveKinematics, gyroAngle: Rotation2d, modulePositions: Array<SwerveModulePosition>, initialPose: Pose2d = Pose2d.kZero) : Odometry<Array<SwerveModulePosition>>` | Tracks field pose from a gyro angle plus each module's encoder distance + angle — a thin `Odometry` specialization. |

### `math/spline/`

Parametric 2D splines (`t` from 0 to 1) used to generate smooth paths through waypoints, and the
machinery ([`SplineParameterizer`](#splineparameterizerkt)) that breaks a spline into
near-constant-curvature arc segments for [`TrajectoryGenerator`](#trajectorygeneratorkt). Ported
from WPILib's own spline math (itself derived from FTC team 254's spline library).

#### `Spline.kt`

`abstract class Spline(private val degree: Int)` — base for `CubicHermiteSpline`
(degree 3)/`QuinticHermiteSpline` (degree 5).

| Symbol | Description |
| --- | --- |
| `abstract .coefficients(): Matrix` | Row 0 = x coefficients, row 1 = y, rows 2-3 = their 1st derivatives, rows 4-5 = 2nd derivatives. |
| `abstract .initialControlVector() / .finalControlVector(): ControlVector` | The control vectors the spline was built from. |
| `.getPoint(t: Double): PoseWithCurvature?` | The pose and curvature at `t` (0 = start, 1 = end); `null` where the spline's velocity is ~zero (heading undefined) — check for this rather than assuming every `t` yields a point. |
| `class ControlVector(x: DoubleArray, y: DoubleArray)` | A control point for a spline: `x`/`y` are each `[position, velocity, ...]` — the value of each successive derivative at that end of the spline. Arrays are defensively copied. |

#### `CubicHermiteSpline.kt`

`class CubicHermiteSpline(xInitialControlVector, xFinalControlVector, yInitialControlVector, yFinalControlVector: DoubleArray) : Spline(3)` —
interpolates between two points given each one's position and velocity (1st derivative). Cheaper
to evaluate than `QuinticHermiteSpline` but can't also match acceleration/curvature at the
endpoints. Each control vector must have `size >= 2`; shorter arrays log via `robotPrintError` and
fall back to a zeroed control-vector matrix.

#### `QuinticHermiteSpline.kt`

`class QuinticHermiteSpline(xInitialControlVector, xFinalControlVector, yInitialControlVector, yFinalControlVector: DoubleArray) : Spline(5)` —
interpolates between two points given each one's position, velocity, and acceleration (1st and
2nd derivatives). Costs more to evaluate than `CubicHermiteSpline` but produces continuous
curvature, which matters for smooth trajectory tracking. Each control vector must have exactly
`size == 3`; a wrong size logs via `robotPrintError` and falls back to a zeroed control-vector
matrix.

#### `PoseWithCurvature.kt`

| Symbol | Description |
| --- | --- |
| `data class PoseWithCurvature(val pose: Pose2d = Pose2d.kZero, val curvatureRadPerMeter: Double = 0.0)` | A pose sampled from a `Spline`, paired with the spline's curvature at that point. |

#### `SplineHelper.kt`

`object SplineHelper` — builds `CubicHermiteSpline`s/`QuinticHermiteSpline`s from waypoints or
control vectors.

| Symbol | Description |
| --- | --- |
| `.getCubicControlVectorsFromWaypoints(start: Pose2d, interiorWaypoints: Array<Translation2d>, end: Pose2d): Array<Spline.ControlVector>` | The 2 cubic control vectors (endpoints only) for a path through `start`/`interiorWaypoints`/`end`; auto-picks a control-vector magnitude (1.2× the distance to the nearest interior waypoint, or to the other endpoint if there are none) that "looks good". |
| `.getQuinticSplinesFromWaypoints(waypoints: List<Pose2d>): Array<QuinticHermiteSpline>` | One quintic spline per consecutive waypoint pair, each control vector's magnitude auto-picked the same way. |
| `.getCubicSplinesFromControlVectors(start: Spline.ControlVector, waypoints: Array<Translation2d>, end: Spline.ControlVector): Array<CubicHermiteSpline>` | Cubic splines through `start`/`waypoints`/`end`, choosing the interior waypoints' headings automatically for continuous curvature by solving a tridiagonal system (Thomas algorithm). |
| `.getQuinticSplinesFromControlVectors(controlVectors: Array<Spline.ControlVector>): Array<QuinticHermiteSpline>` | One quintic spline per consecutive control-vector pair (control vectors are fully specified, so no heading-solving is needed). |
| `.optimizeCurvature(splines: Array<QuinticHermiteSpline>): Array<QuinticHermiteSpline>` | Nudges each shared knot point's curvature towards a weighted average across the adjoining splines, minimizing the integral of the second derivative's absolute value across the whole path (Sprunk 2008, §4.1.2). No-ops for fewer than 2 splines. |

#### `SplineParameterizer.kt`

`object SplineParameterizer` — breaks a `Spline` into a sequence of `PoseWithCurvature` samples
close enough together (recursive bisection) that a trajectory generator can treat each segment as
a constant-radius arc. Ported from FTC team 254's spline parameterizer (also what upstream
WPILib's is based on).

| Symbol | Description |
| --- | --- |
| `.parameterize(spline: Spline, t0: Double = 0.0, t1: Double = 1.0): List<PoseWithCurvature>` | Bisects `[t0, t1]` until each segment's pose delta (via `Pose2d.log`) is within `MAX_DX` (0.127 m), `MAX_DY` (0.00127 m), and `MAX_DTHETA` (0.0872 rad ≈ 5°). |
| `class MalformedSplineException(message: String) : RuntimeException` | Thrown if `spline.getPoint(t)` returns `null` anywhere sampled (near-zero velocity — usually two adjacent waypoints very close together with opposing headings), or if bisection exceeds `MAX_ITERATIONS` (5000) without converging. |

### `math/system/`

State-space control plumbing: matrices, plant models, discretization, and the linear-quadratic
regulator/Kalman-filter loop, ported from WPILib. Where upstream WPILib calls into a JNI-backed
Eigen/Drake solver (matrix exponential, DARE), this is reimplemented in pure Kotlin/EJML instead,
since JNI isn't portable to Android — see the per-file notes below for what that trades off.

#### `Matrix.kt`

`class Matrix private constructor(...)` — a dense, real-valued matrix wrapping EJML's
`SimpleMatrix`, backing the ported LQR/Kalman-filter/`LinearSystem` code and this library's own
kinematics matrix math. Chosen over porting WPILib's own compile-time-dimension-checked
`Matrix<Rows, Cols>` (a `Nat<N>` phantom-type system with ~20 generated classes) as not worth the
weight for how little of this codebase does state-space control — dimensions are checked at
runtime instead, logging via `robotPrintError` and returning a same-shape zero/unchanged matrix on
mismatch rather than throwing.

| Symbol | Description |
| --- | --- |
| `Matrix(rows: Int, cols: Int)` / `Matrix(data: Array<DoubleArray>)` | Constructors: zero-filled, or from row-major data. |
| `.rows` / `.cols: Int` | Dimensions. |
| `[row, col]` get/set | Element access. |
| `.setRow(row: Int, vararg values: Double)` | Overwrites a row in place; logs and no-ops on a size mismatch. |
| `.fill(value: Double)` | Overwrites every element in place. |
| `.assignBlock(startRow: Int, startCol: Int, other: Matrix)` | Copies `other` into this matrix in place at the given offset. |
| `.block(numRows: Int, numCols: Int, startRow: Int, startCol: Int): Matrix` | Extracts a submatrix. |
| `.maxAbs(): Double` | Largest absolute value of any element. |
| `+`, `-`, unary `-`, `*` (matrix), `* Double`, `/ Double` | Arithmetic; `+`/`-`/`*` (matrix) log and return an unchanged operand on a dimension mismatch instead of throwing. |
| `.elementTimes(other: Matrix) / .elementDiv(other: Matrix): Matrix` | Elementwise (Hadamard) multiply/divide. |
| `.transpose() / .copy(): Matrix` | |
| `.setColumn(col: Int, column: Matrix)` / `.column(col: Int): Matrix` | Column access as an n×1 matrix. |
| `.cholesky(): Matrix` | Lower-triangular Cholesky factor `L` (`L·Lᵀ = this`). Requires symmetric positive-*semi*definite (treats a ~zero pivot as a degenerate/dependent direction rather than requiring strict positive-definiteness); logs and returns a zero matrix if a diagonal term goes meaningfully negative. |
| `.inverse(): Matrix` | Requires square, non-singular; logs and returns a zero matrix instead of throwing on a non-square or singular input. |
| `.solve(b: Matrix): Matrix` | Solves `this * x = b` for `x` — more numerically stable than `inverse() * b`. |
| `.pseudoInverse(): Matrix` | Moore-Penrose pseudoinverse — generalizes `inverse` to non-square/singular matrices via least-squares. Used by `MecanumDriveKinematics`/`SwerveDriveKinematics` to turn their overdetermined forward-kinematics systems into solvable ones. |
| `.det() / .trace(): Double` | |
| `.exp(): Matrix` | Matrix exponential `eᴬ`, via scaling-and-squaring with a truncated Taylor series — a from-scratch replacement for WPILib's JNI/Eigen call. Requires square. |
| `.pow(exponent: Int): Matrix` | Repeated squaring for a non-negative integer power. |
| `.normF(): Double` | Frobenius norm. |
| `equals`/`hashCode`/`toString()` | `equals` compares with `1e-9` tolerance (`SimpleMatrix.isIdentical`). |
| `Matrix.zeros(rows, cols) / .eye(n): Matrix` (companion) | Zero / identity matrix. |
| `Matrix.fill(rows, cols, vararg values: Double): Matrix` (companion) | Builds a matrix from row-major `values`; logs and returns zeros on a count mismatch. |
| `Matrix.vector(vararg values: Double): Matrix` (companion) | Builds an n×1 column vector. |

#### `DCMotor.kt`

`class DCMotor(val nominalVoltageVolts: Double, stallTorqueNewtonMeters: Double, stallCurrentAmps: Double, freeCurrentAmps: Double, val freeSpeedRadPerSec: Double, numMotors: Int = 1)` —
the physical constants of a DC motor (or a gearbox of `numMotors` identical ones), for use in
state-space plant models (see `Models`). All fields are SI and already account for `numMotors`.

| Symbol | Description |
| --- | --- |
| `.stallTorqueNewtonMeters` / `.stallCurrentAmps` / `.freeCurrentAmps: Double` | Constructor args × `numMotors`. |
| `.rOhms: Double` | Motor internal resistance. |
| `.kvRadPerSecPerVolt: Double` | Motor velocity constant. |
| `.ktNMPerAmp: Double` | Motor torque constant. |
| `.getCurrent(speedRadiansPerSec: Double, voltageInputVolts: Double): Double` | Current drawn at a given speed under a given voltage. |
| `.getCurrent(torqueNm: Double): Double` | Current drawn to produce a given torque. |
| `.getTorque(currentAmps: Double): Double` | Torque produced by a given current. |
| `.getVoltage(torqueNm: Double, speedRadiansPerSec: Double): Double` | Voltage needed to produce a given torque at a given speed. |
| `.getSpeed(torqueNm: Double, voltageInputVolts: Double): Double` | Angular speed produced by a given torque at a given voltage. |
| `.withReduction(gearboxReduction: Double): DCMotor` | Copy with a gearbox reduction applied (output torque multiplied, output speed divided); resulting `numMotors` is folded to 1. |
| `DCMotor.cim/vex775Pro/neo/miniCim/bag/andymarkRs775_125/banebotsRs775/andymark9015/banebotsRs550/neo550/falcon500/falcon500Foc/romiBuiltIn/krakenX60/krakenX60Foc/neoVortex(numMotors: Int = 1): DCMotor` (companion) | Datasheet constants for common FRC/hobby motors — ported as-is from WPILib; useful as ballpark plant-model inputs even though none of these motors are used in FTC. |

#### `Models.kt`

`object Models` — factory functions building `LinearSystem` plant models for common mechanisms.
Every factory logs via `robotPrintError` (but still builds the system) if a physical parameter is
`<= 0` where it must be positive.

| Symbol | Description |
| --- | --- |
| `.elevatorSystem(motor: DCMotor, massKg: Double, radiusMeters: Double, gearing: Double): LinearSystem` | States/outputs `[position, velocity]`, input `[voltage]`. |
| `.flywheelSystem(motor: DCMotor, jKgMetersSquared: Double, gearing: Double): LinearSystem` | State/output `[angular velocity]`, input `[voltage]`. |
| `.dcMotorSystem(motor: DCMotor, jKgMetersSquared: Double, gearing: Double): LinearSystem` | States/outputs `[angular position, angular velocity]`, input `[voltage]`. |
| `.dcMotorSystem(kV: Double, kA: Double): LinearSystem` | Same shape, built from SysId-characterized `kV`/`kA` instead of motor+load physical parameters. |
| `.drivetrainVelocitySystem(motor: DCMotor, massKg: Double, rMeters: Double, rbMeters: Double, jKgMetersSquared: Double, gearing: Double): LinearSystem` | States/outputs `[left velocity, right velocity]`, inputs `[left voltage, right voltage]` — a coupled differential-drive velocity model. |
| `.singleJointedArmSystem(motor: DCMotor, jKgMetersSquared: Double, gearing: Double): LinearSystem` | States/outputs `[angle, angular velocity]`, input `[voltage]`. |
| `.identifyVelocitySystem(kV: Double, kA: Double): LinearSystem` | State/output `[velocity]`, input `[voltage]`, from SysId gains. |
| `.identifyPositionSystem(kV: Double, kA: Double): LinearSystem` | `= dcMotorSystem(kV, kA)`. |
| `.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular: Double): LinearSystem` | States/outputs `[left velocity, right velocity]`, inputs `[left voltage, right voltage]`, from four independently-characterized SysId gains. |
| `.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular, trackwidthMeters: Double): LinearSystem` | Same, but taking angular gains in volts per radian/sec(²) and converting via `trackwidthMeters` before delegating to the 4-arg overload. |

#### `LinearSystem.kt`

`class LinearSystem(val a: Matrix, val b: Matrix, val c: Matrix, val d: Matrix)` — a plant modeled
in state-space notation: `x' = Ax + Bu`, `y = Cx + Du`. See `Models` for common mechanism
factories. Constructor logs via `robotPrintError` for any non-finite element in `a`/`b`/`c`/`d`
(usually a model implementation error).

| Symbol | Description |
| --- | --- |
| `.calculateX(x: Matrix, clampedU: Matrix, dtSeconds: Double): Matrix` | The next state, given the current `x` and (already-clamped) input `clampedU`, over `dtSeconds` — discretizes `a`/`b` internally via `Discretization.discretizeAB`. |
| `.calculateY(x: Matrix, clampedU: Matrix): Matrix` | The output `y` for state `x` and input `clampedU`: `c*x + d*clampedU`. |
| `.toString()` | Multi-line dump of `A`/`B`/`C`/`D`. |

#### `LinearSystemLoop.kt`

`class LinearSystemLoop(controller: LinearQuadraticRegulator, feedforward: LinearPlantInversionFeedforward, val observer: KalmanFilter, clampFunction: (Matrix) -> Matrix)` —
combines a controller, feedforward, and observer into one full-state-feedback control loop for a
plant. "Inputs"/"outputs" are from the plant's perspective throughout (`u` is what you send to the
motors, `y` is what comes back from sensors). See `LinearQuadraticRegulator`/
`LinearPlantInversionFeedforward`/`KalmanFilter` (`math/control`/`math/estimator`) for those
collaborators.

| Symbol | Description |
| --- | --- |
| `LinearSystemLoop(plant: LinearSystem, controller: LinearQuadraticRegulator, observer: KalmanFilter, maxVoltageVolts: Double, dtSeconds: Double)` | Convenience constructor: builds a `LinearPlantInversionFeedforward` from `plant`/`dtSeconds` and a symmetric `±maxVoltageVolts` clamp. |
| `LinearSystemLoop(controller, feedforward, observer, maxVoltageVolts: Double)` | Same clamp, explicit feedforward. |
| `.u(): Matrix` | The controller's calculated (clamped) control input `u`, plus the feedforward. |
| `.reset(initialState: Matrix)` | Zeroes the reference and controller output, resets the feedforward and the observer's state estimate to `initialState`. Called once from `init`. |
| `.error(): Matrix` | The difference between the reference and the observer's current state estimate. |
| `.correct(y: Matrix)` | Corrects the observer's state estimate using measurement `y`. |
| `.predict(dtSeconds: Double)` | Sets a new controller output, projects the model forward, and runs observer prediction over `dtSeconds`. |
| `.setNextR(nextR: Matrix)` | Sets the reference state for the next `predict` call. |

#### `Discretization.kt`

`object Discretization` — converts continuous-time state-space matrices to their discrete-time
equivalents.

| Symbol | Description |
| --- | --- |
| `.discretizeA(contA: Matrix, dtSeconds: Double): Matrix` | `A_d = e^(A·dt)`. |
| `.discretizeAB(contA: Matrix, contB: Matrix, dtSeconds: Double): Pair<Matrix, Matrix>` | The discretized `(A, B)` pair, via the augmented-matrix exponential trick. |
| `.discretizeAQ(contA: Matrix, contQ: Matrix, dtSeconds: Double): Pair<Matrix, Matrix>` | The discretized `(A, Q)` pair, for propagating a Kalman filter's process-noise covariance. |
| `.discretizeR(contR: Matrix, dtSeconds: Double): Matrix` | `R_d = R / dt`. **`dtSeconds == 0` divides by zero** — not guarded. |

#### `DARE.kt`

`object DARE` — solves the discrete-time algebraic Riccati equation
`AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0` for its unique stabilizing solution `X`, via the
Structure-preserving Doubling Algorithm (SDA; Chu/Fan/Lin 2004 — the same underlying method
WPILib's own JNI-backed "Drake" solver uses). Upstream WPILib calls into a JNI binding to Drake's
C++/Eigen solver here, which isn't portable to Android — this reimplements the doubling iteration
directly in pure Kotlin/EJML. **Preconditions are unchecked** (matching WPILib's `dareNoPrecond` —
the caller is responsible): `Q` symmetric positive semidefinite, `R` symmetric positive definite,
`(A, B)` stabilizable, `(A, C)` where `Q = CᵀC` detectable. Iterates up to 100 times or until
convergence (`1e-10` tolerance on the Frobenius norm of the change in `H`).

| Symbol | Description |
| --- | --- |
| `.solve(A: Matrix, B: Matrix, Q: Matrix, R: Matrix): Matrix` | The standard (no cross-term) DARE. |
| `.solve(A: Matrix, B: Matrix, Q: Matrix, R: Matrix, N: Matrix): Matrix` | The cross-term overload: `AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0`. |

#### `NumericalIntegration.kt`

`object NumericalIntegration` — 4th-order Runge-Kutta numerical integration. Upstream WPILib also
has an adaptive Dormand-Prince integrator (`rkdp`) and a time-varying `f(t, y)` overload of `rk4`
— neither is exercised anywhere in this port (state estimation here only needs fixed-step
`f(x, u)`/`f(x)` integration), so both are left out rather than carried as untested dead code.

| Symbol | Description |
| --- | --- |
| `.rk4(f: (Double) -> Double, x: Double, dtSeconds: Double): Double` | Integrates `dx/dt = f(x)`, scalar form. |
| `.rk4(f: (Matrix, Matrix) -> Matrix, x: Matrix, u: Matrix, dtSeconds: Double): Matrix` | Integrates `dx/dt = f(x, u)`, holding `u` constant over the step. |
| `.rk4(f: (Matrix) -> Matrix, x: Matrix, dtSeconds: Double): Matrix` | Integrates `dx/dt = f(x)`, matrix form. |

#### `NumericalJacobian.kt`

`object NumericalJacobian` — central-difference numerical Jacobians (step `1e-5`), used to
linearize nonlinear plant/measurement models (e.g. for an Extended Kalman Filter).

| Symbol | Description |
| --- | --- |
| `.numericalJacobian(rows: Int, cols: Int, f: (Matrix) -> Matrix, x: Matrix): Matrix` | The `rows`×`cols` Jacobian of `f` w.r.t. its argument, evaluated at `x`. |
| `.numericalJacobianX(rows: Int, states: Int, f: (Matrix, Matrix) -> Matrix, x: Matrix, u: Matrix): Matrix` | The Jacobian of `f(x, u)` w.r.t. `x`, evaluated at `(x, u)`. |
| `.numericalJacobianU(rows: Int, inputs: Int, f: (Matrix, Matrix) -> Matrix, x: Matrix, u: Matrix): Matrix` | The Jacobian of `f(x, u)` w.r.t. `u`, evaluated at `(x, u)`. |

#### `StateSpaceUtil.kt`

`object StateSpaceUtil` — state-space helper functions. Upstream WPILib's
`isStabilizable`/`isDetectable` precondition checks are JNI-backed (Eigen eigenvalue
decomposition) and not portable here — skipped, same as this port's `DARE` solver already being
the unchecked `dareNoPrecond` variant.

| Symbol | Description |
| --- | --- |
| `.makeCovarianceMatrix(stdDevs: Matrix): Matrix` | A diagonal covariance matrix for a Kalman filter's Q/R, from a vector of per-state/output standard deviations (`stdDev²` on the diagonal). |
| `.makeWhiteNoiseVector(stdDevs: Matrix): Matrix` | A vector of independent Gaussian white noise, scaled per-element by `stdDevs`. |
| `.makeCostMatrix(tolerances: Matrix): Matrix` | A diagonal LQR cost matrix from per-state/input tolerances (Bryson's rule): `1/tolerance²` on the diagonal, or `0` where a tolerance is `Double.POSITIVE_INFINITY`. |
| `.clampInputMaxMagnitude(u: Matrix, uMin: Matrix, uMax: Matrix): Matrix` | Element-wise clamp of `u` between `uMin` and `uMax`. |
| `.desaturateInputVector(u: Matrix, maxMagnitude: Double): Matrix` | Uniformly scales `u` down (preserving direction) if its largest-magnitude element exceeds `maxMagnitude`. |

### `math/trajectory/`

Time-parameterized paths through waypoints — samples position/velocity/acceleration at any point
in time, subject to velocity/acceleration/drivetrain constraints. Built on top of `math/spline/`:
`TrajectoryGenerator` builds and parameterizes splines into raw pose+curvature samples,
`TrajectoryParameterizer` assigns each sample a time/velocity/acceleration, and the result is a
`Trajectory`.

#### `Trajectory.kt`

`class Trajectory(val states: List<State> = emptyList())` — a time-parameterized sequence of
`State`s: pose, curvature, velocity, and acceleration at each point in time.

| Symbol | Description |
| --- | --- |
| `.totalTimeSeconds: Double` | The last state's `timeSeconds` (`0.0` if `states` is empty). |
| `.initialPose: Pose2d` | `sample(0.0).pose`. |
| `.sample(timeSeconds: Double): State` | The (possibly interpolated) state at `timeSeconds` since the start; clamps to the first/last state outside `[0, totalTimeSeconds]`. Binary-searches `states`, then interpolates between the bracketing pair. **Throws (`check`) if `states` is empty.** |
| `.transformBy(transform: Transform2d): Trajectory` | This trajectory with every pose carried through `transform`, relative to the first pose — e.g. robot-relative to field-relative. |
| `.relativeTo(pose: Pose2d): Trajectory` | This trajectory with every pose expressed relative to `pose` instead of the field/origin frame. |
| `.concatenate(other: Trajectory): Trajectory` | This trajectory followed by `other`, whose timestamps are shifted to start where this one ends. Returns `other` unchanged if this trajectory has no states. |
| `data class State(val timeSeconds: Double = 0.0, val velocityMetersPerSecond: Double = 0.0, var accelerationMetersPerSecondSq: Double = 0.0, val pose: Pose2d = Pose2d.kZero, val curvatureRadPerMeter: Double = 0.0)` | One point in time along the trajectory. `accelerationMetersPerSecondSq` is `var` because `TrajectoryParameterizer` back-fills each state's acceleration from the *next* state's velocity delta after generation. |
| `State.interpolate(endValue: State, i: Double): State` | This state interpolated `i` of the way towards `endValue`, accounting for direction of travel (including a reversing/near-zero-velocity case) so the interpolated pose lands at the right arc-length fraction, not just a naive time lerp. |

#### `TrajectoryConfig.kt`

`class TrajectoryConfig(val maxVelocityMetersPerSecond: Double, val maxAccelerationMetersPerSecondSq: Double)` —
the start/end velocity, velocity/acceleration caps, custom constraints, and reversed flag used to
generate a `Trajectory`.

| Symbol | Description |
| --- | --- |
| `.constraints: MutableList<TrajectoryConstraint>` | Custom per-point velocity/acceleration constraints, applied in addition to the global max velocity/acceleration. |
| `.startVelocityMetersPerSecond` / `.endVelocityMetersPerSecond: Double` | Both default `0.0`. |
| `.reversed: Boolean` | Default `false` — drive the path backwards (tangent flipped 180°). |
| `.addConstraint(constraint: TrajectoryConstraint): TrajectoryConfig` | Appends one constraint; returns `this` for chaining. |
| `.addConstraints(constraints: List<TrajectoryConstraint>): TrajectoryConfig` | Appends several; returns `this`. |
| `.setKinematics(kinematics: DifferentialDriveKinematics): TrajectoryConfig` | Adds a `DifferentialDriveKinematicsConstraint` capping every wheel's speed at `maxVelocityMetersPerSecond`. |
| `.setKinematics(kinematics: MecanumDriveKinematics): TrajectoryConfig` | Adds a `MecanumDriveKinematicsConstraint`, same cap. |
| `.setKinematics(kinematics: SwerveDriveKinematics): TrajectoryConfig` | Adds a `SwerveDriveKinematicsConstraint`, same cap. |

#### `TrajectoryGenerator.kt`

`object TrajectoryGenerator` — builds `Trajectory`s from waypoints/control vectors via
clamped-cubic or quintic-Hermite splines. Every `generateTrajectory*` overload returns a
single-state "do nothing" trajectory (and logs via `robotPrintError`) instead of throwing if the
underlying splines turn out malformed (`SplineParameterizer.MalformedSplineException`).

| Symbol | Description |
| --- | --- |
| `.generateTrajectory(initial: Spline.ControlVector, interiorWaypoints: List<Translation2d>, end: Spline.ControlVector, config: TrajectoryConfig): Trajectory` | Clamped cubic splines through `initial`/`interiorWaypoints`/`end`'s exterior control vectors. If `config.reversed`, flips the initial/final tangents before spline-fitting and un-flips (180°, negated curvature) the resulting points afterward. |
| `.generateTrajectory(start: Pose2d, interiorWaypoints: List<Translation2d>, end: Pose2d, config: TrajectoryConfig): Trajectory` | Clamped cubic splines through `start`/`interiorWaypoints`/`end`, auto-choosing interior control vectors via `SplineHelper.getCubicControlVectorsFromWaypoints`, then delegating to the control-vector overload. |
| `.generateTrajectoryFromControlVectors(controlVectors: List<Spline.ControlVector>, config: TrajectoryConfig): Trajectory` | Quintic Hermite splines through fully-specified `controlVectors` (guarantees continuous curvature end-to-end, since every derivative up to acceleration is pinned). |
| `.generateTrajectory(waypoints: List<Pose2d>, config: TrajectoryConfig): Trajectory` | Quintic Hermite splines through `waypoints` (auto-derived control vectors, then `SplineHelper.optimizeCurvature`'d). |
| `.splinePointsFromSplines(splines: List<Spline>): List<PoseWithCurvature>` | Parameterizes `splines` by arc length (via `SplineParameterizer.parameterize`) into the `PoseWithCurvature` samples a trajectory is time-parameterized from; drops each spline's duplicate leading point. |

#### `TrajectoryParameterizer.kt`

`object TrajectoryParameterizer` — time-parameterizes a sequence of spline `PoseWithCurvature`
points into a `Trajectory` by running a forward pass (accelerate as much as allowed) and a
backward pass (decelerate as much as required) over the `TrajectoryConstraint`s, then integrating
the resulting velocity profile. See
[Sprunk 2008, "Planning Motion Trajectories for Mobile Robots Using Splines"](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf)
for the derivation.

| Symbol | Description |
| --- | --- |
| `.timeParameterizeTrajectory(points: List<PoseWithCurvature>, constraints: List<TrajectoryConstraint>, startVelocityMetersPerSecond: Double, endVelocityMetersPerSecond: Double, maxVelocityMetersPerSecond: Double, maxAccelerationMetersPerSecondSq: Double, reversed: Boolean): Trajectory` | The full two-pass algorithm — normally called via `TrajectoryGenerator`, not directly. |
| `class TrajectoryGenerationException(message: String) : RuntimeException` | Thrown when a constraint is infeasible (its min acceleration exceeds its max at some point) or the forward/backward passes leave a point with both zero velocity and zero reachable acceleration (the trajectory can't actually be traversed as specified). **Unlike `TrajectoryGenerator`'s malformed-spline case, this one propagates as a real exception, not a logged fallback.** |

### `math/trajectory/constraint/`

Per-point velocity/acceleration limits (`TrajectoryConstraint` implementations) plugged into a
`TrajectoryConfig` to keep a generated trajectory within what the physical drivetrain can actually
do.

#### `TrajectoryConstraint.kt`

`interface TrajectoryConstraint` — a user-defined velocity/acceleration limit applied while
generating a trajectory.

| Symbol | Description |
| --- | --- |
| `.getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): Double` | The absolute maximum velocity at `pose`/`curvatureRadPerMeter`, starting from `velocityMetersPerSecond`. |
| `.getMinMaxAccelerationMetersPerSecondSq(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double): MinMax` | The acceleration bounds at that pose/curvature/velocity. |
| `data class MinMax(val minAccelerationMetersPerSecondSq: Double = -Double.MAX_VALUE, val maxAccelerationMetersPerSecondSq: Double = Double.MAX_VALUE)` | Default is "unconstrained" — constraints that only limit velocity (not acceleration) can return the no-arg default. |

#### `CentripetalAccelerationConstraint.kt`

| Symbol | Description |
| --- | --- |
| `class CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSecondSq: Double) : TrajectoryConstraint` | Caps centripetal acceleration (`v² / r = v² · curvature`), slowing the robot through tight turns so sharp-cornered trajectories stay trackable: `v_max = sqrt(a_c,max / \|curvature\|)`. Doesn't constrain tangential acceleration (`getMinMaxAccelerationMetersPerSecondSq` returns the unconstrained default). |

#### `DifferentialDriveKinematicsConstraint.kt`

| Symbol | Description |
| --- | --- |
| `class DifferentialDriveKinematicsConstraint(kinematics: DifferentialDriveKinematics, maxSpeedMetersPerSecond: Double) : TrajectoryConstraint` | Caps trajectory velocity so neither side of a differential drivetrain exceeds `maxSpeedMetersPerSecond`: converts the candidate speed+curvature to wheel speeds, desaturates them, and converts back to get the achievable chassis `vx`. Only constrains velocity, not acceleration. |

#### `MecanumDriveKinematicsConstraint.kt`

| Symbol | Description |
| --- | --- |
| `class MecanumDriveKinematicsConstraint(kinematics: MecanumDriveKinematics, maxSpeedMetersPerSecond: Double) : TrajectoryConstraint` | Caps trajectory velocity so no wheel of a mecanum drivetrain exceeds `maxSpeedMetersPerSecond` — same desaturate-and-convert-back approach as the differential-drive constraint, but resolving `vx`/`vy` from the pose heading and curvature first. Only constrains velocity, not acceleration. |

#### `SwerveDriveKinematicsConstraint.kt`

| Symbol | Description |
| --- | --- |
| `class SwerveDriveKinematicsConstraint(kinematics: SwerveDriveKinematics, maxSpeedMetersPerSecond: Double) : TrajectoryConstraint` | Caps trajectory velocity so no module of a swerve drivetrain exceeds `maxSpeedMetersPerSecond` — same shape as the mecanum constraint, via `SwerveDriveKinematics.desaturateWheelSpeeds`. Only constrains velocity, not acceleration. |

#### `DifferentialDriveVoltageConstraint.kt`

| Symbol | Description |
| --- | --- |
| `class DifferentialDriveVoltageConstraint(feedforward: SimpleMotorFeedforward, kinematics: DifferentialDriveKinematics, maxVoltage: Double) : TrajectoryConstraint` | Caps trajectory *acceleration* so no wheel of a differential drivetrain ever needs more than `maxVoltage` — the only constraint here that limits acceleration rather than velocity (`getMaxVelocityMetersPerSecond` always returns `Double.POSITIVE_INFINITY`). Computes achievable max/min wheel acceleration from `feedforward` at the candidate wheel speeds, then converts back to chassis acceleration bounds, accounting for how track width and curvature couple the two wheels' accelerations (and flips the sign when the turn radius is tighter than half the track width, since one wheel must then run backwards relative to the other). |

### `hardware/`

`Data.kt` — shared enums used by hardware wrappers:

| Symbol | Description |
| --- | --- |
| `Data.Motors.Direction { FORWARD, REVERSE }` | The direction a motor rotates; carries a `.multiplier: Int` (`1`/`-1`). |
| `Data.Motors.GoBILDA` | GoBILDA yellow-jacket gearbox presets (`RPM_30` … `RPM_1620`, `BARE`) — each carries `.cpr: Double` (ticks/rev) and `.rpm: Double` (free-run RPM). |
| `Data.Motors.RunMode { VELOCITY_CONTROL, POSITION_CONTROL, RAW_POWER }` | `HaMotor`'s control mode. |
| `Data.Motors.ZeroPowerBehavior { UNKNOWN, BRAKE, FLOAT }` | Wraps the SDK's `DcMotor.ZeroPowerBehavior` (`.sdkBehavior`). |
| `Data.Servos.Mode { CR, FULL_RANGE }` | Continuous-rotation vs. positional servo. |
| `Data.Servos.Type { Torque, Speed, SuperSpeed, AxonMax, AxonMini }` | Each entry carries `.range: Double` (total mechanical sweep in degrees — kept as a plain `Double`, not `Rotation2d`, since 300°/350° sweeps exceed the `(-180, 180]` domain `Rotation2d` normalizes into) and `.maxSpeed: AngularVelocity` (datasheet sweep/no-load speed) — `Torque`/`Speed`/`SuperSpeed` are 300°, `AxonMax`/`AxonMini` are 350°. |

`HardwareDevice.kt` — AlonLib's own minimal device interface (distinct from the FTC SDK's own `com.qualcomm.robotcore.hardware.HardwareDevice`, which most of the wrappers below implement instead — see note below):

```kotlin
interface HardwareDevice {
    fun disable()
    fun getDeviceType(): String
}
```

| Symbol | Description |
| --- | --- |
| `.disable()` | Releases the underlying FTC SDK device. |
| `.getDeviceType(): String` | Human-readable description of the device, for logging. |

Only `HaAbsoluteAnalogEncoder` and `HaRevIMU` implement this interface directly. Every other wrapper in this package instead implements the **FTC SDK's** `com.qualcomm.robotcore.hardware.HardwareDevice` (`getManufacturer()`/`getDeviceName()`/`getConnectionInfo()`/`getVersion()`/`resetDeviceConfigurationForOpMode()`/`close()`) — most sensor wrappers do this via Kotlin interface delegation (`: HardwareDevice by <underlying field>`), which forwards every one of those calls straight to the wrapped SDK object with no logic of its own; `HaMotor`, `HaServo`, `HaLimelight3A`, and `HaPinPoint` implement it with hand-written overrides instead (documented per-class below). `HaLED` implements neither, since none of the SDK interfaces it wraps (`Light`/`SwitchableLight`/`Blinker`) extend the SDK's `HardwareDevice` either.

#### `HaMotor`

`hardware/motors/HaMotor.kt` — owns an SDK `DcMotorEx` directly plus its own software PIDF loop
(position/velocity control run in software and written out as voltage, not the motor
controller's own onboard PID) and software current limiting. Implements the SDK's `HardwareDevice`.

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
| `.velocityController` / `.positionController: PIDFController` | The software PID loops backing `RunMode.VELOCITY_CONTROL`/`RunMode.POSITION_CONTROL`. |
| `.feedForwardController: SimpleMotorFeedforward` | Rebuilt from `pidfGains.kS/KV/Ka` whenever `pidfGains` is set. |
| `.cachingTolerance: Double` | The minimum power delta (default `0.0001`) before `percentOutput`'s setter actually writes to `motor`. |
| `.zeroPowerBehavior: ZeroPowerBehavior` | `FLOAT` (default) or `BRAKE`; forwarded to the underlying motor and every one of `followers`. |
| `.runningDirection: Direction` | `FORWARD`/`REVERSE`; backed by `motor.direction`. |
| `.runMode: RunMode` | `RAW_POWER` (default, `update()` is a no-op), `POSITION_CONTROL`, or `VELOCITY_CONTROL` — selects what `setPoint`/`update()` do. |
| `.percentOutput: Percentage` | Direct `[-1, 1]` power. Clamped to `[minPercentOutput, maxPercentOutput]`, further scaled by `currentLimitScalar`. Refuses to move past a tripped `forwardLimit`/`reverseLimit` (logs instead). Mirrored to every one of `followers` once applied. |
| `.voltage: Double` | Get: `batteryVoltage * percentOutput`. Set: converts a target voltage to `percentOutput` given current battery voltage (floored at 1.0V to avoid divide-by-near-zero). |
| `.current: Double` | Motor current in milliamps. |
| `.currentLimit: Double` | Milliamps; `<= 0.0` disables current limiting entirely (default). |
| `.currentLimitStep: Double` | How much `currentLimitScalar` moves per `update()` call while backing off/recovering (`[0, 1]`, default `0.05`). |
| `.currentLimitScalar: Double` (read-only) | Current derating factor in `[0, 1]`; `1.0` = no derating. |
| `.forwardLimit` / `.reverseLimit: () -> Boolean` | Software limit-switch callbacks, checked by `percentOutput`'s setter only (default `{ false }`). |
| `.position: Rotation2d` | Get: current encoder position. Set: sets the PID `setPoint` (in `POSITION_CONTROL` mode), clamped to `[minimumPosition, maximumPosition]`. |
| `.velocity: AngularVelocity` | Get: current encoder velocity. Set: sets the PID `setPoint` (in `VELOCITY_CONTROL` mode); `0.rpm` instead directly zeroes `motor.power`. Clamped to `±maxRpm`. |
| `.pidfGains: PIDFGains` | Applying this pushes `kP/kI/kD` into both PID controllers and rebuilds `feedForwardController` from `kS/KV/Ka`. |
| `.setPoint: Double` | The active controller's raw setpoint (degrees in `POSITION_CONTROL`, RPM in `VELOCITY_CONTROL`); resets both PID controllers first if either has nonzero `i`. Clamped to the relevant min/max. |
| `.error: Double` (read-only) | Active controller's position error; `0.0` in `RAW_POWER`. |
| `.tolerance: Double` | Forwarded to whichever PID controller is active via `setTolerance`. |
| `.inTolerance: Boolean` (read-only) | Active controller's `atSetPoint()`; always `true` in `RAW_POWER`. |
| `.minPercentOutput` / `.maxPercentOutput: Double` | Default `-1.0`/`1.0`; each is coerced to stay on the correct side of the other. |
| `.maximumPosition` / `.minimumPosition: Rotation2d` | Default `±180°`; rejects (logs, doesn't apply) a value that would invert the min/max ordering. |
| `.stop()` | Sets `percentOutput = 0.0`, zeroes `motor.power` directly, and stops every one of `followers` too. |
| `.update()` | **Call every loop.** Runs `limitCurrent()`, then — in `VELOCITY_CONTROL`/`POSITION_CONTROL` — computes `voltage` from the active PID controller + feedforward + `kFF * sign(error)`. No-op in `RAW_POWER`. |
| `HardwareDevice` overrides | `getManufacturer()` (`Unknown`), `getDeviceName()` (`"HaMotor"`), `getConnectionInfo()` (`""`), `getVersion()` (`1`), `resetDeviceConfigurationForOpMode()` (stops+resets encoder, propagated to `followers`), `close()` (closes the motor, propagated to `followers`). |

#### `HaServo`

`hardware/servos/HaServo.kt` — wraps a `Servo` (force-cast to `ServoImplEx` to set a 500–2500µs PWM
range on construction). Implements the SDK's `HardwareDevice`.

```kotlin
class HaServo(hardwareMap: HardwareMap, id: String, mode: Data.Servos.Mode, type: Data.Servos.Type, vararg followers: HaServo)
```

Optional `followers` mirror this servo's raw `[0, 1]` position every time it's written (via
`percentOutput`/`position`/`velocity`) — construct each one the way you want it to run and pass it
in here.

| Symbol | Description |
| --- | --- |
| `.servo: Servo` | The underlying raw servo — escape hatch, not meant for normal use. |
| `.setPwm(pwmRange: PwmControl.PwmRange)` | Overrides the 500–2500µs default PWM range set at construction; returns `this`. |
| `.controller: ServoControllerEx` (read-only) | `servo.controller`, force-cast to `ServoControllerEx`. |
| `.cachingTolerance: Double` | The minimum position delta (default `0.0001`) before a write actually reaches the servo (and its `followers`). |
| `.forwardLimit` / `.reverseLimit: () -> Boolean` | Software limit callbacks for `percentOutput` only (default `{ false }`). |
| `.maxPercentOutput` / `.minPercentOutput: Double` | Default `1.0`/`0.0`, each coerced against the other, both within `[0, 1]`. |
| `.percentOutput: Double` | Get: `servo.position`. Set: clamped to `[minPercentOutput, maxPercentOutput]`; refuses to move past a tripped limit (logs instead). |
| `.maxPosition` / `.minPosition: Rotation2d` | Soft position limits **relative to the center of the servo's physical sweep** (0° = centered, not one end) — half of `type.range` in each direction. Defaults to `±type.range / 2`. |
| `.minLimit` / `.maxLimit: Double` | Soft limits in plain degrees **from the low end of the physical sweep** (e.g. straight off a datasheet), applied on top of (not instead of) `minPosition`/`maxPosition`. Default `0.0`/`type.range` (i.e. no extra restriction). |
| `.position: Rotation2d` | `Mode.FULL_RANGE` only (`Mode.CR` logs an error and does nothing). Set: clamps to `[minPosition, maxPosition]`, converts to an absolute physical angle, clamps again to `[minLimit, maxLimit]`, then writes `servo.position` as a `[0, 1]` fraction of `type.range`. |
| `.maxVelocity` / `.minVelocity: AngularVelocity` | Default `type.maxSpeed`/`0.rpm`, each coerced within `[0, type.maxSpeed]`. |
| `.velocity: AngularVelocity` | `Mode.CR` only (`Mode.FULL_RANGE` logs an error and does nothing). Set: maps `[minVelocity, maxVelocity]` onto the servo's raw `[?, 1]` CR range. |
| `.runningDirection: HaMotor.Direction`-style `Direction` | `FORWARD`/`REVERSE`; backed by `servo.direction` (inverted, since this `Direction` and the FTC SDK's `Servo.Direction` disagree on sense). |
| `.stop()` | `Mode.CR`: sets `percentOutput = 0.0`. `Mode.FULL_RANGE`: no-op (a positional servo has no "stop"). Also stops every one of `followers`. |
| `HardwareDevice` overrides | `getManufacturer()` (`Unknown`), `getDeviceName()` (`"HaServo"`), `getConnectionInfo()` (`""`), `getVersion()` (`1`), `resetDeviceConfigurationForOpMode()` (no-op), `close()` (closes the underlying servo, propagated to `followers`). |

#### `HaCRServo`

`hardware/servos/HaCRServo.kt` — a continuous-rotation servo, in the same style as `HaServo`/`HaMotor`,
with optional absolute-position closed-loop control (e.g. for an Axon servo with a feedback wire),
power-write caching, and optional `followers` that mirror this servo's `percentOutput`. Implements
the SDK's `HardwareDevice` via Kotlin delegation to the wrapped `CRServo`.

```kotlin
class HaCRServo(crServo: CRServo, absolutePositionRadians: (() -> Double)? = null, vararg followers: HaCRServo)
// or, from a hardware map:
HaCRServo(hardwareMap: HardwareMap, id: String, absolutePositionRadians: (() -> Double)? = null, vararg followers: HaCRServo)
```

| Symbol | Description |
| --- | --- |
| `.crServo: CRServo` | The underlying raw SDK continuous-rotation servo. |
| `.absolutePositionRadians: (() -> Double)?` | Optional feedback-position source (e.g. an Axon's feedback wire), used by `RunMode.OPTIMIZED_POSITIONAL_CONTROL`. |
| `RunMode { OPTIMIZED_POSITIONAL_CONTROL, RAW_POWER }` | What `percentOutput` means — a target angle steered towards via `pidf`, or a raw `[-1, 1]` power. |
| `.runMode: RunMode` | Selects `percentOutput`'s meaning; default `RAW_POWER`. |
| `.pidf: PIDFController?` | The closed-loop controller used in `RunMode.OPTIMIZED_POSITIONAL_CONTROL`; must be set before using that mode (throws otherwise). |
| `.getAbsolutePositionRadians(): Double` | Invokes `absolutePositionRadians`; throws `IllegalStateException` if none was configured. |
| `.cachingTolerance: Double` | The minimum power delta (default `0.0001`) before `percentOutput`'s setter actually writes to `crServo`. |
| `.forwardLimit` / `.reverseLimit: () -> Boolean` | Software limit callbacks for `percentOutput` only (default `{ false }`). |
| `.maxPercentOutput` / `.minPercentOutput: Double` | Default `1.0`/`-1.0`, each coerced against the other. |
| `.percentOutput: Double` | In `RAW_POWER`, a direct `[-1, 1]` power; in `OPTIMIZED_POSITIONAL_CONTROL`, a target angle in radians that `pidf` steers `getAbsolutePositionRadians()` towards (via `angleModulus` of the error). Clamped to `[minPercentOutput, maxPercentOutput]`; refuses to move past a tripped limit (logs instead). Mirrored to every one of `followers` once applied. |
| `.setPwm(pwmRange: PwmControl.PwmRange)` | Sets the raw PWM range via `controller.setServoPwmRange`; returns `this`. |
| `.controller: ServoControllerEx` (read-only) | `crServo.controller`, force-cast to `ServoControllerEx`. |
| `.inverted: Boolean` | Backed by `crServo.direction` (`REVERSE`/`FORWARD`). |
| `.stop()` | Sets `percentOutput = 0.0` and stops every one of `followers` too. |
| `HardwareDevice` overrides | All forwarded via Kotlin delegation (`by crServo`) straight to the underlying SDK object. |

### `hardware/sensors/`

The following are thin wrappers over FTC SDK sensor interfaces. Unless noted otherwise, each
implements the SDK's `HardwareDevice` via Kotlin delegation (`: HardwareDevice by <underlying field>`)
— every `HardwareDevice` method (`getManufacturer()`, `getDeviceName()`, `getConnectionInfo()`,
`getVersion()`, `resetDeviceConfigurationForOpMode()`, `close()`) is forwarded straight through to
the wrapped SDK object with no logic of its own.

#### `HaAbsoluteAnalogEncoder`

`hardware/sensors/HaAbsoluteAnalogEncoder.kt` — an absolute analog encoder (e.g. an Axon servo's
feedback wire) read through an `AnalogInput`, normalized to `[0, max)` of `angleUnit`. Implements
AlonLib's own `HardwareDevice` (not the SDK's).

```kotlin
open class HaAbsoluteAnalogEncoder(encoder: AnalogInput, id: String = "", range: Double = 3.3, angleUnit: AngleUnit = AngleUnit.RADIANS)
// or, from a hardware map:
HaAbsoluteAnalogEncoder(hardwareMap: HardwareMap, id: String, range: Double = 3.3, angleUnit: AngleUnit = AngleUnit.RADIANS)
```

| Symbol | Description |
| --- | --- |
| `.angleUnit: AngleUnit` | `DEGREES` or `RADIANS` — determines the range `getCurrentPosition()` normalizes into (`360.0`/`2π`). |
| `.reversed: Boolean` (read-only, set via `setReversed`) | Whether raw voltage is read inverted. |
| `.voltage` (read-only) | Raw voltage from the underlying `AnalogInput`. |
| `.zero(offset: Double)` | Sets the zero offset applied by `getCurrentPosition()`; returns `this`. |
| `.setReversed(reversed: Boolean)` | Sets `reversed`; returns `this`. |
| `.getCurrentPosition(): Double` | Normalized position in `[0, max)` of `angleUnit`, accounting for `zero`/`setReversed`; also updates the internal velocity estimate used by `getVelocity()`. |
| `.getVelocity(): Double` | Estimated rate of change of `getCurrentPosition()`, computed on each `getCurrentPosition()` call. |
| `.getEncoder(): AnalogInput` | The underlying raw `AnalogInput`. |
| `DEFAULT_RANGE` (companion) | `3.3` (volts). |
| `HardwareDevice` overrides | `disable()` is a no-op (matching upstream — the underlying `AnalogInput` is not closed); `getDeviceType()` returns `"Absolute Analog Encoder; $id"`. |

#### `HaAccelerationSensor`

`hardware/sensors/HaAccelerationSensor.kt` — thin wrapper over the SDK's `AccelerationSensor`.

```kotlin
class HaAccelerationSensor(sensor: AccelerationSensor)
// or:
HaAccelerationSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.sensor: AccelerationSensor` | The underlying SDK sensor. |
| `.acceleration` (read-only) | `sensor.acceleration`. |
| `HardwareDevice` overrides | All forwarded via delegation to `sensor`. |

#### `HaAnalogInput`

`hardware/sensors/HaAnalogInput.kt` — a raw analog input pin (e.g. a potentiometer or an
absolute-encoder feedback wire) — see `HaAbsoluteAnalogEncoder` for the normalized-angle version.

```kotlin
class HaAnalogInput(analogInput: AnalogInput)
// or:
HaAnalogInput(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.analogInput: AnalogInput` | The underlying SDK input. |
| `.voltage` (read-only) | `analogInput.voltage`. |
| `.maxVoltage` (read-only) | `analogInput.maxVoltage`. |
| `HardwareDevice` overrides | All forwarded via delegation to `analogInput`. |

#### `HaBlinkinLedDriver`

`hardware/sensors/HaBlinkinLedDriver.kt` — the REV Blinkin LED driver: a servo-PWM-controlled LED
strip with a fixed set of built-in patterns.

```kotlin
class HaBlinkinLedDriver(blinkin: RevBlinkinLedDriver)
// or:
HaBlinkinLedDriver(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.blinkin: RevBlinkinLedDriver` | The underlying SDK driver. |
| `.pattern: RevBlinkinLedDriver.BlinkinPattern?` | Set applies the pattern via `blinkin.setPattern()` (a `null` value is stored but not applied). |
| `HardwareDevice` overrides | All forwarded via delegation to `blinkin`. |

#### `HaColorSensor`

`hardware/sensors/HaColorSensor.kt` — a normalized color sensor (works with any vendor via the
SDK's `NormalizedColorSensor` interface). If the underlying device is also a plain `ColorSensor`
(nearly all of them are, e.g. a REV Color Sensor V3), raw ARGB is available too alongside
`normalizedColors`; if it's also a `DistanceSensor` (again, e.g. a REV Color Sensor V3), so is
`distance()`. Both throw a `ClassCastException` if the underlying device doesn't implement that
interface.

```kotlin
class HaColorSensor(colorSensor: NormalizedColorSensor)
// or:
HaColorSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.colorSensor: NormalizedColorSensor` | The underlying SDK sensor. |
| `.normalizedColors` (read-only) | `colorSensor.normalizedColors`. |
| `.gain: Float` | `colorSensor.gain`. |
| `.hsvToArgb(alpha: Int, hsv: FloatArray): IntArray` | Converts HSV to `[alpha, red, green, blue]` via `android.graphics.Color`. |
| `.rgbToHsv(red: Int, green: Int, blue: Int, hsv: FloatArray): FloatArray` | Converts RGB into the supplied `hsv` array (returned back). |
| `.getArgb(): IntArray` | `[alpha(), red(), green(), blue()]`. |
| `.alpha()` / `.red()` / `.green()` / `.blue(): Int` | Requires the device to also be a `ColorSensor`; throws otherwise. |
| `.distance(unit: DistanceUnit): Double` | Requires the device to also be a `DistanceSensor` (e.g. REV Color Sensor V3); throws otherwise. |
| `HardwareDevice` overrides | All forwarded via delegation to `colorSensor`. |

#### `HaCompassSensor`

`hardware/sensors/HaCompassSensor.kt` — thin wrapper over the SDK's legacy `CompassSensor`.

```kotlin
class HaCompassSensor(sensor: CompassSensor)
// or:
HaCompassSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.sensor: CompassSensor` | The underlying SDK sensor. |
| `.direction` (read-only) | `sensor.direction`. |
| `.calibrationFailed` (read-only) | `sensor.calibrationFailed()`. |
| `.mode: CompassSensor.CompassMode` | Setting it calls `sensor.setMode(mode)`. |
| `HardwareDevice` overrides | All forwarded via delegation to `sensor`. |

#### `HaDigitalChannel`

`hardware/sensors/HaDigitalChannel.kt` — a single digital I/O pin (e.g. a beam-break or a limit
switch wired directly, not through a `TouchSensor`), with optional debouncing for input use — set
`threshold` above `0.0`, call `update()` once per loop, then read `isActive`.

```kotlin
class HaDigitalChannel(digitalChannel: DigitalChannel, threshold: Double = 0.0)
// or:
HaDigitalChannel(hardwareMap: HardwareMap, id: String, threshold: Double = 0.0)
```

| Symbol | Description |
| --- | --- |
| `.digitalChannel: DigitalChannel` | The underlying SDK channel. |
| `.mode: DigitalChannel.Mode` | `digitalChannel.mode`. |
| `.state: Boolean` | `digitalChannel.state`; set via `digitalChannel.setState()`. |
| `.threshold: Double` | Debounce window in milliseconds; `0.0` (default) disables debouncing entirely. Coerced to `>= 0.0`. |
| `.update()` | Refreshes `isActive` against `threshold` and `state` — call once per loop before reading `isActive`. |
| `.isActive: Boolean` (read-only) | `state`, debounced by `threshold` if it's above `0.0`. |
| `HardwareDevice` overrides | All forwarded via delegation to `digitalChannel`. |

#### `HaDistanceSensor`

`hardware/sensors/HaDistanceSensor.kt` — wraps a `DistanceSensor` (works with any vendor, e.g. the
REV 2m time-of-flight sensor), with optional named `DistanceTarget` tracking.

```kotlin
class HaDistanceSensor(distanceSensor: DistanceSensor, targets: List<DistanceTarget> = emptyList())
// or:
HaDistanceSensor(hardwareMap: HardwareMap, id: String, targets: List<DistanceTarget> = emptyList())
```

| Symbol | Description |
| --- | --- |
| `.distanceSensor: DistanceSensor` | The underlying SDK sensor. |
| `.getDistance(unit: DistanceUnit): Double` | `distanceSensor.getDistance(unit)`. |
| `.targetReached(target: DistanceTarget): Boolean` | Whether a live reading (in the target's unit) falls within it. |
| `.addTarget(target: DistanceTarget)` / `.addTargets(targets: List<DistanceTarget>)` | Adds to the internal target list (deduplicated by reference). |
| `.checkAllTargets(): Map<DistanceTarget, Boolean>` | Every registered target mapped to whether it's currently reached. |
| `HardwareDevice` overrides | All forwarded via delegation to `distanceSensor`. |

`DistanceTarget` — a named distance range that `targetReached`/`checkAllTargets` check a live reading against:

```kotlin
class DistanceTarget(unit: DistanceUnit, minThreshold: Double, maxThreshold: Double, name: String = "Distance Target")
// or, a target centered on `target`, ±5 `unit` either side:
DistanceTarget(unit: DistanceUnit, target: Double)
```

| Symbol | Description |
| --- | --- |
| `.unit: DistanceUnit` | The unit `minThreshold`/`maxThreshold`/`target` are in. |
| `.minThreshold` / `.maxThreshold: Double` | The target range; constructor `require()`s both `>= 0` and `min <= max`. |
| `.name: String` | Defaults to `"Distance Target"`. |
| `.target: Double` (read-only) | `(minThreshold + maxThreshold) / 2.0`. |
| `.atTarget(currentDistance: Double): Boolean` | Whether `currentDistance` falls within `[minThreshold, maxThreshold]`. |

#### `HaGyroscope`

`hardware/sensors/HaGyroscope.kt` — the legacy `GyroSensor` interface — superseded by `HaIMU` on
modern hubs, but still SDK-supported for older gyro modules.

```kotlin
class HaGyroscope(gyroSensor: GyroSensor)
// or:
HaGyroscope(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.gyroSensor: GyroSensor` | The underlying SDK sensor. |
| `.calibrate()` | `gyroSensor.calibrate()`. |
| `.isCalibrating` (read-only) | `gyroSensor.isCalibrating`. |
| `.heading` (read-only) | `gyroSensor.heading`. |
| `.rotationFraction` (read-only) | `gyroSensor.rotationFraction`. |
| `.rawX` / `.rawY` / `.rawZ` (read-only) | `gyroSensor.rawX()`/`rawY()`/`rawZ()`. |
| `.resetZAxisIntegrator()` | `gyroSensor.resetZAxisIntegrator()`. |
| `.getAngularVelocity(angleUnit: AngleUnit)` | Requires the device to also implement the SDK's `Gyroscope` interface; throws otherwise. |
| `HardwareDevice` overrides | All forwarded via delegation to `gyroSensor`. |

#### `HaHuskyLens`

`hardware/sensors/HaHuskyLens.kt` — the DFRobot HuskyLens AI vision sensor: onboard
object/tag/color/line recognition, in whichever `HuskyLens.Algorithm` mode is selected.

```kotlin
class HaHuskyLens(huskyLens: HuskyLens)
// or:
HaHuskyLens(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.huskyLens: HuskyLens` | The underlying SDK device. |
| `.knock()` | `huskyLens.knock()` — re-establishes the I2C connection. |
| `.selectAlgorithm(algorithm: HuskyLens.Algorithm)` | `huskyLens.selectAlgorithm(algorithm)`. |
| `.blocks(): Array<HuskyLens.Block>` / `.blocks(id: Int): Array<HuskyLens.Block>` | All detected blocks, or only those matching `id`. |
| `.arrows(): Array<HuskyLens.Arrow>` / `.arrows(id: Int): Array<HuskyLens.Arrow>` | All detected arrows, or only those matching `id`. |
| `HardwareDevice` overrides | All forwarded via delegation to `huskyLens`. |

#### `HaIMU`

`hardware/sensors/HaIMU.kt` — wraps the FTC SDK's universal `IMU` interface — works with any modern
hub-mounted IMU, regardless of vendor.

```kotlin
class HaIMU(imu: IMU)
// or:
HaIMU(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.imu: IMU` | The underlying SDK device. |
| `.initialize(parameters: IMU.Parameters)` | `imu.initialize(parameters)`. |
| `.resetYaw()` | `imu.resetYaw()`. |
| `.yawPitchRollAngles` (read-only) | `imu.robotYawPitchRollAngles`. |
| `.rotation2d: Rotation2d` (read-only) | Yaw converted to a `Rotation2d`, the form the rest of AlonLib's geometry/odometry expects. |
| `.getOrientation(axesReference: AxesReference, axesOrder: AxesOrder, angleUnit: AngleUnit)` | `imu.getRobotOrientation(...)`. |
| `.orientationAsQuaternion` (read-only) | `imu.robotOrientationAsQuaternion`. |
| `.getAngularVelocity(angleUnit: AngleUnit)` | `imu.getRobotAngularVelocity(angleUnit)`. |
| `HardwareDevice` overrides | All forwarded via delegation to `imu`. |

#### `HaIrSeekerSensor`

`hardware/sensors/HaIrSeekerSensor.kt` — thin wrapper over the SDK's `IrSeekerSensor`.

```kotlin
class HaIrSeekerSensor(irSeekerSensor: IrSeekerSensor)
// or:
HaIrSeekerSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.irSeekerSensor: IrSeekerSensor` | The underlying SDK sensor. |
| `.signalDetectedThreshold: Double` | `irSeekerSensor.signalDetectedThreshold`. |
| `.mode: IrSeekerSensor.Mode` | `irSeekerSensor.mode`. |
| `.signalDetected` (read-only) | `irSeekerSensor.signalDetected()`. |
| `.angle` (read-only) | `irSeekerSensor.angle`. |
| `.strength` (read-only) | `irSeekerSensor.strength`. |
| `.individualSensors: Array<IrSeekerSensor.IrSeekerIndividualSensor>` (read-only) | `irSeekerSensor.individualSensors`. |
| `HardwareDevice` overrides | All forwarded via delegation to `irSeekerSensor`. |

#### `HaLED`

`hardware/sensors/HaLED.kt` — covers any of the SDK's `Light`/`SwitchableLight`/`Blinker`
interfaces. Unlike most `Ha*` wrappers, none of these extend the SDK's `HardwareDevice`, so this
doesn't either. Methods specific to one of the three throw a `ClassCastException` if the underlying
device doesn't support it.

```kotlin
class HaLED(device: Any)
// or, gets a SwitchableLight from the hardware map:
HaLED(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.isOn: Boolean` (read-only) | Requires the device to be a `Light`; throws otherwise. |
| `.setOn(on: Boolean)` | Requires the device to be a `SwitchableLight`; throws otherwise. |
| `.setPattern(pattern: Collection<Blinker.Step>)` | Requires the device to be a `Blinker`; throws otherwise. |
| `.setConstantColor(color: Int)` | Requires the device to be a `Blinker`; throws otherwise. |
| `.stopBlinking()` | Requires the device to be a `Blinker`; throws otherwise. |

#### `HaLEDStick`

`hardware/sensors/HaLEDStick.kt` — the SparkFun addressable LED stick: per-pixel color/brightness
control.

```kotlin
class HaLEDStick(ledStick: SparkFunLEDStick)
// or:
HaLEDStick(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.ledStick: SparkFunLEDStick` | The underlying SDK device. |
| `.setColor(color: Int)` | Sets every pixel to `color`. |
| `.setColor(position: Int, color: Int)` | Sets pixel `position` to `color`. |
| `.setColors(colors: IntArray)` | Sets every pixel individually, one color per element. |
| `.setBrightness(brightness: Int)` / `.setBrightness(position: Int, brightness: Int)` | Sets brightness for all pixels, or one. |
| `.turnAllOff()` | `ledStick.turnAllOff()`. |
| `HardwareDevice` overrides | All forwarded via delegation to `ledStick`. |

#### `HaLimelight3A`

`hardware/sensors/HaLimelight3A.kt` — thin wrapper over the FTC SDK's `Limelight3A`. Implements the
SDK's `HardwareDevice`.

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

#### `HaOTOS`

`hardware/sensors/HaOTOS.kt` — the SparkFun Optical Tracking Odometry Sensor: absolute
position/velocity/acceleration tracking, with an onboard IMU for heading.

```kotlin
class HaOTOS(otos: SparkFunOTOS)
// or:
HaOTOS(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.otos: SparkFunOTOS` | The underlying SDK device. |
| `.calibrateImu(numSamples: Int = 255, waitUntilDone: Boolean = true)` | `otos.calibrateImu(...)`. |
| `.imuCalibrationProgress` (read-only) | `otos.imuCalibrationProgress`. |
| `.linearUnit: DistanceUnit` | `otos.linearUnit`. |
| `.angularUnit: AngleUnit` | `otos.angularUnit`. |
| `.linearScalar: Double` | Get: `otos.linearScalar`. Set: `otos.setLinearScalar(value)`. |
| `.angularScalar: Double` | Get: `otos.angularScalar`. Set: `otos.setAngularScalar(value)`. |
| `.resetTracking()` | `otos.resetTracking()`. |
| `.status: SparkFunOTOS.Status` (read-only) | `otos.status`. |
| `.offset: SparkFunOTOS.Pose2D` | `otos.offset`. |
| `.position: SparkFunOTOS.Pose2D` | `otos.position`. |
| `.velocity: SparkFunOTOS.Pose2D` (read-only) | `otos.velocity`. |
| `.acceleration: SparkFunOTOS.Pose2D` (read-only) | `otos.acceleration`. |
| `HardwareDevice` overrides | All forwarded via delegation to `otos`. |

#### `HaOctoQuad`

`hardware/sensors/HaOctoQuad.kt` — the DFRobot/Digital Chicken Labs OctoQuad: an 8-channel
quadrature/pulse-width encoder reader. Covers channel reading/reset/direction/bank config — see
`.octoQuad` directly for the localizer and pulse-width-specific features.

```kotlin
class HaOctoQuad(octoQuad: OctoQuad)
// or:
HaOctoQuad(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.octoQuad: OctoQuad` | The underlying SDK device. |
| `.firmwareVersion: String` (read-only) | `octoQuad.firmwareVersionString`. |
| `.setEncoderDirection(channel: Int, direction: OctoQuad.EncoderDirection)` / `.getEncoderDirection(channel: Int)` | Per-channel encoder direction. |
| `.setAllEncoderDirections(reversed: BooleanArray)` | `octoQuad.setAllEncoderDirections(reversed)`. |
| `.setChannelBankConfig(config: OctoQuad.ChannelBankConfig)` / `.getChannelBankConfig()` | Channel bank configuration. |
| `.readAllEncoderData(): OctoQuad.EncoderDataBlock` | Every channel's position/velocity in one I2C transaction — prefer this over per-channel reads when polling all 8. |
| `.readPosition(channel: Int)` / `.readVelocity(channel: Int)` | Per-channel single reads. |
| `.resetPosition(channel: Int)` / `.resetAllPositions()` | Per-channel or all-channel position reset. |
| `.setCachingMode(mode: OctoQuad.CachingMode)` / `.refreshCache()` | Read caching control. |
| `.saveParametersToFlash()` | `octoQuad.saveParametersToFlash()`. |
| `.resetEverything()` | `octoQuad.resetEverything()`. |
| `HardwareDevice` overrides | All forwarded via delegation to `octoQuad`. |

#### `HaOpticalDistanceSensor`

`hardware/sensors/HaOpticalDistanceSensor.kt` — thin wrapper over the SDK's `OpticalDistanceSensor`.

```kotlin
class HaOpticalDistanceSensor(sensor: OpticalDistanceSensor)
// or:
HaOpticalDistanceSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.sensor: OpticalDistanceSensor` | The underlying SDK sensor. |
| `.lightDetected` (read-only) | `sensor.lightDetected`. |
| `.rawLightDetected` (read-only) | `sensor.rawLightDetected`. |
| `.rawLightDetectedMax` (read-only) | `sensor.rawLightDetectedMax`. |
| `.enableLed(enable: Boolean)` | `sensor.enableLed(enable)`. |
| `HardwareDevice` overrides | All forwarded via delegation to `sensor`. |

#### `HaPinPoint`

`hardware/sensors/HaPinPoint.kt` — thin wrapper over goBILDA's `GoBildaPinpointDriver` odometry
computer. Implements the SDK's `HardwareDevice`.

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

#### `HaPwmOutput`

`hardware/sensors/HaPwmOutput.kt` — a raw PWM output pin.

```kotlin
class HaPwmOutput(pwmOutput: PWMOutput)
// or:
HaPwmOutput(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.pwmOutput: PWMOutput` | The underlying SDK device. |
| `.pulseWidthOutputTimeMicros: Int` | The output pulse width, in microseconds; backed by `pwmOutput.pulseWidthOutputTime`. |
| `.pulseWidthPeriodMicros: Int` | The full PWM period, in microseconds; backed by `pwmOutput.pulseWidthPeriod`. |
| `HardwareDevice` overrides | All forwarded via delegation to `pwmOutput`. |

#### `HaRevIMU`

`hardware/sensors/HaRevIMU.kt` — the REV Expansion/Control Hub's built-in `BNO055IMU`. Prefer
`HaIMU` on modern hubs (it wraps the universal `IMU` interface, working with any vendor); this
exists for the legacy BNO055-specific API some older configs still expect. Implements AlonLib's own
`HardwareDevice` (not the SDK's).

```kotlin
open class HaRevIMU(revIMU: BNO055IMU)
// or:
HaRevIMU(hardwareMap: HardwareMap, imuName: String = "imu")
```

| Symbol | Description |
| --- | --- |
| `.init()` | Initializes with a default `BNO055IMU.Parameters` (`AngleUnit.DEGREES`, `calibrationDataFile = "BNO055IMUCalibration.json"`, logging enabled, `loggingTag = "IMU"`). |
| `.init(parameters: BNO055IMU.Parameters)` | Initializes with custom parameters; resets the internal heading offset. |
| `.invertGyro()` | Flips the sign of every heading this reports. |
| `.getHeading(): Double` | `getAbsoluteHeading() - globalHeadingOffset`. |
| `.getAbsoluteHeading(): Double` | `revIMU.angularOrientation.firstAngle`, scaled by the sign set via `invertGyro()`. |
| `.getAngles(): DoubleArray` | `[firstAngle, secondAngle, thirdAngle]` from `revIMU.angularOrientation`. |
| `.getRotation2d(): Rotation2d` | `getHeading()` as a `Rotation2d`. |
| `.reset()` | Adds the current `getHeading()` into the internal offset (effectively re-zeroes it). |
| `.getRevIMU(): BNO055IMU` | The underlying raw SDK device. |
| `HardwareDevice` overrides | `disable()` closes `revIMU`; `getDeviceType()` returns `"Rev Expansion Hub IMU"`. |

#### `HaTouchSensor`

`hardware/sensors/HaTouchSensor.kt` — thin wrapper over the SDK's `TouchSensor`.

```kotlin
class HaTouchSensor(touchSensor: TouchSensor)
// or:
HaTouchSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.touchSensor: TouchSensor` | The underlying SDK sensor. |
| `.value` (read-only) | `touchSensor.value`. |
| `.isPressed` (read-only) | `touchSensor.isPressed`. |
| `HardwareDevice` overrides | All forwarded via delegation to `touchSensor`. |

#### `HaUltrasonicSensor`

`hardware/sensors/HaUltrasonicSensor.kt` — thin wrapper over the SDK's `UltrasonicSensor`.

```kotlin
class HaUltrasonicSensor(sensor: UltrasonicSensor)
// or:
HaUltrasonicSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.sensor: UltrasonicSensor` | The underlying SDK sensor. |
| `.ultrasonicLevel` (read-only) | `sensor.ultrasonicLevel`. |
| `.status: String` (read-only) | `sensor.status()`. |
| `HardwareDevice` overrides | All forwarded via delegation to `sensor`. |

#### `HaVoltageSensor`

`hardware/sensors/HaVoltageSensor.kt` — typically the Control/Expansion Hub's own battery voltage
sensor (`hardwareMap.voltageSensor` also enumerates every one on the robot).

```kotlin
class HaVoltageSensor(voltageSensor: VoltageSensor)
// or:
HaVoltageSensor(hardwareMap: HardwareMap, id: String)
```

| Symbol | Description |
| --- | --- |
| `.voltageSensor: VoltageSensor` | The underlying SDK sensor. |
| `.voltage` (read-only) | `voltageSensor.voltage`. |
| `HardwareDevice` overrides | All forwarded via delegation to `voltageSensor`. |

### `drives/`

Drivebase subsystem classes. All extend the shared abstract base below.

#### `RobotDrive`

`RobotDrive.kt` — shared plumbing for every drivebase in this package: input clipping/squaring
and wheel-speed normalization. Abstract; not instantiated directly.

```kotlin
abstract class RobotDrive
```

| Symbol | Description |
| --- | --- |
| `MotorType` (nested enum) | Indices into a drivebase's motor array: `FRONT_LEFT`/`FRONT_RIGHT`/`BACK_LEFT`/`BACK_RIGHT` (0-3, for 4-motor drives), `LEFT`/`RIGHT`/`SLIDE` (0-2, for 3-motor H-drives — `LEFT`/`RIGHT` alias the same values as `FRONT_LEFT`/`FRONT_RIGHT`). |
| `.setMaxSpeed(maxOutput: Double)` / `.getMaxSpeed(): Double` | Get/set an overall output scaler (default `1.0`) every drive method multiplies wheel powers by. |
| `.setRange(min: Double, max: Double)` | Sets the clip range drive inputs are clamped to (default `[-1, 1]`). |
| `.clipRange(value: Double): Double` (protected) | Clamps `value` to the configured range. |
| `.stop()` (abstract) | Stops the drivebase; each subclass implements this. |
| `.normalize(wheelSpeeds: DoubleArray, magnitude: Double)` (protected) | Scales `wheelSpeeds` in place so the largest-magnitude entry has magnitude `magnitude`. |
| `.normalize(wheelSpeeds: DoubleArray)` (protected) | Scales `wheelSpeeds` down to `[-1, 1]` in place if any entry exceeds it, preserving ratios. |
| `.squareInput(input: Double): Double` (protected) | Squares `input`'s magnitude while preserving sign — finer control near zero on joystick inputs. |

#### `DifferentialDrive`

`DifferentialDrive.kt` — a two-side (tank) drivebase: `left`/`right` motors (each optionally an
`HaMotor` with its own followers) driven together per side.

```kotlin
class DifferentialDrive(left: HaMotor, right: HaMotor, autoInvert: Boolean = true)
```

| Symbol | Description |
| --- | --- |
| `.isRightSideInverted: Boolean` (read-only) | Whether the right side's multiplier is `-1.0`. |
| `.setRightSideInverted(isInverted: Boolean)` | Sets the right-side multiplier to `-1.0`/`1.0`. |
| `.stop()` | Stops both motors. |
| `.arcadeDrive(forwardSpeed, turnSpeed, squareInputs: Boolean = false)` | `forwardSpeed` drives both sides equally; `turnSpeed` adds to the left, subtracts from the right; wheel speeds normalized before output. |
| `.tankDrive(leftSpeed, rightSpeed, squareInputs: Boolean = false)` | Drives each side directly (each side's speed independently clipped/normalized). |

Invert an individual motor yourself before passing it in if needed — `autoInvert`/`setRightSideInverted`
only control the whole right side's multiplier.

#### `HDrive`

`HDrive.kt` — a holonomic drivebase: either the classic 3-motor "H-drive" (two angled drive wheels
plus a perpendicular slide wheel) or, given four motors, a mecanum-like layout.

```kotlin
class HDrive(
    motors: Array<HaMotor>,
    leftMotorAngleRadians: Double = DEFAULT_LEFT_MOTOR_ANGLE,   // 2π/3
    rightMotorAngleRadians: Double = DEFAULT_RIGHT_MOTOR_ANGLE, // π/3
    slideMotorAngleRadians: Double = DEFAULT_SLIDE_MOTOR_ANGLE, // 3π/2
)
// or, for the 3-motor case specifically:
HDrive(left: HaMotor, right: HaMotor, slide: HaMotor, leftMotorAngleRadians = ..., rightMotorAngleRadians = ..., slideMotorAngleRadians = ...)
```

| Symbol | Description |
| --- | --- |
| `.stop()` | Stops every motor. |
| `.driveFieldCentric(strafeSpeed, forwardSpeed, turn, headingRadians)` | Field-relative drive. With 3 motors, projects the rotated strafe/forward vector onto each wheel's angled axis (plus turn), normalizes, and drives — note the left/right output slots are deliberately swapped in the 3-motor branch, matching upstream SolversLib's behavior (documented in-source as intentional, not a transcription bug). With 4 motors, falls through to the same mecanum-style kinematics as `MecanumDrive`. |
| `.driveRobotCentric(strafeSpeed, forwardSpeed, turn)` | Robot-relative — `driveFieldCentric(..., headingRadians = 0.0)`. |

#### `MecanumDrive`

`MecanumDrive.kt` — a four-wheel mecanum drivebase built directly from four `HaMotor`s (rather
than an `Array<HaMotor>` + `MotorType` indices, unlike `HDrive`/`HaMecanumDrive`). See the
kinematics derivation this implementation follows: https://www.youtube.com/watch?v=8rhAkjViHEQ.

```kotlin
class MecanumDrive(frontLeft: HaMotor, frontRight: HaMotor, backLeft: HaMotor, backRight: HaMotor, autoInvert: Boolean = true)
```

| Symbol | Description |
| --- | --- |
| `.isRightSideInverted: Boolean` (read-only) | Whether the right side's multiplier is `-1.0`. |
| `.setRightSideInverted(isInverted: Boolean)` | Sets the right-side multiplier to `-1.0`/`1.0`. |
| `.stop()` | Stops every motor. |
| `.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, squareInputs: Boolean = false)` | Robot-relative — forward always drives the way the robot's currently facing. |
| `.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, headingRadians, squareInputs: Boolean = false)` | Field-relative — forward always drives away from the driver regardless of `headingRadians`. Rotates the strafe/forward vector, computes per-wheel sine-based speeds, normalizes, adds turn, normalizes again, then drives. |
| `.driveWithMotorPowers(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed)` | Drives each wheel directly (right-side speeds scaled by the right-side multiplier), all scaled by `maxOutput`. |

**Relationship to `drives/mecanumDrive/HaMecanumDrive.kt`:** these are two independent, largely
redundant mecanum implementations, not a base/derived pair. `MecanumDrive` takes its four motors as
named constructor parameters (`frontLeft`, `frontRight`, `backLeft`, `backRight`); `HaMecanumDrive`
takes an `Array<HaMotor>` indexed via `RobotDrive.MotorType` (with a secondary constructor accepting
the same four named motors). Both extend `RobotDrive`, both implement the identical field/robot-centric
mecanum kinematics (rotate input vector, `sin(theta ± π/4)` per-wheel speeds, normalize, add turn,
normalize again) and both default to inverting the right side. The differences are cosmetic: `HaMecanumDrive`
exposes `motors`/`rightSideMultiplier` as public `var`s while `MecanumDrive` keeps them private behind
accessors, and `HaMecanumDrive`'s `driveRobotCentric`/`driveFieldCentric` overloads are separate
functions with an explicit `Boolean` overload rather than a defaulted parameter. Neither wraps or
calls the other. Prefer `HaMecanumDrive` for new code — it's the one already covered in this README's
worked examples and is the one actively referenced elsewhere in the library (e.g. no other file reads
or constructs a plain `MecanumDrive`).

#### `drives/mecanumDrive/HaMecanumDrive`

`HaMecanumDrive.kt` — a mecanum drivetrain subsystem extending `RobotDrive`, driving
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

#### `drives/swerve/coaxial/CoaxialSwerveDrivetrain`

`CoaxialSwerveDrivetrain.kt` — a standard 4-module coaxial swerve drivetrain: 4 drive motors + 4
pod-rotation servos (Axon-style, with absolute encoders). `motors`/`swervos` are ordered starting
from front-right, going counterclockwise. Constructing it sets every motor's `runMode` to
`RAW_POWER` and `zeroPowerBehavior` to `BRAKE`.

```kotlin
class CoaxialSwerveDrivetrain(
    trackWidth: Double,
    wheelBase: Double,
    maxSpeed: Double,
    swervoPidf: PIDFController,
    motors: Array<HaMotor>,   // exactly 4
    swervos: Array<HaCRServo>, // exactly 4
)
```

Throws `IllegalArgumentException` (via `require`) if `motors`/`swervos` aren't each exactly 4
elements, or if `trackWidth`/`wheelBase`/`maxSpeed` aren't all positive.

| Symbol | Description |
| --- | --- |
| `.modules: Array<CoaxialSwerveModule>` | The 4 modules, built from `motors[i]`/`swervos[i]` with each one's offset from the robot's center derived from `trackWidth`/`wheelBase`. |
| `.targetVelocity: ChassisSpeeds` (read-only from outside) | The currently commanded robot-centric velocity — set via `setTargetVelocity`. |
| `.setCachingTolerance(motorCachingTolerance, swervoCachingTolerance)` | Forwards to every module's `setCachingTolerance`. Returns `this`. |
| `.setTargetVelocity(velocity: ChassisSpeeds)` | Sets `targetVelocity`, scaled down (preserving direction) if it exceeds `maxSpeed`/`getMaxSpeed()`'s linear or angular limit. |
| `.update(): Array<Vector2d>` | Computes each module's velocity vector for the current `targetVelocity`, normalizes their magnitudes together, drives each module, and returns the resulting (possibly re-scaled) per-module vectors. |
| `.updateWithTargetVelocity(velocity: ChassisSpeeds)` | `setTargetVelocity(velocity)` followed by `update()`. |
| `.updateWithXLock()` | Points every module diagonally outward (an X shape, at 90°-apart angles) to resist being pushed, driving each at a nominal `0.0001` magnitude. |
| `.stop()` | Stops every module (motor + servo). |

#### `drives/swerve/coaxial/CoaxialSwerveModule`

`CoaxialSwerveModule.kt` — one coaxial swerve module: a drive `motor` plus a pod-rotation `swervo`
(an `HaCRServo` with an absolute encoder configured so 0 means "wheel facing forward, positive
motor power drives the robot forward"). `offset` is the module's position relative to the robot's
center, in inches.

```kotlin
class CoaxialSwerveModule(motor: HaMotor, swervo: HaCRServo, offset: Vector2d, maxSpeed: Double, swervoPidf: PIDFController)
```

| Symbol | Description |
| --- | --- |
| `.targetVelocity: Vector2d` (read-only from outside) | The module's currently commanded (unnormalized) velocity vector — set via `setTargetVelocity`. |
| `.angleError: Double` (read-only) | The last-computed heading error (radians) between the swervo's current angle and `targetVelocity`'s direction, after any wheel-flip adjustment. |
| `.wheelFlipped: Boolean` (read-only) | True if the last `updateModule()` call chose to spin the wheel backward and turn the pod ≤90° instead of turning it further to face forward. |
| `.calculateVectorRobotCentric(target: ChassisSpeeds): Vector2d` | The unnormalized module velocity vector (robot-centric) needed to track `target`'s linear + angular components; see the in-source Desmos link for the derivation. |
| `.setTargetVelocity(velocity: Vector2d)` | Sets `targetVelocity` without driving hardware. |
| `.updateModule()` | Drives the motor/swervo to follow the current `targetVelocity`: picks the shorter turn (flipping the wheel's drive direction if the raw angle error exceeds ±90°), then sets motor power (scaled by `cos(angleError)`) and runs the swervo's PIDF to zero out `angleError`. |
| `.updateModuleWithVelocity(velocity: Vector2d)` | `setTargetVelocity(velocity)` followed by `updateModule()`. |
| `.stop()` | Stops the swervo, then the motor. |
| `.setCachingTolerance(motorCachingTolerance, swervoCachingTolerance)` | Sets `motor.cachingTolerance` / `swervo.cachingTolerance`. Returns `this`. |
| `.getPowerTelemetry(): String` | A formatted `"Motor=...,Servo=...,Absolute Encoder=..."` string for telemetry/debugging. |
| `.setSwervoPidf(pidf: PIDFController)` | Replaces the swervo's PIDF controller. |

### `gamepad/`

Edge-detected, driver-friendly wrappers over the raw FTC SDK `Gamepad`.

#### `KeyReader`

`KeyReader.kt` — the shared interface for edge-detecting a boolean control (a button, or a
thresholded trigger/axis). Call `readValue()` once per loop to advance it.

```kotlin
interface KeyReader
```

| Symbol | Description |
| --- | --- |
| `.readValue()` | Advances the reader's internal last/current state — call once per loop. |
| `.isDown(): Boolean` | Whether the control is currently held. |
| `.wasJustPressed(): Boolean` | True for one loop when the control transitions false→true. |
| `.wasJustReleased(): Boolean` | True for one loop when the control transitions true→false. |
| `.stateJustChanged(): Boolean` | True if `isDown()` differs from last loop's state. |

#### `ButtonReader`

`ButtonReader.kt` — edge-detects a button, either read from a `GamepadEx`/`GamepadKeys.Button` pair
or an arbitrary `() -> Boolean` supplier. Implements `KeyReader`. `isDown()` here re-reads the live
supplier rather than the cached `currState` (so it can differ from `stateJustChanged()`'s notion
of "current" until the next `readValue()`).

```kotlin
open class ButtonReader(buttonState: () -> Boolean)
// or:
ButtonReader(gamepad: GamepadEx, button: GamepadKeys.Button)
```

| Symbol | Description |
| --- | --- |
| `.readValue()` | Advances last/current state from `buttonState()`. |
| `.isDown(): Boolean` | `buttonState()`, read live. |
| `.wasJustPressed(): Boolean` | `!lastState && currState`. |
| `.wasJustReleased(): Boolean` | `lastState && !currState`. |
| `.stateJustChanged(): Boolean` | `lastState != currState`. |

#### `ToggleButtonReader`

`ToggleButtonReader.kt` — a `ButtonReader` that flips a persistent on/off state each time the
button is released.

```kotlin
class ToggleButtonReader(buttonState: () -> Boolean)
// or:
ToggleButtonReader(gamepad: GamepadEx, button: GamepadKeys.Button)
```

| Symbol | Description |
| --- | --- |
| `.state: Boolean` (read-only) | The current toggle state; flips whenever read immediately after `wasJustReleased()` becomes true. |

#### `TriggerReader`

`TriggerReader.kt` — edge-detects an analog trigger, treating it as "down" past `threshold`.
Implements `KeyReader`.

```kotlin
class TriggerReader(gamepad: GamepadEx, trigger: GamepadKeys.Trigger, threshold: Double = 0.5)
```

| Symbol | Description |
| --- | --- |
| `.readValue()` | Advances last/current state from `gamepad.getTrigger(trigger) > threshold`. |
| `.isDown(): Boolean` | The cached `currState` (unlike `ButtonReader`, doesn't re-read live). |
| `.wasJustPressed()` / `.wasJustReleased()` / `.stateJustChanged()` | Same semantics as `KeyReader`. |

#### `GamepadKeys`

`GamepadKeys.kt` — every button/trigger `GamepadEx` can read.

```kotlin
object GamepadKeys
```

| Symbol | Description |
| --- | --- |
| `Button` (enum) | `Y, X, A, B, LEFT_BUMPER, RIGHT_BUMPER, BACK, START, OPTIONS, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON, TRIANGLE, SQUARE, CROSS, CIRCLE, PS, SHARE, TOUCHPAD, TOUCHPAD_FINGER_1, TOUCHPAD_FINGER_2`. PlayStation-style aliases (`TRIANGLE`/`SQUARE`/`CROSS`/`CIRCLE`) map to the same physical buttons as `Y`/`X`/`A`/`B`. |
| `Trigger` (enum) | `LEFT_TRIGGER, RIGHT_TRIGGER`. |

#### `GamepadEx`

`GamepadEx.kt` — a richer wrapper over the raw FTC SDK `Gamepad`: edge-detected buttons, optional
joystick slew-rate limiting, and `GamepadButton`s (see `commands/button/`) for binding commands.

```kotlin
class GamepadEx(val gamepad: Gamepad)
```

| Symbol | Description |
| --- | --- |
| `.gamepad: Gamepad` | The wrapped raw SDK gamepad. |
| `.getButton(button: GamepadKeys.Button): Boolean` | Live read of that button's state, resolving PlayStation aliases to the same underlying field as their Xbox-style counterpart. |
| `.getTrigger(trigger: GamepadKeys.Trigger): Double` | Live read of that trigger's analog value. |
| `.setJoystickSlewRateLimiters(lx, ly, rx, ry: SlewRateLimiter?)` | Enables slew-rate limiting on the left/right stick X/Y axes; pass `null` for any axis that shouldn't be limited. Returns `this`. |
| `.leftY` / `.rightY` / `.leftX` / `.rightX: Double` (read-only) | Joystick axis values (through the corresponding limiter if set). Only `leftY` is sign-flipped from the raw SDK convention (`-gamepad.left_stick_y`, so pushing up is positive); `rightY`, `leftX`, and `rightX` pass the raw SDK value straight through unflipped. |
| `.wasJustPressed(button)` / `.wasJustReleased(button)` / `.isDown(button)` / `.stateJustChanged(button)` | Delegate to that button's internal `ButtonReader`. |
| `.readButtons()` | Advances every tracked button's edge detection — call once per loop. |
| `.getGamepadButton(button: GamepadKeys.Button): GamepadButton` | The command-binding `GamepadButton` for that button (see `commands/button/GamepadButton`). |

### `p2p/`

#### `P2PController`

`P2PController.kt` — a simple point-to-point field-centric controller: drives straight towards
`target` (magnitude from `translationalController`, direction from the straight-line bearing to
it) while `headingController` independently turns to face `target`'s rotation. Unlike SolversLib's
version, this doesn't take a separate `AngleUnit` parameter — `Rotation2d` is already unit-agnostic
(canonically radians internally, with both `.radians`/`.degrees` accessors).

```kotlin
class P2PController(
    translationalController: PIDController,
    headingController: PIDController,
    start: Pose2d = Pose2d.kZero,
    target: Pose2d = Pose2d.kZero,
    positionTolerance: Double,
    angularToleranceRadians: Double,
)
```

| Symbol | Description |
| --- | --- |
| `.translationalController: PIDController` | The PID driving translational (straight-line distance) output. |
| `.headingController: PIDController` | The PID driving heading output. |
| `.target: Pose2d` (read-only from outside) | The current target pose — set via `setTarget`. |
| `.error: Transform2d` (read-only) | The pose error (`target` minus current), updated each `calculate()` call and after construction. |
| `.calculate(pv: Pose2d): ChassisSpeeds` | Computes field-centric chassis speeds to drive from `pv` towards `target`: translational PID output resolved along the bearing to the target, heading PID output independently for rotation; both optionally passed through their slew-rate limiters. |
| `.setSlewRateLimiters(magnitudeLimiter, headingLimiter: SlewRateLimiter?)` | Sets optional output limiters for the translational magnitude and heading outputs. Returns `this`. |
| `.setTarget(target: Pose2d)` | Updates the target pose. |
| `.setTolerance(positionTolerance, angularToleranceRadians: Double)` | Sets both underlying PID controllers' tolerances. |
| `.atTarget(): Boolean` | `translationalController.atSetPoint() && headingController.atSetPoint()`. |

### `purepursuit/`

A pure-pursuit path-following implementation: an ordered `Path` of `Waypoint`s, driven each loop
by feeding in the robot's current pose and reading back `[strafe, forward, turn]` motor powers.
Ported from SolversLib's pure-pursuit module (line-circle intersection algorithm credited in-source
to FTC team 11115 "Gluten Free").

#### `Waypoint`

`Waypoint.kt` — the interface every point on a `Path` implements. See `GeneralWaypoint` and its
subtypes for the concrete implementations.

```kotlin
interface Waypoint
```

| Symbol | Description |
| --- | --- |
| `.type: WaypointType` | Which concrete kind of waypoint this is. |
| `.pose: Pose2d` | The waypoint's field position/heading. |
| `.followDistance: Double` | The pure-pursuit follow (look-ahead) radius used when computing line-circle intersections against this waypoint's segment. |
| `.timeoutMilliseconds: Long` | Per-waypoint timeout; `-1` means none. |

#### `types/WaypointType`

`types/WaypointType.kt` — the kind of waypoint, as read from `Waypoint.type`.

```kotlin
enum class WaypointType
```

| Symbol | Description |
| --- | --- |
| `GENERAL` | Ordinary curve-through waypoint (`GeneralWaypoint`). |
| `POINT_TURN` | Stop, turn in place, continue (`PointTurnWaypoint`). |
| `INTERRUPT` | Like `POINT_TURN`, plus fires an action (`InterruptWaypoint`). |
| `START` | The path's mandatory first waypoint (`StartWaypoint`). |
| `END` | The path's mandatory last waypoint (`EndWaypoint`). |

#### `types/PathType`

`types/PathType.kt` — how a `Path` picks the "best" intersection when several are found in the
same loop.

```kotlin
enum class PathType
```

| Symbol | Description |
| --- | --- |
| `HEADING_CONTROLLED` | Prefers the intersection closest to the robot's current heading (falls back to point-turn priority rules identical to the other mode). |
| `WAYPOINT_ORDERING_CONTROLLED` | (Default.) Prefers the intersection belonging to the furthest-along (highest-index) waypoint in path order. |

#### `waypoints/GeneralWaypoint`

`waypoints/GeneralWaypoint.kt` — the ordinary pure-pursuit waypoint: the robot curves through it
without stopping. Most other waypoint types (`PointTurnWaypoint`, `InterruptWaypoint`, `EndWaypoint`)
extend this one. SolversLib's Java version has ~5 overloaded constructors covering every combination
of (translation+rotation | pose | bare x/y) and (with/without a preferred angle); this Kotlin port
collapses them into named/default parameters.

```kotlin
open class GeneralWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    preferredAngleRadians: Double? = null,
    copyMode: Boolean = false, // internal
)
// or:
GeneralWaypoint(x: Double, y: Double, movementSpeed = 0.0, turnSpeed = 0.0, followRadius = 0.0, preferredAngleRadians: Double? = null)
```

`copyMode`: if true, this waypoint's speed/radius/timeout/angle are overwritten from the previous
waypoint in the path when the path initializes (via `inherit`) — lets you specify a waypoint's
position now and its motion settings later, in one shared place.

| Symbol | Description |
| --- | --- |
| `.movementSpeed` / `.turnSpeed: Double` (read-only from outside) | Coerced to `[0, 1]`; set via `setMovementSpeed`/`setTurnSpeed`. |
| `.followRadius: Double` (read-only from outside) | The pure-pursuit look-ahead radius; set via `setFollowRadius`. |
| `.timeoutMilliseconds: Long` (read-only from outside) | `-1` (none) by default; set via `setTimeout`. |
| `.usingPreferredAngle: Boolean` | Whether a preferred heading is configured. |
| `.preferredAngleRadians: Double` | The preferred heading, if `usingPreferredAngle`; throws otherwise. |
| `.type: WaypointType` | `GENERAL` (overridden by subclasses). |
| `.followDistance: Double` | `followRadius`. |
| `.setMovementSpeed(speed)` / `.setTurnSpeed(speed)` / `.setFollowRadius(radius)` / `.setPreferredAngle(angleRadians)` / `.setTimeout(millis)` / `.disablePreferredAngle()` | Fluent setters; each returns `this`. |
| `.reset()` (open) | Called once when the path initializes; subclasses override to reset per-run state. No-op here. |
| `.inherit(waypoint: Waypoint)` | If `copyMode`, copies `waypoint`'s movement/turn speed, follow radius, timeout, and preferred angle onto this one. Throws `IllegalArgumentException` if `waypoint` isn't itself a `GeneralWaypoint`. Called by `Path.init()`. |

#### `waypoints/PointTurnWaypoint`

`waypoints/PointTurnWaypoint.kt` — a waypoint the robot comes to a complete stop at, turns in place
towards the next waypoint, then continues, rather than curving through it like a plain `GeneralWaypoint`.
Extends `GeneralWaypoint`.

```kotlin
open class PointTurnWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    positionBuffer: Double,
    rotationBuffer: Double,
    preferredAngleRadians: Double? = null,
)
```

`positionBuffer`/`rotationBuffer` must each be `> 0` (throws `IllegalArgumentException` otherwise) —
they're the position/rotation tolerances used to decide the robot has "arrived" and can start/finish
its turn.

| Symbol | Description |
| --- | --- |
| `.positionBuffer` / `.rotationBuffer: Double` (read-only from outside) | Set via `setPositionBuffer`/`setRotationBuffer` (both re-validate `> 0`). |
| `.hasTraversed: Boolean` (read-only from outside) | Whether the robot has finished its turn at this waypoint. |
| `.setTraversed()` (open) | Marks `hasTraversed = true`. |
| `.reset()` | Clears `hasTraversed`. |
| `.type: WaypointType` | `POINT_TURN`. |

#### `waypoints/InterruptWaypoint`

`waypoints/InterruptWaypoint.kt` — a `PointTurnWaypoint` that also performs an action once the
robot stops and turns to face it, before continuing — for "do something mid-path". Extends
`PointTurnWaypoint`.

```kotlin
open class InterruptWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    positionBuffer: Double,
    rotationBuffer: Double,
    preferredAngleRadians: Double? = null,
    action: InterruptAction = InterruptAction {},
)
```

| Symbol | Description |
| --- | --- |
| `.actionPerformed: Boolean` (read-only from outside) | Whether `action` has fired since the last `reset()`. |
| `.setAction(newAction: InterruptAction)` (open) | Replaces the action to perform. Returns `this`. |
| `.performAction()` | Runs `action` once, if not already performed since the last reset. |
| `.reset()` | Clears `actionPerformed` (plus the `PointTurnWaypoint` reset). |
| `.type: WaypointType` | `INTERRUPT`. |

#### `waypoints/StartWaypoint`

`waypoints/StartWaypoint.kt` — the mandatory first waypoint of every `Path`; the robot never
actually traverses it, so it carries no motion settings.

```kotlin
class StartWaypoint(pose: Pose2d) : Waypoint
```

| Symbol | Description |
| --- | --- |
| `.type` | `START`. |
| `.followDistance` | `0.0` (never used). |
| `.timeoutMilliseconds` | `-1`. |

#### `waypoints/EndWaypoint`

`waypoints/EndWaypoint.kt` — the mandatory last waypoint of every `Path`: an `InterruptWaypoint`
whose action marks the path finished, and can't be changed. Extends `InterruptWaypoint`.

```kotlin
class EndWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    positionBuffer: Double,
    rotationBuffer: Double,
    preferredAngleRadians: Double? = null,
)
```

| Symbol | Description |
| --- | --- |
| `.isFinished: Boolean` (read-only from outside) | True once the path has reached and traversed this waypoint. |
| `.setTraversed()` | Sets `isFinished = true` (overrides `PointTurnWaypoint.setTraversed`, does not touch `hasTraversed`). |
| `.setAction(newAction)` | Always throws `IllegalArgumentException` — an end waypoint's action (marking the path finished) can't be replaced. |
| `.reset()` | Clears `isFinished` (plus the inherited reset chain). |
| `.type` | `END`. |

#### `actions/InterruptAction`

`actions/InterruptAction.kt` — the action an `InterruptWaypoint` performs once it's reached.

```kotlin
fun interface InterruptAction {
    fun doAction()
}
```

#### `actions/TriggeredAction`

`actions/TriggeredAction.kt` — an action a `Path` fires once `isTriggered()` becomes true (e.g.
"robot crossed Y=2m"), independent of any specific waypoint. Registered on a path via
`Path.addTriggeredActions`.

```kotlin
abstract class TriggeredAction
```

| Symbol | Description |
| --- | --- |
| `.loop()` | Polled every `Path.loop()` call; fires `doAction(alreadyPerformed)` once `isTriggered()` is true, then remembers it fired. |
| `.reset()` | Clears the "already performed" flag. |
| `.isTriggered(): Boolean` (abstract) | The trigger condition to implement. |
| `.doAction(alreadyPerformed: Boolean)` (abstract) | The action to implement; `alreadyPerformed` is true if this has already fired once before (e.g. to only act on the first crossing). |

#### `PathMotionProfile`

`PathMotionProfile.kt` — shapes how a `Path` speeds up leaving a waypoint and slows down
approaching one. Implement `accelerate`/`decelerate` for a custom profile; set it on a path via
`Path.setMotionProfile` or library-wide via `Path.setDefaultMotionProfile`. `Path` ships an internal
default: full configured speed until within 0.15 units of the target, then a linear ramp scaled by
`(distance * 10 + 0.1)`.

```kotlin
abstract class PathMotionProfile
```

| Symbol | Description |
| --- | --- |
| `.processDecelerate(motorSpeeds, distanceToTarget, configuredMovementSpeed, configuredTurnSpeed)` | Called by `Path` every loop while approaching a target; estimates robot speed from the previous call and relays to `decelerate`. |
| `.processAccelerate(motorSpeeds, distanceFromTarget, configuredMovementSpeed, configuredTurnSpeed)` | Called by `Path` every loop while leaving a waypoint; estimates speed and relays to `accelerate`. |
| `.decelerate(motorSpeeds, distanceToTarget, speed, configuredMovementSpeed, configuredTurnSpeed)` (abstract) | Scales `motorSpeeds` in place to slow the robot approaching a target. |
| `.accelerate(motorSpeeds, distanceFromTarget, speed, configuredMovementSpeed, configuredTurnSpeed)` (abstract) | Scales `motorSpeeds` in place to speed the robot up leaving a target. |

#### `DecelerationController`

`DecelerationController.kt` — a standalone deceleration profile, independent of `Path`'s own
`PathMotionProfile` hook — for custom deceleration logic used outside a `Path`.

```kotlin
abstract class DecelerationController
```

| Symbol | Description |
| --- | --- |
| `.process(motorSpeeds, distanceToTarget, configuredMovementSpeed, configuredTurnSpeed)` | Calls `decelerateMotorSpeeds` with the previous call's distance/timestamp, then records the new ones. |
| `.decelerateMotorSpeeds(motorSpeeds, distanceToTarget, lastDistanceToTarget, timeSinceLastCallNanos, configuredMovementSpeed, configuredTurnSpeed)` (abstract) | Scales `motorSpeeds` in place; `lastDistanceToTarget`/`timeSinceLastCallNanos` are `-1` on the first call, and can otherwise be used to estimate speed. |

#### `PurePursuitUtil`

`PurePursuitUtil.kt` — standalone math helpers behind `Path`'s pure-pursuit algorithm.

```kotlin
object PurePursuitUtil
```

| Symbol | Description |
| --- | --- |
| `.angleWrap(angle: Double): Double` | Wraps `angle` (radians) into `[-π, π]`. |
| `.isInFront(linePoint1, linePoint2, point1, point2: Translation2d): Boolean` | True if `point1` is farther along the line (`linePoint1` → `linePoint2`) than `point2`. Both points are assumed to already lie on the line. |
| `.positionEqualsWithBuffer(p1, p2: Translation2d, buffer: Double): Boolean` | True if `p1`/`p2` are within `buffer` of each other on both axes. |
| `.rotationEqualsWithBuffer(a1, a2: Double, buffer: Double): Boolean` | True if angles `a1`/`a2` (radians) are within `buffer` of each other. |
| `.moveToPosition(cx, cy, ca, tx, ty, ta: Double, turnOnly: Boolean): DoubleArray` | The raw `[strafe, forward, turn]` motor powers to drive from `(cx, cy, ca)` towards `(tx, ty, ta)` (angles in radians). If `turnOnly`, only the turn component is populated. |
| `.lineCircleIntersection(circleCenter: Translation2d, radius: Double, linePoint1, linePoint2: Translation2d): List<Translation2d>` | Every point where the line (`linePoint1` → `linePoint2`) crosses the circle centered at `circleCenter` with `radius`, bounded to the line segment. Ported from FTC team 11115 "Gluten Free" (via SolversLib). |

#### `Path`

`Path.kt` — a pure-pursuit path: an ordered list of `Waypoint`s (extends `ArrayList<Waypoint>`),
plus everything needed to turn "the robot's current position" into motor powers that follow them.
Call `init()` once before the first `loop()` call; call `loop()` every robot loop with the robot's
current pose to get the next `[strafe, forward, turn]` motor powers — or use `followPath` to run
the whole thing as a blocking loop.

```kotlin
class Path(waypoints: List<Waypoint> = emptyList()) : ArrayList<Waypoint>(waypoints)
// or:
Path(vararg waypoints: Waypoint)
```

A legal path needs at least 2 waypoints, must start with a `StartWaypoint`, must end with an
`EndWaypoint`, and must not contain either type anywhere else — `init()`/`isLegalPath()` enforce this.

| Symbol | Description |
| --- | --- |
| `.timedOut: Boolean` (read-only from outside) | Set once the path's overall timeout (`setPathTimeout`) or a waypoint's individual timeout elapses. |
| `.init()` | Verifies legality, resets state, has each waypoint `inherit` the previous one's settings (for `copyMode` waypoints), and marks init complete. Must be called before `loop()`. |
| `.followPath(drive: (Double, Double, Double) -> Unit, stop: () -> Unit, pose: () -> Pose2d, updateOdometry: () -> Unit): Boolean` | Runs `init()` itself, then blocks in a loop calling `pose()`/`loop()`/`drive(...)`/`updateOdometry()` each iteration until the path finishes (`stop()` is called, returns `true`) or times out / loses the path with retrace disabled (returns `false`). |
| `.loop(vPosition, hPosition, rotation: Double): DoubleArray` | The principal path method: given the robot's current pose, returns `[strafe, forward, turn]` motor powers to follow the path. All-zero powers mean the path timed out, lost the path (retrace disabled), or reached the destination — check `isFinished()`/`timedOut` to tell which. Throws `IllegalStateException` if called before `init()`. |
| `.setPathTimeout(milliseconds: Long)` | Sets an overall timeout for the path; if it doesn't finish in time, it aborts. Returns `this`. |
| `.setPathType(type: PathType)` | Not recommended unless you know what you're doing — default is `WAYPOINT_ORDERING_CONTROLLED`. Returns `this`. |
| `.setMotionProfile(profile: PathMotionProfile)` | Overrides the path's acceleration/deceleration shaping. Returns `this`. |
| `.setWaypointTimeouts(vararg timeouts: Long)` | Sets the first `timeouts.size` waypoints' individual timeouts. Returns `this`. |
| `.setWaypointTimeouts(timeout: Long)` | Sets every waypoint's individual timeout to the same value (not recommended). Returns `this`. |
| `.setRetraceSettings(movementSpeed, turnSpeed: Double)` | Configures how fast the robot retraces its path after losing it (both coerced to `[0, 1]`, default `1.0` each). Returns `this`. |
| `.resetTimeouts()` | Clears `timedOut` and resets the internal waypoint timestamp. Returns `this`. |
| `.enableRetrace()` / `.disableRetrace()` | Toggles whether the robot retraces its moves to try to re-find the path after losing it (retrace enabled by default). Returns `this`. |
| `.addTriggeredActions(vararg actions: TriggeredAction)` | Registers actions polled every `loop()`. Returns `this`. |
| `.removeTriggeredAction(action: TriggeredAction)` / `.clearTriggeredActions()` | Unregister one/all triggered actions. Returns `this`. |
| `.isLegalPath(): Boolean` | True if this path has ≥2 waypoints, starts with a start waypoint, ends with an end waypoint, and has neither type anywhere else. |
| `.isFinished(): Boolean` | Whether the last waypoint (expected to be an `EndWaypoint`) reports `isFinished`. |
| `.reset()` | Resets every waypoint, timeout, and triggered action to its initial state. Called by `init()`. |
| `.setDefaultMotionProfile(profile: PathMotionProfile)` (companion) | Sets the library-wide default motion profile new `Path`s use if `setMotionProfile` is never called. |

Internally, `loop()` finds every point where the robot's look-ahead circle (radius =
`followDistance`) crosses each path segment, picks the "best" one per `PathType`, dispatches to
per-waypoint-type handling (general curve-through vs. point-turn vs. interrupt/end), applies the
motion profile, and normalizes the final motor speeds. If no intersections are found and retrace is
enabled, it drives straight back toward the last known intersection point instead.

### `commands/`

A command-based robot framework (a port of WPILib's/SolversLib's command-based pattern) built
around three pieces: a **[`Command`](#commandkt)** state machine, a **[`Subsystem`](#subsystemkt)**
that a command can claim exclusive use of, and the **[`CommandScheduler`](#commandschedulerkt)**
singleton that runs everything each robot loop.

#### `Command.kt`

The core interface. Every command is a small state machine driven by the scheduler:
`initialize()` once, `execute()` every tick while scheduled, then `end(interrupted)` once
`isFinished()` returns `true` (or the command is interrupted/canceled). A command declares the
[`Subsystem`](#subsystemkt)s it needs exclusive use of via `requirements`; the scheduler won't run
two commands that share a requirement at the same time.

| Symbol | Description |
| --- | --- |
| `fun initialize()` | Called once when scheduled. Default no-op. |
| `fun execute()` | Called every tick while scheduled. Default no-op. |
| `fun end(interrupted: Boolean)` | Called once when the command finishes or is interrupted/canceled. Default no-op. |
| `fun isFinished(): Boolean` | Once `true`, the scheduler calls `end(false)` and un-schedules the command. Default `false` (runs forever). |
| `val requirements: Set<Subsystem>` | Subsystems this command needs exclusive use of. Prefer a stored field over allocating a new set each call. |
| `fun hasRequirement(requirement: Subsystem): Boolean` | `requirements.contains(requirement)`. |
| `fun runsWhenDisabled(): Boolean` | Whether the command should still run while `Robot.isDisabled`. Default `false`. |
| `val name: String` | Defaults to the class's simple name. |
| `fun schedule(interruptible: Boolean = true)` | `CommandScheduler.schedule(interruptible, this)`. |
| `fun cancel()` | `CommandScheduler.cancel(this)`. |
| `fun isScheduled(): Boolean` | `CommandScheduler.isScheduled(this)`. |
| `fun withTimeout(millis: Long): Command` | `ParallelRaceGroup(this, WaitCommand(millis))` — interrupts/un-schedules this command if it's still running after `millis`. |
| `fun interruptOn(condition: () -> Boolean): Command` | `ParallelRaceGroup(this, WaitUntilCommand(condition))` — interrupts as soon as `condition` becomes true. |
| `fun whenFinished(toRun: () -> Unit): Command` | `SequentialCommandGroup(this, InstantCommand(toRun))` — runs `toRun` once this command finishes. |
| `fun beforeStarting(toRun: () -> Unit): Command` | `SequentialCommandGroup(InstantCommand(toRun), this)` — runs `toRun` once before this command starts. |
| `fun beforeStarting(command: Command): Command` | `SequentialCommandGroup(command, this)` — runs `command` to completion, then this command. |
| `fun andThen(vararg next: Command): Command` | This command, then `next` in sequence (`SequentialCommandGroup`). |
| `fun deadlineWith(vararg parallel: Command): Command` | `ParallelDeadlineGroup(this, *parallel)` — this command as the deadline. |
| `fun alongWith(vararg parallel: Command): Command` | This command alongside `parallel`, ending once every one has (`ParallelCommandGroup`). |
| `fun raceWith(vararg parallel: Command): Command` | This command racing `parallel`, ending (and interrupting the rest) as soon as any one finishes (`ParallelRaceGroup`). |
| `fun perpetually(): Command` | `PerpetualCommand(this)` — ignores this command's own end condition. |
| `fun asProxy(): Command` | `ProxyScheduleCommand(this)` — runs "by proxy" so a containing command group doesn't inherit its requirements. |
| `fun uninterruptible(): Command` | `UninterruptibleCommand(this)` — refuses interruption by another command sharing a requirement. |
| `fun whenActive(condition: () -> Boolean, runnable: () -> Unit): Command` | `CallbackCommand(this)` that runs `runnable` the first time `condition` becomes true while this command is active. |
| `fun whenActive(condition: () -> Boolean, command: Command): Command` | Same, but schedules `command` instead of calling a runnable. |

#### `CommandBase.kt`

Abstract base for `Command` implementations: tracks a mutable `name` and `subsystemGroup` label,
plus a backing `requirementsSet` (exposed read-only as `requirements`).

| Symbol | Description |
| --- | --- |
| `abstract class CommandBase : Command` | |
| `var name: String` | Defaults to the class's simple name; overrides `Command.name`. |
| `var subsystemGroup: String` | Defaults to `"Ungrouped"`. |
| `fun addRequirements(vararg requirements: Subsystem): CommandBase` | Adds to `requirementsSet`; returns `this`. |
| `fun setName(newName: String): CommandBase` | Sets `name`; returns `this` (SolversLib-style method-call API). |
| `fun setSubsystem(newSubsystem: String): CommandBase` | Sets `subsystemGroup`; returns `this`. |

#### `CommandGroupBase.kt`

Abstract base for command groups (`SequentialCommandGroup`, `ParallelCommandGroup`,
`ParallelRaceGroup`, `ParallelDeadlineGroup`). Statically tracks every command that's been added to
*any* group, in a `WeakHashMap`-backed set, so a grouped command can't also be scheduled
independently (which would leave its state inconsistent between the two).

| Symbol | Description |
| --- | --- |
| `abstract class CommandGroupBase : CommandBase()` | |
| `abstract fun addCommands(vararg commands: Command)` | Adds `commands` to this group. |
| `CommandGroupBase.clearGroupedCommands()` | Frees every grouped command to be scheduled independently again. Use with care. |
| `CommandGroupBase.clearGroupedCommand(command: Command)` | Frees just `command`. |
| `CommandGroupBase.requireUngrouped(vararg commands: Command)` / `requireUngrouped(commands: Collection<Command>)` | Throws if any of `commands` already belongs to another group. |

#### `CommandState.kt`

`class CommandState(val isInterruptible: Boolean)` — the scheduler's per-scheduled-command
bookkeeping record (currently just the interruptible flag).

#### `CommandScheduler.kt`

Singleton `object` that runs everything: schedules/executes/ends commands, arbitrates subsystem
requirements, polls button bindings, and runs default commands. Call `run()` once per robot loop.

| Symbol | Description |
| --- | --- |
| `fun schedule(interruptible: Boolean = true, vararg commands: Command)` | Schedules each command if its requirements are free, or (if all current owners are themselves interruptible) by canceling those owners first. No-op for a command that's already scheduled, disabled while `Robot.isDisabled` and `!runsWhenDisabled()`, or part of a command group. Called mid-iteration (`inRunLoop`), it's deferred to the end of the current `run()`. |
| `fun run()` | One scheduler iteration: runs every registered subsystem's `periodic()`, polls button bindings, calls `execute()` on every scheduled command (ending+removing finished ones, and interrupting/removing any that shouldn't run while disabled), processes any schedule/cancel calls deferred from mid-iteration, then schedules each subsystem's default command if nothing else currently requires it. No-op while the scheduler itself is `disable()`d. |
| `fun registerSubsystem(vararg subsystems: Subsystem)` | Registers subsystems so `periodic()` runs and a default command can be scheduled for them. |
| `fun unregisterSubsystem(vararg subsystems: Subsystem)` | |
| `fun setDefaultCommand(subsystem: Subsystem, defaultCommand: Command)` | Requires `subsystem in defaultCommand.requirements` and that it never finishes; auto-scheduled whenever nothing else requires `subsystem`. |
| `fun getDefaultCommand(subsystem: Subsystem): Command?` | |
| `fun getScheduledCommands(): List<Command>` | Commands scheduled directly by the scheduler (nested commands inside a running group aren't included). |
| `fun cancel(vararg commands: Command)` | Interrupts and un-schedules `commands`, even non-interruptible ones; calls `end(interrupted = true)`. Deferred like `schedule` if called mid-iteration. |
| `fun cancelAll()` | Cancels every currently scheduled command. |
| `fun isScheduled(vararg commands: Command): Boolean` | Whether every one of `commands` is directly scheduled. |
| `fun requiring(subsystem: Subsystem): Command?` | The command currently owning `subsystem`, or `null` if free. |
| `fun isAvailable(subsystem: Subsystem): Boolean` | `requiring(subsystem) == null`. |
| `fun disable()` / `fun enable()` | Stops/resumes `run()` entirely (distinct from `Robot.isDisabled`, which only affects individual commands' `runsWhenDisabled()`). |
| `fun reset()` | Clears all scheduler state (scheduled commands, requirement owners, subsystems, buttons, callbacks). Mainly for between OpModes/tests. |
| `fun addButton(button: () -> Unit)` / `fun clearButtons()` | Registers a callback polled every `run()` (used by `Trigger`/`Button` bindings). |
| `fun setBulkReading(hardwareMap: HardwareMap, cachingMode: LynxModule.BulkCachingMode)` | Sets every hub's bulk-caching mode; if `MANUAL`, clears each hub's bulk cache once per `run()`. |
| `fun onCommandInitialize/Execute/Interrupt/Finish(action: (Command) -> Unit)` | Registers a callback fired on the corresponding lifecycle event for every command the scheduler runs. |

#### `Subsystem.kt` / `SubsystemBase.kt`

`Subsystem` encapsulates a piece of hardware for commands to claim via `requirements`.
`SubsystemBase` is a convenience base that auto-`register()`s with the `CommandScheduler` in its
`init` block and tracks a display `name` (`subsystemGroup` is a `name`-backed alias, for
SolversLib-API compatibility).

| Symbol | Description |
| --- | --- |
| `fun periodic()` | Called every scheduler tick (via `CommandScheduler.run()`), for subsystem-internal state that shouldn't live in a command. Default no-op. |
| `fun setDefaultCommand(defaultCommand: Command)` | `CommandScheduler.setDefaultCommand(this, defaultCommand)`. |
| `fun defaultCommand(): Command?` / `fun currentCommand(): Command?` | This subsystem's default command / the command currently requiring it. |
| `fun register()` | `CommandScheduler.registerSubsystem(this)`. |
| `fun runOnce(action: () -> Unit): Command` | `InstantCommand(action, this)`. |
| `fun run(action: () -> Unit): Command` | `RunCommand(action, this)`. |
| `fun startEnd(start: () -> Unit, end: () -> Unit): Command` | `StartEndCommand(start, end, this)`. |
| `fun runEnd(run: () -> Unit, end: () -> Unit): Command` | `FunctionalCommand({}, run, { end() }, { false }, this)` — runs `run` every tick, `end` on interruption/cancel, never finishes on its own. |
| `fun startRun(start: () -> Unit, run: () -> Unit): Command` | `FunctionalCommand(start, run, {}, { false }, this)` — runs `start` once then `run` every tick, never finishes on its own. |
| `fun defer(supplier: () -> Command): Command` | `DeferredCommand(supplier, setOf(this))`. |
| `class SubsystemBase : Subsystem` | `var name: String` (defaults to the class's simple name), `var subsystemGroup: String` (get/set aliasing `name`). Registers itself in `init`. |

#### `CommandOpMode.kt`

`abstract class CommandOpMode : LinearOpMode()` — drives the `CommandScheduler` for a whole
`LinearOpMode` lifecycle so you don't hand-roll the loop: `initialize()` → repeated
`initializeLoop()` while in init → `preRun()` once → repeated `run()` (i.e.
`CommandScheduler.run()`) while active → `end()`, with `reset()` (`CommandScheduler.reset()`)
always run afterward in a `finally`.

| Symbol | Description |
| --- | --- |
| `abstract fun initialize()` | Runs once at OpMode init. |
| `open fun initializeLoop()` | Runs repeatedly during init, like `LinearOpMode`'s own init loop. Default no-op. |
| `open fun preRun()` | Runs once, after init but before the OpMode goes active. Default no-op. |
| `open fun run()` | `CommandScheduler.run()`; called repeatedly while active. Overridable. |
| `open fun end()` | Runs once the OpMode is no longer active. Default no-op. |
| `fun reset()` | `CommandScheduler.reset()`. |
| `fun schedule(vararg commands: Command)` | `CommandScheduler.schedule(true, *commands)`. |
| `fun register(vararg subsystems: Subsystem)` | `CommandScheduler.registerSubsystem(*subsystems)`. |
| `CommandOpMode.disable()` / `CommandOpMode.enable()` | Companion-object shortcuts for `Robot.disable()`/`Robot.enable()`. |

#### `Robot.kt`

`object Robot` — a thin convenience wrapper around the `CommandScheduler` singleton, plus the
`isDisabled` flag it checks (independent of `CommandScheduler`'s own `disable()`/`enable()`, which
stops the scheduler outright rather than flagging individual commands).

| Symbol | Description |
| --- | --- |
| `var isDisabled: Boolean` | Checked by the scheduler against each command's `runsWhenDisabled()`. |
| `fun reset()` / `fun run()` | Forward to `CommandScheduler.reset()` / `.run()`. |
| `fun schedule(vararg commands: Command)` | `CommandScheduler.schedule(true, *commands)`. |
| `fun register(vararg subsystems: Subsystem)` | `CommandScheduler.registerSubsystem(*subsystems)`. |
| `fun disable()` / `fun enable()` | Sets/clears `isDisabled`. |
| `fun setBulkReading(hardwareMap: HardwareMap, cachingMode: LynxModule.BulkCachingMode)` | `CommandScheduler.setBulkReading(...)`. |

---

##### Composite & decorator commands

Small, well-understood composition patterns. Each wraps one or more component commands and forwards
the lifecycle calls; unless noted, they require the union of their components' requirements and
register those components with `CommandGroupBase` so they can't also be scheduled independently.

| File | Description |
| --- | --- |
| `SequentialCommandGroup(vararg commands: Command)` | Runs `commands` one after another (initializing the next only as the previous finishes); finishes once the last one has. `addCommands` appends more (only while not running). `currentCommandName: String` exposes the currently-running component's name. `runsWhenDisabled()` is true only if every component is. |
| `ParallelCommandGroup(vararg commands: Command)` | Runs `commands` simultaneously; each is `end(false)`'d individually as it finishes, and the group itself finishes once none are still running. If the group is interrupted, every still-running component is `end(true)`'d. Components may not share requirements with each other. |
| `ParallelRaceGroup(vararg commands: Command)` | Runs `commands` simultaneously; finishes (and `end(true)`'s every other still-running component) the moment *any one* finishes. |
| `ParallelDeadlineGroup(deadline: Command, vararg commands: Command)` | Runs `deadline` alongside `commands`; finishes (and interrupts whatever else is still running) as soon as `deadline` finishes, regardless of the others. `setDeadline(newDeadline: Command)` swaps the deadline (adding it to the group first if needed). |
| `ConditionalCommand(onTrue: Command, onFalse: Command, condition: () -> Boolean)` | Picks `onTrue` or `onFalse` when initialized (based on `condition()`) and runs the chosen one *through itself* — so it composes correctly nested inside another group. Requires the union of both branches' requirements; `runsWhenDisabled()` requires both branches to allow it. |
| `SelectCommand(commands: Map<Any, Command>, selector: () -> Any)` / `SelectCommand(toRun: () -> Command)` | Picks a command from a fixed map keyed by `selector()`'s result (logging an error and substituting `InstantCommand()` for an unknown key), or builds one fresh from `toRun` each time — runs it through itself like `ConditionalCommand`. The map form requires the union of every mapped command's requirements. |
| `RepeatCommand(command: Command, maxRepeatTimes: Int = 0, condition: (() -> Boolean)? = null)` | Restarts `command` every time it finishes until *this* command is interrupted, or (if given) until it has repeated `maxRepeatTimes` times or `condition()` becomes true. |
| `PerpetualCommand(command: Command)` | Runs `command` forever, ignoring its own `isFinished()` — only external interruption/cancellation stops it. Equivalent to `command.perpetually()`. |
| `ParallelDeadlineGroup`/`ParallelRaceGroup` via `Command.withTimeout`/`interruptOn` | See `Command.kt` above. |
| `UninterruptibleCommand(command: Command)` | Schedules `command` with `interruptible = false`; finishes once `command` is no longer scheduled. Wraps a single command — group several first if needed. Equivalent to `command.uninterruptible()`. |
| `ProxyScheduleCommand(vararg toSchedule: Command)` | Schedules `toSchedule` independently (*not* as this command's own requirements) when initialized; finishes once none of them are scheduled anymore, and cancels them all if it's itself interrupted. Equivalent to `command.asProxy()` for a single command. |
| `DeferredCommand(supplier: () -> Command, requirements: Collection<Subsystem> = emptySet())` | Builds the actual command by calling `supplier()` only when initialized (useful for runtime-dependent setup, e.g. building a trajectory mid-autonomous). `supplier` must return a *new* command each call — for picking among a fixed set instead, use `SelectCommand`. |
| `CallbackCommand<T : Command>(command: T)` | Schedules `command` on init, then lets you attach one-off callbacks that fire (and remove themselves) the first time a condition becomes true while it's running: `whenTrue`/`whenTrueSelf` (condition as `() -> Boolean`) and `whenSelf`/`whenSelf`/`whenSelf` (condition tested against the wrapped command, as `(T) -> Boolean`), each with a runnable-, `Command`-, or `(T) -> Unit`-consumer overload. Finishes once `command` is no longer scheduled. Backs `Command.whenActive`. |
| `StartEndCommand(onInit: () -> Unit, onEnd: () -> Unit, vararg requirements: Subsystem)` | Runs `onInit` once at start, `onEnd` once at end (interrupted or not) — e.g. spin up a motor, then stop it. No end condition of its own. |
| `FunctionalCommand(onInit, onExecute, onEnd, isFinishedFn, vararg requirements: Subsystem)` | Assembles a command from plain lambdas for each lifecycle method — handy for a one-off command not worth its own class. |
| `RunCommand(toRun: () -> Unit, vararg requirements: Subsystem)` | Runs `toRun` every `execute()`, with no end condition of its own — pair with `withTimeout`/`interruptOn`, or use `InstantCommand` for a one-shot. |
| `InstantCommand(toRun: () -> Unit = {}, vararg requirements: Subsystem)` | Runs `toRun` once in `initialize()` and finishes immediately (`isFinished()` is `final override` `true`) — initializes, executes*, and ends on the same scheduler tick (*`execute()` itself is the inherited no-op; all the work happens in `initialize()`). |
| `PrintCommand(message: String)` | `InstantCommand({ println(message) })` that also overrides `runsWhenDisabled()` to `true`. |
| `WaitCommand(millis: Long)` | Does nothing until `millis` milliseconds have elapsed since `initialize()` (measured via `System.nanoTime()`). Sets its own `name` to `"$name: $millis milliseconds"`. Runs even while disabled. |
| `WaitUntilCommand(condition: () -> Boolean)` | Does nothing until `condition()` returns `true`. Runs even while disabled. |

---

##### Button bindings — `button/`

`Trigger.kt` — links commands to a polled boolean condition (e.g. a gamepad button), by registering
callbacks with `CommandScheduler.addButton` that get polled every `run()`.

| Symbol | Description |
| --- | --- |
| `open class Trigger(private val isActive: () -> Boolean = { false })` | Subclass and override `get()` for anything beyond a plain condition (see `GamepadButton`). |
| `open fun get(): Boolean` | Whether the trigger is currently active. Default `isActive()`. |
| `fun whenActive(command: Command, interruptible: Boolean = true)` / `fun whenActive(toRun: () -> Unit)` | Schedules `command` (or an `InstantCommand(toRun)`) the moment the trigger goes inactive → active. |
| `fun whileActiveContinuous(command: Command, interruptible: Boolean = true)` / `(toRun: () -> Unit)` | Re-schedules `command` every tick the trigger is active, and cancels it the moment it isn't. |
| `fun whileActiveOnce(command: Command, interruptible: Boolean = true)` | Schedules `command` once the trigger becomes active, cancels it once inactive — never re-schedules while it stays active. |
| `fun whenInactive(command: Command, interruptible: Boolean = true)` / `(toRun: () -> Unit)` | Schedules the moment the trigger goes active → inactive. |
| `fun toggleWhenActive(command: Command, interruptible: Boolean = true)` | Toggles `command` on (schedule) / off (cancel) each time the trigger goes inactive → active. |
| `fun toggleWhenActive(commandOne: Command, commandTwo: Command, interruptible: Boolean = true)` / `(runnableOne, runnableTwo)` | Alternates between the two each time the trigger goes inactive → active. |
| `fun cancelWhenActive(command: Command)` | Cancels `command` the moment the trigger goes inactive → active. |
| `infix fun and(other: Trigger): Trigger` / `infix fun or(other: Trigger): Trigger` | A trigger active only when both are / active when either is. |
| `fun negate(): Trigger` | A trigger active exactly when this one isn't. |

`Button.kt` — `abstract class Button(isPressed: () -> Boolean = { false }) : Trigger(isPressed)`,
same behavior as `Trigger` renamed to fit a button's use case:

| Symbol | Trigger equivalent |
| --- | --- |
| `whenPressed(command/toRun)` | `whenActive` |
| `whileHeld(command/toRun)` | `whileActiveContinuous` |
| `whenHeld(command)` | `whileActiveOnce` |
| `whenReleased(command/toRun)` | `whenInactive` |
| `toggleWhenPressed(command)` / `(commandOne, commandTwo)` / `(runnableOne, runnableTwo)` | `toggleWhenActive` |
| `cancelWhenPressed(command)` | `cancelWhenActive` |

`GamepadButton.kt` — `class GamepadButton(gamepad: GamepadEx, private vararg val buttons: GamepadKeys.Button) : Button()`.
`get()` returns `true` only while **all** of `buttons` are held (`gamepad.getButton(it)` for each).

---

##### Trajectory-following & motion-profile commands

These wrap this library's own `math/` control/kinematics/trajectory types (`PIDController`,
`ProfiledPIDController`, `TrapezoidProfile`, `RamseteController`, `*DriveKinematics`, `Trajectory`,
`Pose2d`) as `Command`s — a from-scratch (non-RoadRunner) trajectory-following path, as opposed to
`RoadRunnerCommands.kt`'s `ActionCommand` bridge.

`OdometrySubsystem.kt`

| Symbol | Description |
| --- | --- |
| `class OdometrySubsystem(private val updatePose: () -> Pose2d, private val currentPose: () -> Pose2d) : SubsystemBase()` | Wraps odometry updates as a subsystem so `pose` refreshes every scheduler tick without a command needing to poll it explicitly. Takes plain lambdas rather than an `Odometry` instance directly, since `Odometry.update(gyroAngle, wheelPositions)` needs fresh sensor readings each tick — pass `{ odometry.update(gyro.heading, wheelPositions) }` as `updatePose`. |
| `val pose: Pose2d` | `currentPose()`. |
| `periodic()` | Calls `updatePose()`. |

`ProfiledPIDCommand.kt`

| Symbol | Description |
| --- | --- |
| `open class ProfiledPIDCommand(protected val controller: ProfiledPIDController, private val measurement: () -> Double, private val goal: () -> TrapezoidProfile.State, private val useOutput: (Double, TrapezoidProfile.State) -> Unit, vararg requirements: Subsystem) : CommandBase()` | Drives `useOutput` with `controller` tracking `goal`, every scheduler tick. Runs forever — subclass or `Command.withTimeout` it for an end condition. For a plain position (zero target velocity) goal, pass `{ TrapezoidProfile.State(targetPosition, 0.0) }`. |
| `initialize()` | `controller.reset(measurement())`. |
| `execute()` | `useOutput(controller.calculate(measurement(), goal()), controller.setpoint)`. |
| `end(interrupted)` | `useOutput(0.0, TrapezoidProfile.State())`. |

`TrapezoidProfileCommand.kt`

| Symbol | Description |
| --- | --- |
| `class TrapezoidProfileCommand(constraints: TrapezoidProfile.Constraints, private val goal: TrapezoidProfile.State, private val initial: TrapezoidProfile.State = TrapezoidProfile.State(), private val output: (TrapezoidProfile.State) -> Unit, vararg requirements: Subsystem) : CommandBase()` | Runs a `TrapezoidProfile(constraints)` from `initial` towards `goal`, piping each step's state to `output` (based on elapsed time since `initialize()`), until the profile completes (`profile.isFinished(elapsedSeconds)`). |

`MecanumControllerCommand.kt`

| Symbol | Description |
| --- | --- |
| `class MecanumControllerCommand(trajectory: Trajectory, pose: () -> Pose2d, kinematics: MecanumDriveKinematics, xController: PIDController, yController: PIDController, thetaController: ProfiledPIDController, maxWheelVelocityMetersPerSecond: Double, feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0), frontLeftController: PIDController? = null, rearLeftController: PIDController? = null, frontRightController: PIDController? = null, rearRightController: PIDController? = null, currentWheelSpeeds: (() -> MecanumDriveWheelSpeeds)? = null, outputDriveVoltages: ((MecanumDriveMotorVoltages) -> Unit)? = null, outputWheelSpeeds: ((MecanumDriveWheelSpeeds) -> Unit)? = null) : CommandBase()` | Follows `trajectory` with a mecanum drive: `xController`/`yController`/`thetaController` correct pose error against the sampled trajectory state each tick (the robot turns towards the trajectory's *final* heading throughout, not the per-sample heading), producing target wheel speeds via `kinematics`. Finishes once elapsed time exceeds `trajectory.totalTimeSeconds`. |
| PID mode (all four wheel `*Controller`s + `currentWheelSpeeds` + `outputDriveVoltages` given) | Adds `feedforward` + per-wheel velocity PID correction on top of the target wheel speeds, calling `outputDriveVoltages` with motor voltages. |
| Lighter-weight mode (wheel controllers/`currentWheelSpeeds` omitted) | Calls `outputWheelSpeeds` directly with the raw target wheel speeds instead. |

`RamseteCommand.kt`

| Symbol | Description |
| --- | --- |
| `class RamseteCommand(trajectory: Trajectory, pose: () -> Pose2d, follower: RamseteController, kinematics: DifferentialDriveKinematics, feedforward: SimpleMotorFeedforward? = null, wheelSpeeds: (() -> DifferentialDriveWheelSpeeds)? = null, leftController: PIDController? = null, rightController: PIDController? = null, output: (left: Double, right: Double) -> Unit) : CommandBase()` | Follows `trajectory` with a differential drive using `follower` (RAMSETE) to turn pose error into target wheel speeds via `kinematics`, calling `output(left, right)` every tick. Calls `output(0.0, 0.0)` on `end`. Finishes once elapsed time exceeds `trajectory.totalTimeSeconds`. |
| PID mode (`feedforward`/`wheelSpeeds`/`leftController`/`rightController` given) | Adds feedforward + per-side velocity PID correction before calling `output` with voltage-like values. |
| Lighter-weight mode (all four omitted) | `output` receives the raw target wheel speeds from RAMSETE directly (e.g. if a smart motor controller already does onboard velocity PID). |

`PurePursuitCommand.kt`

| Symbol | Description |
| --- | --- |
| `class PurePursuitCommand(driveRobotCentric: (strafe: Double, forward: Double, turn: Double) -> Unit, stop: () -> Unit, pose: () -> Pose2d, vararg waypoints: Waypoint) : CommandBase()` | Drives a mecanum-style robot along a pure-pursuit `Path` built from `waypoints`, every tick, until `path.isFinished()`. Takes `driveRobotCentric`/`stop` lambdas rather than a concrete drivebase type — pass `drive::driveRobotCentric`/`drive::stop` from an `alonlib.drives` class. Calls `stop()` on `end`. |
| `fun addWaypoint(waypoint: Waypoint)` / `fun addWaypoints(vararg waypoints: Waypoint)` | Adds to the underlying `Path`. |
| `fun removeWaypointAtIndex(index: Int)` | Removes from the underlying `Path`. |

---

#### `Extentions.kt`

Infix/extension sugar over this library's own `Command`:

| Symbol | Description |
| --- | --- |
| `Command until condition: () -> Boolean` | `this.raceWith(WaitUntilCommand { condition() })`. |
| `Command andThen next: Command` / `Command andThen next: () -> Command` | `this.andThen(next)`, or `this.andThen(next())` for the lazy form. |
| `Command finallyDo end: (interrupted: Boolean) -> Unit` | Wraps this command so `end` also runs `onEnd` after the command's own `end(interrupted)`. |
| `Command finallyDo command: Command` | Schedules `command` when this command ends, regardless of interruption. |
| `Command alongWith parallel: Command` | `this.alongWith(parallel)`. |
| `Command raceWith parallel: Command` | `this.raceWith(parallel)`. |
| `Command withTimeout seconds: Double` | `this.withTimeout((seconds * 1000.0).toLong())`. |
| `Command withName commandName: String` | `this.setName(commandName)` — for multi-subsystem commands; see `SubsystemBase.withName` for single-subsystem ones. |
| `withName(commandName: String, commandSupplier: () -> CommandBase): Command` | Builds a command from the supplier and names it `commandName`. |
| `SubsystemBase.withName(commandName: String, commandSupplier: () -> CommandBase): Command` | Same, but names it `"$commandName : ${this.name}"` (appends the owning subsystem's name). |

#### `Factories.kt`

| Symbol | Description |
| --- | --- |
| `wait(duration: Time): WaitCommand` | `WaitCommand(duration.asMilliseconds.toLong())` — a command that finishes after `duration` elapses. |
| `waitUntil(until: () -> Boolean): WaitUntilCommand` | A command that finishes once `until()` returns `true`. |
| `instantCommand(toRun: () -> Unit): InstantCommand` | Runs `toRun` once and finishes immediately. |
| `(() -> Unit).asInstantCommand: Command` | Same as `instantCommand`, as an extension property. **Requires no subsystems** — don't use it for an action that needs to claim one. |

#### `RoadRunnerCommands.kt`

Bridges a RoadRunner `Action` (e.g. a trajectory from `MecanumDrive.actionBuilder(...).build()`)
into a `Command`, so it can be scheduled as a default/triggered command or combined with
`SequentialCommandGroup` etc., instead of only being runnable via `Actions.runBlocking` in a plain
`LinearOpMode`.

| Symbol | Description |
| --- | --- |
| `class ActionCommand(action: Action, vararg requirements: Subsystem) : CommandBase` | Each `execute()` runs the action once (every loop, so trajectory following stays accurate) and sends the resulting `TelemetryPacket` to FTC Dashboard. `isFinished()` becomes `true` once `action.run(...)` returns `false`. |
| `Action.asCommand(vararg requirements: Subsystem): Command` | Shorthand for `ActionCommand(this, *requirements)`. |

## Running on a desktop emulator instead of a robot

`alonlib-emulator` lets your OpModes — real, unmodified `LinearOpMode`/`OpMode`/`CommandOpMode`
subclasses, exactly as they'll run on the robot — run against simulated hardware on your desktop
via [ftc-control-hub-emulator](https://github.com/alonHamb/ftc-control-hub-emulator), instead of a
physical REV Control/Expansion Hub. It backs every hardware type the emulator itself models —
`hardwareMap.get(...)` for `DcMotorEx`, `Servo`/`CRServo`, `TouchSensor`/`DigitalChannel`,
`AnalogInput`/`OpticalDistanceSensor`, `IMU`, `ColorSensor`/`NormalizedColorSensor`/
`DistanceSensor`/`CompassSensor`, and `LynxModule` (bulk data, input voltage) — with simulated
motor/servo dynamics and a simulated battery, and drives the OpMode lifecycle
(`init`/`start`/`loop`/`stop`) the same way the Driver Station does. Its regression tests —
hardware-map wiring for every device type above (`EmulatedDeviceTypesTest`), `LynxModule` bulk
data/voltage plus `HaMotor`/`HaServo` end-to-end (`RuntimeSmokeTest`), the `OpMode` lifecycle
harness (`OpModeHarnessSmokeTest`), `EmuTelemetry`'s own `Telemetry` behavior in isolation —
formatting, retained items, logs (`EmuTelemetryTest`), `HaServo` position-range edge cases
(`HaServoPositionRegressionTest`), and `EmulatorAutoLauncher`'s config discovery/OpMode
scanning/drive-wheel guessing (`EmulatorAutoLauncherTest`) — pass against the real FTC SDK classes
at runtime; see `alonlib-emulator/src/test`.

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
    testImplementation 'com.github.alonHamb.AlonLib:alonlib-emulator:<version>'
}
```

Replace `<version>` with the [latest release tag](https://github.com/alonHamb/AlonLib/tags), same as
[Installing](#installing), above (JitPack is already a repository if you're using AlonLib itself).

> **Note:** if a feature documented below isn't in the latest tag yet, depend on `master-SNAPSHOT`
> (or a specific commit hash) instead — see [Installing](#installing) for that syntax.

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

If your project doesn't have a hardware config XML at all (or you'd rather not depend on one), but
still want OpMode auto-discovery, construct `EmulatedRobot` yourself (from hand-declared
`EmulatedHub`s — see the worked example below) and pass it to `EmulatorAutoLauncher().launch(...)`
instead of calling the no-arg `launch()`:

```kotlin
class EmulatorMain {
    @Test
    fun launch() {
        val controlHub = EmulatedHub(HubId.CONTROL, motors = mapOf(0 to "front left motor"))
        EmulatorAutoLauncher().launch(EmulatedRobot(controlHub), title = "My Robot Emulator")
    }
}
```

This still classpath-scans for your `@TeleOp`/`@Autonomous` OpModes exactly like the no-arg
`launch()` does — only the hardware-config-XML lookup is skipped.

If OpMode auto-discovery itself doesn't fit either — multiple hardware config files, OpModes you
don't want auto-discovered, a non-mecanum drivetrain — construct `EmulatedRobot` yourself and call
its own `.launch(title, opModes)` directly instead; see the worked example below.

### API reference — `alonlib-emulator`

| Symbol | Description |
| --- | --- |
| `EmulatorAutoLauncher().launch()` | The zero-code entry point above. Also available as a plain `fun main()` in the same file, for the `JavaExec` task above. |
| `EmulatorAutoLauncher().launch(emulatedRobot: EmulatedRobot, title: String = "Emulator")` | Same OpMode auto-discovery as the no-arg `launch()`, but against a hardware map you already built yourself (e.g. an `EmulatedRobot` from hand-declared `EmulatedHub`s) instead of requiring a hardware config XML on disk. |
| `EmulatedHub(hub: HubId, motors: Map<Int, String> = emptyMap(), servos: Map<Int, String> = emptyMap(), digitalDevices: Map<Int, String> = emptyMap(), analogDevices: Map<Int, String> = emptyMap(), imus: Map<Int, String> = emptyMap(), i2cDevices: Map<Int, String> = emptyMap())` | One physical hub's worth of simulated devices, keyed by REV port index (or I2C bus, for `imus`/`i2cDevices`) — matching how you'd describe a real robot's wiring. `.motors`/`.servos`/`.digitalDevices`/`.analogDevices`/`.imus`/`.i2cDevices` expose the underlying `Sim*` devices (for advancing sim time or setting readings in tests, e.g. `.update(dt)`/`.setReading(...)`); `.devices` lists all of them. Only needed if you're wiring `EmulatedRobot` by hand instead of using `EmulatorAutoLauncher`. |
| `buildEmulatedHardwareMap(controlHub: EmulatedHub, expansionHub: EmulatedHub? = null, batteryVoltage: () -> Double): HardwareMap` | Builds a real `HardwareMap` pre-populated with each hand-declared hub's devices, so `hardwareMap.get(DcMotorEx::class.java/Servo::class.java/LynxModule::class.java, ...)` all work exactly as they would against real hardware. |
| `buildEmulatedHardwareMap(simulatedRobot: emulator.config.SimulatedRobot, batteryVoltage: () -> Double): HardwareMap` | Same, but built straight from a `SimulatedRobot` (i.e. `emulator.config.buildSimulatedRobot(parseRobotConfigXml(...))`) instead of hand-declared hubs — what `EmulatorAutoLauncher` uses under the hood. |
| `EmulatedRobot(controlHub: EmulatedHub, expansionHub: EmulatedHub? = null, driveWheels: DriveWheels? = null)` | Ties one or two hand-declared `EmulatedHub`s to the emulator UI and drives whichever OpMode is selected through the real OpMode lifecycle. `.hardwareMap` is the resulting fake `HardwareMap`. `DriveWheels(frontLeft, frontRight, backLeft, backRight)` is optional and only powers the emulator's live field-pose display. |
| `EmulatedRobot(simulatedRobot: emulator.config.SimulatedRobot, driveWheels: DriveWheels? = null)` | Same, but built straight from a `SimulatedRobot` — what `EmulatorAutoLauncher` uses under the hood. |
| `EmulatedRobot.launch(title: String, opModes: Map<String, () -> OpMode>)` | Blocks the calling thread, showing the emulator window, until it's closed. Each map entry is a name shown in the OpMode dropdown → a factory for a fresh instance (matching how the real SDK constructs a new instance on every Init). Same headless-JVM caveat as `EmulatorAutoLauncher` above applies here too. |
| `EmuDcMotorEx(sim: SimMotor) : DcMotorEx` | A `DcMotorEx` backed by a simulated motor — real code that talks to `DcMotor`/`DcMotorEx` directly, or via `HaMotor` (which wraps one), runs unmodified against simulated dynamics. PID/current-alert configuration is accepted but not modeled, since `HaMotor` runs its own software PIDF loop and writes plain power/voltage. |
| `emulatedServo(controller: EmuServoController, port: Int): ServoImplEx` | Builds a genuine `ServoImplEx` for one hub port, needed because `HaServo` unconditionally force-casts `Servo` to `ServoImplEx`. |
| `EmuServoController(portsToSims: Map<Int, SimServo>) : ServoControllerEx` | Backs a hub's worth of simulated servos — the `ServoControllerEx` a real `ServoImplEx` delegates every operation to. |
| `EmuCRServo(sim: SimServo) : CRServo` | A `CRServo` backed by the same `SimServo` a positional `Servo` adapter uses for that device name — the config format doesn't distinguish the two either, so both get registered and whichever interface your OpMode asks for is what it gets. Only tracks the last commanded power (real CR servos have no position feedback); doesn't drive `SimServo`'s positional slew dynamics. |
| `emulatedLynxModule(motorsByPort: Map<Int, SimMotor>, batteryVoltage: () -> Double): LynxModule` | A Mockito-backed `LynxModule` whose bulk-read motor data and input voltage come from simulated motors/battery instead of a real REV hub over USB — `LynxModule` has no way to be constructed directly. |
| `EmuTouchSensor(sim: SimDigitalDevice) : TouchSensor`, `EmuDigitalChannel(sim: SimDigitalDevice) : DigitalChannel` | Both read/write the same `SimDigitalDevice.state`, registered under the same device name — set it directly from your test to simulate a sensor input, or read it back after your OpMode writes it as an output. |
| `emulatedAnalogInput(sim: SimAnalogDevice): AnalogInput`, `EmuOpticalDistanceSensor(sim: SimAnalogDevice) : OpticalDistanceSensor` | Both read the same `SimAnalogDevice.voltage` (0-3.3V), registered under the same device name. `AnalogInput` is a concrete SDK class (like `ServoImplEx`), so `emulatedAnalogInput` only overrides `getDeviceName()` to avoid its real implementation's `AppUtil` crash. |
| `EmuImu(sim: SimImu) : IMU` | Reports `SimImu.headingRad` as yaw (pitch/roll always read zero, matching `SimImu`'s own shape), relative to whatever heading was set the last time `resetYaw()` was called — mirror your simulated chassis's heading onto `SimImu.headingRad` in your `onTick` the way a real IMU tracks the robot. |
| `EmuColorSensor(sim: SimI2cDevice) : NormalizedColorSensor, ColorSensor, DistanceSensor` | Reads `sim.getReading("red"/"green"/"blue"/"alpha")` (normalized `[0,1)`) and `sim.getReading("distanceMm")` — matches a real REV Color Sensor V3, which `HaColorSensor` force-casts to both `ColorSensor` and `DistanceSensor`. |
| `EmuCompassSensor(sim: SimI2cDevice) : CompassSensor` | Reads `sim.getReading("headingDeg")`, normalized to `[0, 360)`. Registered under the same device name as `EmuColorSensor` for every generic I2C device, since the config format can't tell which one a real sensor actually is. |
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

    val motor = HaMotor(hardwareMap, "front left motor", Data.Motors.GoBILDA.RPM_435)
    motor.percentOutput = 0.5
    motor.update()
    controlHub.motors.getValue(0).update(0.5) // advance simulated dynamics by 0.5s

    assertTrue(motor.position.degrees != 0.0)
}
```

### Known limitations

- **Anything that reaches `AppUtil`/a real Android `Context`** — `FtcDashboard.getInstance()`,
  camera-based vision pipelines (`VisionPortal`/EasyOpenCV), vendor-specific sensor drivers that do
  their own I2C bring-up (`HaLimelight3A`, `HaPinPoint`, `HaOctoQuad`, `HaOTOS`, `HaHuskyLens`, ...)
  — isn't emulated and will throw or crash, since there's no real Android runtime underneath. Guard
  those calls (e.g. behind a flag) if your OpMode uses them, the way `BlueMainTeleop`/`RedMainTeleop`
  in a Decode-Robot-shaped project need to for `FtcDashboard.getInstance()`.
- **Everything `SimulatedRobot`/`EmulatedHub` can represent does back a real `HardwareMap` type**:
  `DcMotorEx`, `Servo`/`CRServo`, `TouchSensor`/`DigitalChannel`, `AnalogInput`/
  `OpticalDistanceSensor`, `IMU`, `ColorSensor`/`NormalizedColorSensor`/`DistanceSensor`,
  `CompassSensor`, and `LynxModule` (bulk data, input voltage). A generic I2C sensor tag your config
  doesn't specifically name (a color sensor, a distance sensor, a compass, ...) gets *both* an
  `EmuColorSensor`-shaped adapter and an `EmuCompassSensor` adapter under the same device name,
  since the config format alone can't tell which one a real sensor is — drive whichever
  `SimI2cDevice.setReading(...)` keys match the SDK interface your OpMode actually asks for
  (`"red"`/`"green"`/`"blue"`/`"alpha"`, `"distanceMm"`, or `"headingDeg"` — see `EmuColorSensor`/
  `EmuCompassSensor`'s doc comments). None of these model real sensor dynamics (no color/distance
  physics, no IMU drift) -- they're values your test/adapter code drives directly, same as the
  underlying `SimDigitalDevice`/`SimAnalogDevice`/`SimImu`/`SimI2cDevice` they're backed by. Webcams
  and other USB devices aren't adapted to any SDK interface (no camera frames are simulated either,
  so there'd be nothing for a vision pipeline to see).
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
</content>
