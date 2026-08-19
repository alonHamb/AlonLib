# AlonLib

A Kotlin utility library for FTC (*FIRST* Tech Challenge) robot code: unit-safe wrappers for
motors/servos/sensors, a units system (`Length`, `AngularVelocity`, `Rotation2d` extensions),
PID/feedforward helpers, moving-window filters, and SolversLib command-based extensions.

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
    implementation 'com.github.alonHamb:AlonLib:v1.1.0'
}
```

Use a specific tag (recommended, e.g. `v1.1.0`), a commit hash, or `<branch>-SNAPSHOT` to track a
branch directly.

## Requirements

The FTC SDK (`org.firstinspires.ftc:*`, v11.1.0) and SolversLib (`org.solverslib:core`) are bundled
as `api` dependencies, so they come along transitively — you don't need to declare either yourself
just to use AlonLib's classes. If your project also depends on `FtcRobotController` directly (the
usual FTC project setup, needed to actually build/run the robot controller app), Gradle will
de-duplicate the shared FTC SDK version automatically as long as the versions match.

## Structure

Everything lives under `alonlib/src/main/java/org/firstinspires/ftc/teamcode/alonlib`:

- `hardware/` — `HaMotor`, `HaServo`, `HaLimelight3A`, `HaPinPoint`
- `units/` — `Length`, `AngularVelocity`, `Rotation2d` extensions, unit conversions
- `math/` — PID/feedforward gains, moving-window filters, deadband/interpolation helpers
- `commands/` — SolversLib `Command` extensions and factories
- `drives/` — drivetrain subsystems

`alonlib-emulator/src/main/java` has the desktop-emulator adapters (`EmulatedRobot`, `EmulatedHub`,
`EmuDcMotorEx`, ...) — see "Running on a desktop emulator instead of a robot", below.

## Running on a desktop emulator instead of a robot

`alonlib-emulator` lets your OpModes — real, unmodified `LinearOpMode`/`OpMode`/`CommandOpMode`
subclasses, exactly as they'll run on the robot — run against simulated hardware on your desktop
via [ftc-control-hub-emulator](https://github.com/alonHamb/ftc-control-hub-emulator), instead of a
physical REV Control/Expansion Hub. It backs `hardwareMap.get(DcMotorEx::class.java, ...)`,
`hardwareMap.get(Servo::class.java, ...)`, and `hardwareMap.get(LynxModule::class.java, "Control
Hub")` with simulated motor/servo dynamics and a simulated battery, and drives the OpMode lifecycle
(`init`/`start`/`loop`/`stop`) the same way the Driver Station does.

**Only ever add this to your TeamCode module's `testImplementation`, never `implementation`** — it
pulls in Mockito and ftc-control-hub-emulator's Swing UI and JNA-based gamepad reading, none of
which belong in the APK that ships to the robot.

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
    testImplementation 'com.github.alonHamb:alonlib-emulator:v1.2.0'
}
```

(JitPack is already a repository if you're using AlonLib itself — see Installing, above.)

### A worked example

Put this in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/EmulatorMain.kt` (a local unit
test, run manually from your IDE's gutter/Run button — it opens a window and blocks, so it's not
meant to run as part of `./gradlew test`). This mirrors a `RobotMap` like:

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

## Building locally

```bash
./gradlew :alonlib:assembleRelease
```
