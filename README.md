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
    implementation 'com.github.alonHamb:AlonLib:v1.0.1'
}
```

Use a specific tag (recommended, e.g. `v1.0.1`), a commit hash, or `<branch>-SNAPSHOT` to track a
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

## Building locally

```bash
./gradlew :alonlib:assembleRelease
```
