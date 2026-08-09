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

Consumers must provide the FTC SDK and SolversLib on their own classpath (typically already the
case via the `FtcRobotController` module and `org.solverslib:core`) — this library declares them
as `compileOnly`/`api` and does not bundle them.

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
