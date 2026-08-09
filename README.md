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
    implementation 'com.github.alonHamb:AlonLib:34e1cb8'
}
```

Use a specific tag (e.g. `v1.0.1`), a commit hash, or `<branch>-SNAPSHOT` to track a branch
directly. As of this writing the `v1.0.1` tag itself is pinned to a stale JitPack cache entry from
before the tag existed (see [jitpack.io/com/github/alonHamb/AlonLib](https://jitpack.io/com/github/alonHamb/AlonLib)
for current status) — the commit SHA above is confirmed building. Switch to the tag once it
resolves cleanly.

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
