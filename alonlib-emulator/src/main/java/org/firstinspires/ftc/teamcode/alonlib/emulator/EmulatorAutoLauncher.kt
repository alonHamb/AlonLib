package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import emulator.config.SimulatedRobot
import emulator.config.buildSimulatedRobot
import emulator.config.parseRobotConfigXml
import emulator.hardware.SimMotor
import io.github.classgraph.ClassGraph
import org.junit.Test
import java.io.File

/**
 * Zero-code entry point for the emulator. It builds the emulator's hardware straight from the
 * same hardware configuration XML file your project already uploads to the Control Hub (see
 * [findHardwareConfigXml]), and populates the OpMode dropdown by scanning your classpath for
 * `@TeleOp`/`@Autonomous`-annotated `OpMode` classes (see [discoverOpModes]) -- the same two
 * things the real Driver Station does, so adding a device to your config or an OpMode to your
 * project makes it show up here automatically too, with nothing to keep in sync by hand.
 *
 * **Run it from your IDE, not `./gradlew testDebugUnitTest`:** the [Test]-annotated [launch] below
 * is there so Android Studio/IntelliJ's own JUnit runner can run it with one click straight from
 * this class's sources (attached via alonlib-emulator's `withSourcesJar()`) -- Navigate to
 * `EmulatorAutoLauncher`, click the gutter arrow next to `launch()`. Running it through Gradle's
 * own `Test` task instead (whether via CLI or an IDE run configuration delegated to Gradle) won't
 * work: the Android Gradle Plugin forces `-Djava.awt.headless=true` on unit test JVMs, which makes
 * the emulator window silently never appear and the test hang until you kill it. For a CLI-runnable
 * command instead of an IDE click, add a `JavaExec` task pointed at [main] (see the README) -- a
 * `JavaExec` JVM isn't subject to that Android-unit-test default.
 *
 * If your robot doesn't fit the XML-based path -- multiple config files, OpModes you don't want
 * auto-discovered, non-mecanum drive -- construct [EmulatedRobot] yourself and pass it to the
 * [launch] overload below instead, which still gets you OpMode auto-discovery without needing a
 * hardware config XML on disk at all; see the README's worked example.
 */
class EmulatorAutoLauncher {
    @Test
    fun launch() {
        val configFile = findHardwareConfigXml()
        val simulatedRobot = buildSimulatedRobot(parseRobotConfigXml(configFile))

        launch(
            EmulatedRobot(simulatedRobot, driveWheels = guessDriveWheels(simulatedRobot)),
            title = "${configFile.nameWithoutExtension} Emulator"
        )
    }

    /**
     * Same OpMode auto-discovery as the no-arg [launch] above, but against a hardware map you
     * already built yourself -- e.g. an [EmulatedRobot] constructed from hand-declared
     * [EmulatedHub]s -- instead of requiring a hardware config XML file under `res/xml` to exist
     * on disk. Use this when your project doesn't have one, or you don't want auto-discovery to
     * depend on it, but still want OpModes picked up automatically.
     */
    fun launch(emulatedRobot: EmulatedRobot, title: String = "Emulator") {
        val opModes = discoverOpModes()

        require(opModes.isNotEmpty()) {
            "No @TeleOp/@Autonomous OpMode classes found on the classpath. Add one (and make sure " +
                "it isn't @Disabled)."
        }

        emulatedRobot.launch(title = title, opModes = opModes)
    }
}

/** Same as [EmulatorAutoLauncher.launch], as a plain `main()` for a `JavaExec` Gradle task -- see the README. */
fun main() {
    EmulatorAutoLauncher().launch()
}

/**
 * Finds the hardware configuration XML file REV Hardware Client / Driver Station wrote for this
 * project -- the same file your project uploads to the Control Hub, normally at
 * `src/main/res/xml/<config name>.xml`. Searches `res/xml` under [startDir] and up to four parent
 * directories (covers both a Gradle test task, whose working directory is the module root, and an
 * IDE run configuration, which usually defaults to the same place), picking out whichever `.xml`
 * files actually declare a `<LynxModule>` -- since `res/xml` can hold unrelated resources too --
 * and requiring exactly one match.
 */
internal fun findHardwareConfigXml(startDir: File = File(".")): File {
    var dir: File? = startDir.absoluteFile
    val searched = mutableListOf<File>()

    repeat(5) {
        val xmlDir = File(dir, "src/main/res/xml")
        searched += xmlDir
        if (xmlDir.isDirectory) {
            val candidates = xmlDir.listFiles { f -> f.extension == "xml" && f.readText().contains("<LynxModule") }.orEmpty()
            when (candidates.size) {
                1 -> return candidates[0]
                0 -> {}
                else -> error(
                    "Found multiple hardware config files in $xmlDir (${candidates.joinToString { it.name }}) -- " +
                        "EmulatorAutoLauncher needs exactly one. Construct EmulatedRobot yourself with the right " +
                        "one instead -- see the README."
                )
            }
        }
        dir = dir?.parentFile
    }

    error(
        "Couldn't find a hardware config XML (a res/xml/*.xml declaring <LynxModule>) in any of: " +
            searched.joinToString { it.path } +
            ". Run this from your TeamCode module (its working directory needs to be the module root), " +
            "or construct EmulatedRobot yourself -- see the README."
    )
}

/**
 * Scans the classpath for every `OpMode` subclass annotated `@TeleOp`/`@Autonomous` and not
 * `@Disabled` -- exactly what the real Driver Station's OpMode list is built from -- and maps each
 * one's declared name (or its simple class name, if the annotation didn't set one, matching the
 * real SDK's own fallback) to a factory that constructs a fresh instance.
 */
internal fun discoverOpModes(): Map<String, () -> OpMode> {
    val opModes = linkedMapOf<String, () -> OpMode>()

    ClassGraph().enableClassInfo().enableAnnotationInfo().scan().use { scanResult ->
        val annotated = scanResult.getClassesWithAnnotation(TeleOp::class.java.name) +
            scanResult.getClassesWithAnnotation(Autonomous::class.java.name)

        for (classInfo in annotated) {
            if (classInfo.hasAnnotation(Disabled::class.java.name)) continue

            val clazz = classInfo.loadClass()
            if (!OpMode::class.java.isAssignableFrom(clazz)) continue
            @Suppress("UNCHECKED_CAST")
            val opModeClass = clazz as Class<out OpMode>

            val declaredName = clazz.getAnnotation(TeleOp::class.java)?.name
                ?: clazz.getAnnotation(Autonomous::class.java)?.name
                ?: ""
            val displayName = declaredName.ifBlank { clazz.simpleName }

            opModes[displayName] = { opModeClass.getDeclaredConstructor().apply { isAccessible = true }.newInstance() }
        }
    }

    return opModes
}

/**
 * Best-effort match of [simulatedRobot]'s motors to a standard four-wheel mecanum layout, purely
 * by name (`front`/`left`/`right`/`back`/`rear`, case- and separator-insensitive -- so
 * `"frontLeftMotor"`, `"front_left_motor"`, and `"left front motor"` all match), so
 * [EmulatorAutoLauncher] can wire up the field-pose view without you having to say which motors are
 * drive wheels. Returns `null` (field view just won't move) if all four can't be found unambiguously.
 */
internal fun guessDriveWheels(simulatedRobot: SimulatedRobot): EmulatedRobot.DriveWheels? {
    fun find(position: List<String>, side: String): SimMotor? = simulatedRobot.motors.values.singleOrNull { motor ->
        val normalized = motor.name.lowercase().replace(Regex("[^a-z]"), "")
        position.any { normalized.contains(it) } && normalized.contains(side)
    }

    val frontLeft = find(listOf("front"), "left")
    val frontRight = find(listOf("front"), "right")
    val backLeft = find(listOf("back", "rear"), "left")
    val backRight = find(listOf("back", "rear"), "right")

    return if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
        EmulatedRobot.DriveWheels(frontLeft, frontRight, backLeft, backRight)
    } else {
        null
    }
}
