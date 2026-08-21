package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import emulator.config.buildSimulatedRobot
import emulator.config.parseRobotConfigXml
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.File

/**
 * Regression suite for [EmulatorAutoLauncher]'s zero-code pieces: finding the real hardware config
 * XML a project uploads to its Control Hub, discovering `@TeleOp`/`@Autonomous` OpModes the same
 * way the real Driver Station does, and best-effort mecanum drive-wheel matching by name.
 */
class EmulatorAutoLauncherTest {

    @get:Rule
    val tmp = TemporaryFolder()

    private val realisticConfigXml = """
        <Robot type="FirstInspires-FTC">
            <LynxUsbDevice name="Control Hub Portal" parentModuleAddress="173" serialNumber="(embedded)">
                <LynxModule name="Control Hub" port="173">
                    <Motor name="front left motor" port="0" />
                    <Motor name="front right motor" port="1" />
                    <Motor name="back left motor" port="2" />
                    <Motor name="back right motor" port="3" />
                    <Servo name="claw servo" port="0" />
                </LynxModule>
            </LynxUsbDevice>
        </Robot>
    """.trimIndent()

    // -- findHardwareConfigXml -------------------------------------------------------------

    @Test
    fun `finds the one xml under res slash xml that declares a LynxModule`() {
        val projectDir = tmp.newFolder("TeamCode")
        val xmlDir = File(projectDir, "src/main/res/xml").apply { mkdirs() }
        File(xmlDir, "unrelated.xml").writeText("<resources><string name=\"app_name\">Foo</string></resources>")
        val configFile = File(xmlDir, "my_robot.xml").apply { writeText(realisticConfigXml) }

        assertEquals(configFile, findHardwareConfigXml(projectDir))
    }

    @Test
    fun `searches parent directories when the working directory is nested`() {
        val projectDir = tmp.newFolder("TeamCode")
        val xmlDir = File(projectDir, "src/main/res/xml").apply { mkdirs() }
        val configFile = File(xmlDir, "my_robot.xml").apply { writeText(realisticConfigXml) }
        val nestedCwd = File(projectDir, "build/intermediates").apply { mkdirs() }

        assertEquals(configFile, findHardwareConfigXml(nestedCwd))
    }

    @Test
    fun `throws a clear error when no config xml is found`() {
        val emptyDir = tmp.newFolder("EmptyProject")
        val error = runCatching { findHardwareConfigXml(emptyDir) }.exceptionOrNull()
        assertTrue(error is IllegalStateException)
        assertTrue(error!!.message!!.contains("Couldn't find"))
    }

    @Test
    fun `throws a clear error when multiple config xmls are found`() {
        val projectDir = tmp.newFolder("TeamCode")
        val xmlDir = File(projectDir, "src/main/res/xml").apply { mkdirs() }
        File(xmlDir, "robot_a.xml").writeText(realisticConfigXml)
        File(xmlDir, "robot_b.xml").writeText(realisticConfigXml)

        val error = runCatching { findHardwareConfigXml(projectDir) }.exceptionOrNull()
        assertTrue(error is IllegalStateException)
        assertTrue(error!!.message!!.contains("multiple"))
    }

    // -- discoverOpModes ---------------------------------------------------------------------

    @Test
    fun `discovers annotated OpModes by declared name, falls back to class name, and skips Disabled`() {
        val opModes = discoverOpModes()

        assertTrue("Discoverable Teleop" in opModes)
        assertTrue("UnnamedAutonomous" in opModes)
        assertTrue("Should not appear" !in opModes)

        val instance = opModes.getValue("Discoverable Teleop")()
        assertTrue(instance is DiscoverableTeleop)
    }

    // -- guessDriveWheels ----------------------------------------------------------------------

    @Test
    fun `guesses the four mecanum drive wheels from a real config's motor names`() {
        val simulatedRobot = buildSimulatedRobot(parseRobotConfigXml(realisticConfigXml))

        val driveWheels = guessDriveWheels(simulatedRobot)

        assertTrue(driveWheels != null)
        assertEquals(simulatedRobot.motors.getValue("front left motor"), driveWheels!!.frontLeft)
        assertEquals(simulatedRobot.motors.getValue("front right motor"), driveWheels.frontRight)
        assertEquals(simulatedRobot.motors.getValue("back left motor"), driveWheels.backLeft)
        assertEquals(simulatedRobot.motors.getValue("back right motor"), driveWheels.backRight)
    }

    @Test
    fun `returns null when the four drive wheels can't be matched unambiguously`() {
        val nonDrivetrainXml = """
            <Robot type="FirstInspires-FTC">
                <LynxModule name="Control Hub" port="173">
                    <Motor name="arm motor" port="0" />
                    <Motor name="intake motor" port="1" />
                </LynxModule>
            </Robot>
        """.trimIndent()
        val simulatedRobot = buildSimulatedRobot(parseRobotConfigXml(nonDrivetrainXml))

        assertNull(guessDriveWheels(simulatedRobot))
    }

    // -- end to end: buildEmulatedHardwareMap(SimulatedRobot) ----------------------------------

    @Test
    fun `hardware map built from a SimulatedRobot resolves motors and servos by name`() {
        val simulatedRobot = buildSimulatedRobot(parseRobotConfigXml(realisticConfigXml))
        val hardwareMap = buildEmulatedHardwareMap(simulatedRobot) { 12.7 }

        val motor = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx::class.java, "front left motor")
        motor.power = 0.5
        simulatedRobot.motors.getValue("front left motor").update(0.5)
        assertTrue("expected the encoder to have moved", motor.currentPosition != 0)

        val servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo::class.java, "claw servo")
        assertTrue(servo != null)

        val hub = hardwareMap.get(com.qualcomm.hardware.lynx.LynxModule::class.java, "Control Hub")
        assertEquals(motor.currentPosition, hub.bulkData.getMotorCurrentPosition(0))
    }
}

// Top-level (not nested -- a nested `private class`'s constructor isn't reflectively accessible
// without setAccessible, and these need to look like ordinary user OpMode classes) fixtures for
// the discoverOpModes() test above.

@TeleOp(name = "Discoverable Teleop")
class DiscoverableTeleop : OpMode() {
    override fun init() {}
    override fun loop() {}
}

@Autonomous // no name set -- should fall back to the simple class name, like the real SDK does
class UnnamedAutonomous : LinearOpMode() {
    override fun runOpMode() {}
}

@TeleOp(name = "Should not appear")
@Disabled
class DisabledTeleop : OpMode() {
    override fun init() {}
    override fun loop() {}
}
