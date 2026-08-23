package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.CompassSensor
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.TouchSensor
import emulator.config.buildSimulatedRobot
import emulator.config.parseRobotConfigXml
import emulator.hardware.HubId
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Regression suite for the non-motor/servo-position device adapters (`EmuCRServo`,
 * `EmuTouchSensor`/`EmuDigitalChannel`, `emulatedAnalogInput`/`EmuOpticalDistanceSensor`, `EmuImu`,
 * `EmuColorSensor`/`EmuCompassSensor`) -- these back every device category `SimulatedRobot` resolves
 * that isn't a motor or servo, wired in by `wireDevices` inside `buildEmulatedHardwareMap`.
 */
class EmulatedDeviceTypesTest {

    private fun hub() = EmulatedHub(
        HubId.CONTROL,
        servos = mapOf(0 to "claw servo"),
        digitalDevices = mapOf(0 to "bumper"),
        analogDevices = mapOf(0 to "pot"),
        imus = mapOf(0 to "imu"),
        i2cDevices = mapOf(1 to "color sensor")
    )

    @Test
    fun `CRServo tracks last commanded power independent of the same-named Servo adapter`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }

        val crServo = hardwareMap.get(CRServo::class.java, "claw servo")
        crServo.power = 0.6
        assertEquals(0.6, crServo.power, 1e-9)

        crServo.direction = DcMotorSimple.Direction.REVERSE
        assertEquals(DcMotorSimple.Direction.REVERSE, crServo.direction)
    }

    @Test
    fun `TouchSensor and DigitalChannel both read the same underlying digital state`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }
        val sim = controlHub.digitalDevices.getValue(0)

        val touch = hardwareMap.get(TouchSensor::class.java, "bumper")
        val digital = hardwareMap.get(DigitalChannel::class.java, "bumper")

        assertFalse(touch.isPressed)
        sim.state = true
        assertTrue(touch.isPressed)
        assertEquals(1.0, touch.value, 1e-9)
        assertTrue(digital.state)

        digital.setState(false)
        assertFalse(sim.state)
        assertFalse(touch.isPressed)
    }

    @Test
    fun `AnalogInput and OpticalDistanceSensor both read the same underlying voltage`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }
        val sim = controlHub.analogDevices.getValue(0)

        sim.voltage = 1.65

        val analogInput = hardwareMap.get(AnalogInput::class.java, "pot")
        assertEquals(1.65, analogInput.voltage, 1e-9)
        assertEquals(3.3, analogInput.maxVoltage, 1e-9)

        val ods = hardwareMap.get(com.qualcomm.robotcore.hardware.OpticalDistanceSensor::class.java, "pot")
        assertEquals(1.65, ods.rawLightDetected, 1e-9)
        assertEquals(0.5, ods.lightDetected, 1e-9)
    }

    @Test
    fun `IMU reports heading relative to the last resetYaw call`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }
        val sim = controlHub.imus.getValue(0)

        val imu = hardwareMap.get(IMU::class.java, "imu")
        sim.headingRad = Math.PI / 2
        assertEquals(90.0, imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES), 1e-6)

        imu.resetYaw()
        assertEquals(0.0, imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES), 1e-6)

        sim.headingRad = Math.PI
        assertEquals(90.0, imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES), 1e-6)
    }

    @Test
    fun `ColorSensor, NormalizedColorSensor, and DistanceSensor all read the same SimI2cDevice readings`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }
        val sim = controlHub.i2cDevices.getValue(1)

        sim.setReading("red", 1.0)
        sim.setReading("green", 0.5)
        sim.setReading("blue", 0.0)
        sim.setReading("alpha", 1.0)
        sim.setReading("distanceMm", 50.0)

        val normalized = hardwareMap.get(NormalizedColorSensor::class.java, "color sensor")
        assertEquals(1.0f, normalized.normalizedColors.red, 1e-6f)
        assertEquals(0.5f, normalized.normalizedColors.green, 1e-6f)

        val raw = hardwareMap.get(ColorSensor::class.java, "color sensor")
        assertEquals(255, raw.red())
        assertEquals(0, raw.blue())

        val distance = hardwareMap.get(DistanceSensor::class.java, "color sensor")
        assertEquals(50.0, distance.getDistance(DistanceUnit.MM), 1e-9)
        assertEquals(5.0, distance.getDistance(DistanceUnit.CM), 1e-9)
    }

    @Test
    fun `CompassSensor reads the same SimI2cDevice's headingDeg reading, normalized to 0-360`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }
        val sim = controlHub.i2cDevices.getValue(1)

        sim.setReading("headingDeg", -30.0)

        val compass = hardwareMap.get(CompassSensor::class.java, "color sensor")
        assertEquals(330.0, compass.direction, 1e-9)
        assertFalse(compass.calibrationFailed())
    }

    @Test
    fun `buildEmulatedHardwareMap from a SimulatedRobot also resolves digital, analog, IMU, and I2C devices`() {
        val configXml = """
            <Robot type="FirstInspires-FTC">
                <LynxModule name="Control Hub" port="173">
                    <Motor name="front left motor" port="0" />
                    <Servo name="claw servo" port="0" />
                    <TouchSensor name="bumper" port="0" />
                    <AnalogInput name="pot" port="0" />
                    <LynxEmbeddedIMU name="imu" bus="0" />
                    <LynxI2cColorRangeSensor name="color sensor" bus="1" />
                </LynxModule>
            </Robot>
        """.trimIndent()
        val simulatedRobot = buildSimulatedRobot(parseRobotConfigXml(configXml))
        val hardwareMap = buildEmulatedHardwareMap(simulatedRobot) { 12.7 }

        simulatedRobot.digitalDevices.getValue("bumper").state = true
        assertTrue(hardwareMap.get(TouchSensor::class.java, "bumper").isPressed)

        simulatedRobot.analogDevices.getValue("pot").voltage = 2.0
        assertEquals(2.0, hardwareMap.get(AnalogInput::class.java, "pot").voltage, 1e-9)

        simulatedRobot.imus.getValue("imu").headingRad = Math.PI / 2
        assertEquals(90.0, hardwareMap.get(IMU::class.java, "imu").robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES), 1e-6)

        simulatedRobot.i2cDevices.getValue("color sensor").setReading("distanceMm", 42.0)
        assertEquals(42.0, hardwareMap.get(DistanceSensor::class.java, "color sensor").getDistance(DistanceUnit.MM), 1e-9)

        assertTrue(hardwareMap.get(CRServo::class.java, "claw servo") != null)
    }
}
