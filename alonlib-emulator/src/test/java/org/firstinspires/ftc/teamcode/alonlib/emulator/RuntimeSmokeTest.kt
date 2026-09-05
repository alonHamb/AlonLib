package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import emulator.hardware.HubId
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Exercises the emulated [HardwareMap] end to end -- including real, unmodified `HaMotor`/`HaServo`
 * from `:alonlib` -- against the real FTC SDK classes at runtime, not just compile-time. This is a
 * regression suite for the trickiest parts of this module ([EmulatedHardwareMapImpl]'s bypass of
 * `Device.isRevControlHub()`'s native-lib crash, the [emulatedLynxModule] Mockito wiring, and
 * [emulatedServo]'s real `ServoImplEx` construction) surviving a future FTC SDK version bump.
 */
class RuntimeSmokeTest {

    private fun hub() = EmulatedHub(
        HubId.CONTROL,
        motors = mapOf(0 to "front left motor"),
        servos = mapOf(0 to "hood servo")
    )

    @Test
    fun `raw DcMotorEx and Servo lookups work`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }

        val motor = hardwareMap.get(DcMotorEx::class.java, "front left motor")
        motor.power = 0.6
        controlHub.motors.getValue(0).update(0.5) // advance sim dynamics by 0.5s
        assertTrue("expected the encoder to have moved", motor.currentPosition != 0)

        val servo = hardwareMap.get(Servo::class.java, "hood servo")
        assertTrue("HaServo force-casts this to ServoImplEx", servo is ServoImplEx)
        servo.position = 0.75
        controlHub.servos.getValue(0).update(10.0) // plenty of time to finish the slew
        assertEquals(0.75, servo.position, 0.01)
    }

    @Test
    fun `LynxModule bulk data and voltage are wired to the same sim motor`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 11.5 }

        val motor = hardwareMap.get(DcMotorEx::class.java, "front left motor")
        motor.power = 1.0
        controlHub.motors.getValue(0).update(1.0)

        val hub = hardwareMap.get(LynxModule::class.java, "Control Hub")
        assertEquals(motor.currentPosition, hub.bulkData.getMotorCurrentPosition(0))
        assertEquals(11.5, hub.getInputVoltage(VoltageUnit.VOLTS), 0.001)
    }

    @Test
    fun `HaMotor and HaServo from alonlib work against the emulated hardware map unmodified`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }

        val haMotor = HaMotor(hardwareMap, "front left motor", Data.Motors.GoBILDA.RPM_435)
        haMotor.percentOutput = 0.4.fraction
        haMotor.update()
        controlHub.motors.getValue(0).update(0.2)
        // Reading through HaMotor exercises hub.bulkData.getMotorCurrentPosition/Velocity.
        haMotor.position
        haMotor.velocity

        // position is relative to the center of the servo's sweep (0deg = centered), so a physical
        // 0.75 fraction is a quarter-range offset from center: (0.75 - 0.5) * range.
        val haServo = HaServo(hardwareMap, "hood servo", Data.Servos.Mode.FullRange, Data.Servos.Type.Speed)
        haServo.position = ( Data.Servos.Type.Speed.range * 0.25)
        controlHub.servos.getValue(0).update(10.0)
        assertEquals(0.75, haServo.servo.position, 0.01)
    }
}
