package org.firstinspires.ftc.teamcode.alonlib.emulator

import emulator.hardware.HubId
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Regression test for an AlonLib bug where `HaServo`'s default `maxPosition`/`minPosition` (derived
 * from `Data.Servos.Type.range`) threw `IllegalArgumentException("Cannot coerce value to an empty
 * range...")` the moment `position` was first set, for every built-in `Type`. Root cause: `range` was
 * a `Rotation2d`, which normalizes any angle into (-180, 180], so a 300deg/350deg sweep silently
 * became -60deg/-10deg -- putting the `maxPosition` default below the `minPosition` default of 0deg.
 *
 * Fixed by keeping `position`/`minPosition`/`maxPosition` as `Rotation2d`, but relative to the
 * *center* of the servo's sweep instead of one end -- half of even the widest built-in sweep
 * (350deg/2 = 175deg) fits inside Rotation2d's domain, so it can never silently wrap.
 */
class HaServoPositionRegressionTest {

    private fun hub() = EmulatedHub(HubId.CONTROL, servos = mapOf(0 to "hood servo"))

    @Test
    fun `every built-in servo Type has a valid default position range and can be driven end to end`() {
        val controlHub = hub()
        val hardwareMap = buildEmulatedHardwareMap(controlHub) { 12.7 }
        val simServo = controlHub.servos.getValue(0)

        fun settle() = simServo.update(10.0) // plenty of time to finish the slew

        for (type in Data.Servos.Type.entries) {
            val haServo = HaServo(hardwareMap, "hood servo", Data.Servos.Mode.FullRange, type)
            val halfRange = type.range / 2.0

            assertTrue(
                "$type: default maxPosition (${haServo.maxPosition.degrees}) should exceed default minPosition (${haServo.minPosition.degrees})",
                haServo.maxPosition.degrees > haServo.minPosition.degrees
            )

            haServo.position = 0.0.degrees
            settle()
            assertEquals("$type: centered position should be the middle of the physical sweep", 0.5, haServo.servo.position, 1e-9)

            haServo.position = halfRange
            settle()
            assertEquals("$type: maxPosition should reach the top of the physical sweep", 1.0, haServo.servo.position, 1e-9)

            haServo.position = (-halfRange)
            settle()
            assertEquals("$type: minPosition should reach the bottom of the physical sweep", 0.0, haServo.servo.position, 1e-9)
        }
    }
}
