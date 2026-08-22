package org.firstinspires.ftc.teamcode.alonlib.math

import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDController
import org.junit.Assert.assertEquals
import org.junit.Test

class PIDFGainsTest {

    @Test
    fun `defaults all gains to zero`() {
        val gains = PIDFGains()
        assertEquals(0.0, gains.kP, 1e-9)
        assertEquals(0.0, gains.kI, 1e-9)
        assertEquals(0.0, gains.kD, 1e-9)
        assertEquals(0.0, gains.kFF, 1e-9)
        assertEquals(0.0, gains.kS, 1e-9)
        assertEquals(0.0, gains.KV, 1e-9)
        assertEquals(0.0, gains.Ka, 1e-9)
        assertEquals(0.0, gains.kIZone, 1e-9)
    }

    @Test
    fun `toString includes every gain`() {
        val gains = PIDFGains(kP = 1.0, kI = 2.0, kD = 3.0, kFF = 4.0, kS = 5.0, KV = 6.0, Ka = 7.0)
        val text = gains.toString()

        assertEquals(
            "(kP: 1.0 ,kI: 2.0 ,Kd: 3.0 ,kFF: 4.0 ,kS:5.0 ,kV: 6.0 ,kA:7.0 )",
            text
        )
    }

    @Test
    fun `configPID copies P, I and D onto a PIDController`() {
        val controller = PIDController(0.0, 0.0, 0.0)
        val gains = PIDFGains(kP = 1.5, kI = 0.25, kD = 0.1, kFF = 9.9)

        controller.configPID(gains)

        assertEquals(1.5, controller.p, 1e-9)
        assertEquals(0.25, controller.i, 1e-9)
        assertEquals(0.1, controller.d, 1e-9)
    }
}
