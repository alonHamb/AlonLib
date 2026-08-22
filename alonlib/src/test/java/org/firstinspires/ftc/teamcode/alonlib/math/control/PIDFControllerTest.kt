package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class PIDFControllerTest {

    private val delta = 1e-6

    @Test
    fun `pure P controller outputs kP times the error`() {
        val controller = PIDFController(2.0, 0.0, 0.0, 0.0, sp = 10.0, pv = 0.0)
        val output = controller.calculate(0.0)
        assertEquals(2.0 * 10.0, output, delta)
    }

    @Test
    fun `feedforward term is kF times the setpoint regardless of error`() {
        val controller = PIDFController(0.0, 0.0, 0.0, 0.5, sp = 4.0, pv = 4.0)
        assertEquals(0.5 * 4.0, controller.calculate(4.0), delta)
    }

    @Test
    fun `atSetPoint respects the configured position tolerance`() {
        val controller = PIDFController(1.0, 0.0, 0.0, 0.0, sp = 10.0, pv = 10.0)
        controller.setTolerance(0.1)
        controller.calculate(10.0)
        assertTrue(controller.atSetPoint())

        controller.calculate(10.5)
        assertTrue(!controller.atSetPoint())
    }

    @Test
    fun `reset zeroes the accumulated integral`() {
        val controller = PIDFController(0.0, 1.0, 0.0, 0.0, sp = 1.0, pv = 0.0)
        controller.calculate(0.0)
        controller.calculate(0.0)

        controller.reset()
        assertEquals(0.0, controller.totalError, delta)
    }

    @Test
    fun `p, i, d, f properties are directly settable and match setPIDF`() {
        val controller = PIDFController(0.0, 0.0, 0.0, 0.0)
        controller.p = 1.0
        controller.i = 2.0
        controller.d = 3.0
        controller.f = 4.0
        assertEquals(doubleArrayOf(1.0, 2.0, 3.0, 4.0).toList(), controller.coefficients.toList())

        controller.setPIDF(5.0, 6.0, 7.0, 8.0)
        assertEquals(doubleArrayOf(5.0, 6.0, 7.0, 8.0).toList(), controller.coefficients.toList())
    }

    @Test
    fun `setPoint setter immediately recomputes positionError`() {
        val controller = PIDFController(0.0, 0.0, 0.0, 0.0, sp = 0.0, pv = 3.0)
        controller.setPoint = 10.0
        assertEquals(7.0, controller.positionError, delta)
    }

    @Test
    fun `minOutput and maxOutput bound the output magnitude while not at the setpoint`() {
        val controller = PIDFController(100.0, 0.0, 0.0, 0.0, sp = 10.0, pv = 0.0)
        controller.setTolerance(0.001)
        controller.maxOutput = 5.0
        val output = controller.calculate(0.0)
        assertEquals(5.0, abs(output), delta)
    }

    @Test
    fun `calculate(pv, sp) sets the setpoint before calculating`() {
        val controller = PIDFController(1.0, 0.0, 0.0, 0.0)
        val output = controller.calculate(0.0, 5.0)
        assertEquals(5.0, output, delta)
        assertEquals(5.0, controller.setPoint, delta)
    }
}
