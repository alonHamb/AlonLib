package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeHarness
import com.qualcomm.robotcore.hardware.Gamepad
import emulator.hardware.HubId
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Exercises [OpModeHarness]'s same-package access to `OpMode`/`OpModeInternal`'s package-private
 * lifecycle hooks at runtime, for both plain [OpMode] and [LinearOpMode] subclasses -- a regression
 * suite protecting [OpModeHarness.init]'s hand-rolled reimplementation of `internalInit()` (needed
 * because the real one spawns its thread via `ThreadPool`, which touches an Android-only collection
 * class with no working desktop implementation) against a future FTC SDK version bump.
 */
class OpModeHarnessSmokeTest {

    private class CountingOpMode : OpMode() {
        var initCalls = 0
        var loopCalls = 0
        var stopCalls = 0
        override fun init() {
            initCalls++
        }

        override fun loop() {
            loopCalls++
        }

        override fun stop() {
            stopCalls++
        }
    }

    private class CrashingLinearOpMode : LinearOpMode() {
        override fun runOpMode() {
            throw IllegalStateException("boom")
        }
    }

    private fun hardwareMap() =
        buildEmulatedHardwareMap(EmulatedHub(HubId.CONTROL)) { 12.7 }

    @Test
    fun `iterative OpMode goes through init, loop, and stop`() {
        val opMode = CountingOpMode()
        val harness = OpModeHarness(opMode)
        harness.init(hardwareMap())

        // init() runs on a background thread; give it a moment to land.
        val deadline = System.currentTimeMillis() + 2000
        while (opMode.initCalls == 0 && System.currentTimeMillis() < deadline) Thread.sleep(5)
        assertEquals(1, opMode.initCalls)

        harness.start()
        harness.tick(Gamepad(), Gamepad())
        harness.tick(Gamepad(), Gamepad())
        Thread.sleep(50) // let the background OpMode thread actually spin its loop() a few times
        assertTrue("expected loop() to have run at least once", opMode.loopCalls > 0)

        harness.stop()
        assertEquals(1, opMode.stopCalls)
        assertNull(harness.crash)
    }

    @Test
    fun `an exception thrown from user code is surfaced as a crash, not silently swallowed`() {
        val harness = OpModeHarness(CrashingLinearOpMode())
        harness.init(hardwareMap())
        harness.start()

        val deadline = System.currentTimeMillis() + 2000
        while (harness.crash == null && System.currentTimeMillis() < deadline) Thread.sleep(5)

        assertTrue(harness.crash is IllegalStateException)
        assertEquals("boom", harness.crash?.message)
    }
}
