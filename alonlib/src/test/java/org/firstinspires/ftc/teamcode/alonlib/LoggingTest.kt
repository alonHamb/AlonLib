package org.firstinspires.ftc.teamcode.alonlib

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.mockito.Mockito.mock
import org.mockito.Mockito.verify
import java.io.ByteArrayOutputStream
import java.io.PrintStream

class LoggingTest {

    private fun captureStdOut(block: () -> Unit): String {
        val original = System.out
        val buffer = ByteArrayOutputStream()
        System.setOut(PrintStream(buffer))
        try {
            block()
        } finally {
            System.setOut(original)
        }
        return buffer.toString()
    }

    @Test
    fun `robotPrint wraps the message with markers`() {
        val output = captureStdOut { robotPrint("hello") }
        assertTrue(output.contains("ROBOT PRINT( hello )END"))
    }

    @Test
    fun `robotPrintError wraps the message with markers`() {
        val output = captureStdOut { robotPrintError("oops") }
        assertTrue(output.contains("ROBOT ERROR: oops END"))
    }

    @Test
    fun `transmissionIntervalMs is tighter for Testing than for Competition`() {
        assertEquals(25, TelemetryLevel.Testing.transmissionIntervalMs)
        assertEquals(250, TelemetryLevel.Competition.transmissionIntervalMs)
    }

    @Test
    fun `throttleTo sets msTransmissionInterval to the level's interval`() {
        val telemetry = mock(Telemetry::class.java)

        telemetry.throttleTo(TelemetryLevel.Competition)

        verify(telemetry).msTransmissionInterval = 250
    }
}
