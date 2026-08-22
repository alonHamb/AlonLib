package org.firstinspires.ftc.teamcode.alonlib.emulator

import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [EmuTelemetry] is a plain, self-contained [Telemetry] implementation with no dependency on real
 * hardware or an Android runtime, so unlike the rest of `alonlib-emulator` it's tested directly
 * rather than through a simulated hardware map.
 */
class EmuTelemetryTest {

    // A named helper, rather than a trailing lambda, sidesteps ambiguity between addData's
    // `Any?` overload (which would capture the lambda object itself as the value) and its
    // Func<T> overload -- Kotlin doesn't reliably prefer the SAM conversion between them.
    private fun <T> func(block: () -> T): Func<T> = object : Func<T> {
        override fun value(): T = block()
    }

    @Test
    fun `addData with a caption renders as caption, separator, value`() {
        val telemetry = EmuTelemetry()
        telemetry.addData("speed", 5)
        assertEquals(listOf("speed : 5"), telemetry.snapshot())
    }

    @Test
    fun `addData with a format string and args renders the formatted value`() {
        val telemetry = EmuTelemetry()
        telemetry.addData("pos", "x=%d y=%d", 1, 2)
        assertEquals(listOf("pos : x=1 y=2"), telemetry.snapshot())
    }

    @Test
    fun `addData with a value producer re-evaluates it on every snapshot`() {
        val telemetry = EmuTelemetry()
        var counter = 0
        telemetry.addData("counter", func { counter })

        counter = 1
        assertEquals(listOf("counter : 1"), telemetry.snapshot())
        counter = 2
        assertEquals(listOf("counter : 2"), telemetry.snapshot())
    }

    @Test
    fun `addLine groups multiple items behind an item separator`() {
        val telemetry = EmuTelemetry()
        val line = telemetry.addLine("status: ")
        line.addData("a", 1)
        line.addData("b", 2)

        assertEquals(listOf("status: a : 1 | b : 2"), telemetry.snapshot())
    }

    @Test
    fun `custom item and caption-value separators are honored`() {
        val telemetry = EmuTelemetry()
        telemetry.itemSeparator = ", "
        telemetry.captionValueSeparator = "="
        val line = telemetry.addLine()
        line.addData("a", 1)
        line.addData("b", 2)

        assertEquals(listOf("a=1, b=2"), telemetry.snapshot())
    }

    @Test
    fun `clear removes plain-value items but keeps Func-backed ones`() {
        val telemetry = EmuTelemetry()
        telemetry.addData("plain", 1)
        telemetry.addData("live", func { 2 })

        telemetry.clear()

        assertEquals(listOf("live : 2"), telemetry.snapshot())
    }

    @Test
    fun `setRetained overrides an item's default clear behavior`() {
        val telemetry = EmuTelemetry()
        val plainItem = telemetry.addData("plain", 1)
        plainItem.setRetained(true)

        telemetry.clear()

        assertEquals(listOf("plain : 1"), telemetry.snapshot())
    }

    @Test
    fun `clearAll removes every item regardless of retained state`() {
        val telemetry = EmuTelemetry()
        val liveItem = telemetry.addData("live", func { 1 })
        liveItem.setRetained(true)

        telemetry.clearAll()

        assertTrue(telemetry.snapshot().isEmpty())
    }

    @Test
    fun `removeItem stops an item from appearing in future snapshots`() {
        val telemetry = EmuTelemetry()
        val item = telemetry.addData("a", 1)
        telemetry.addData("b", 2)

        val removed = telemetry.removeItem(item)

        assertTrue(removed)
        assertEquals(listOf("b : 2"), telemetry.snapshot())
    }

    @Test
    fun `removeLine stops a whole line from appearing in future snapshots`() {
        val telemetry = EmuTelemetry()
        val line = telemetry.addLine()
        line.addData("a", 1)
        telemetry.addData("b", 2)

        val removed = telemetry.removeLine(line)

        assertTrue(removed)
        assertEquals(listOf("b : 2"), telemetry.snapshot())
    }

    @Test
    fun `update runs pending actions and auto-clears by default`() {
        val telemetry = EmuTelemetry()
        var actionRan = false
        telemetry.addAction { actionRan = true }
        telemetry.addData("plain", 1)

        val result = telemetry.update()

        assertTrue(result)
        assertTrue(actionRan)
        assertTrue(telemetry.snapshot().isEmpty())
    }

    @Test
    fun `update does not clear when autoClear is disabled`() {
        val telemetry = EmuTelemetry()
        telemetry.isAutoClear = false
        telemetry.addData("plain", 1)

        telemetry.update()

        assertEquals(listOf("plain : 1"), telemetry.snapshot())
    }

    @Test
    fun `removeAction stops it from running on update`() {
        val telemetry = EmuTelemetry()
        var actionRan = false
        val token = telemetry.addAction { actionRan = true }

        val removed = telemetry.removeAction(token)
        telemetry.update()

        assertTrue(removed)
        assertFalse(actionRan)
    }

    @Test
    fun `msTransmissionInterval getter and setter round-trip`() {
        val telemetry = EmuTelemetry()
        telemetry.msTransmissionInterval = 42
        assertEquals(42, telemetry.msTransmissionInterval)
    }

    @Test
    fun `log entries are pushed in insertion order and capped at the configured capacity`() {
        val telemetry = EmuTelemetry()
        telemetry.log().capacity = 2

        telemetry.log().add("first")
        telemetry.log().add("second")
        telemetry.log().add("third")

        assertEquals(listOf("second", "third"), telemetry.snapshot())
    }

    @Test
    fun `log add with a format string formats before storing`() {
        val telemetry = EmuTelemetry()
        telemetry.log().add("value=%d", 7)

        assertEquals(listOf("value=7"), telemetry.snapshot())
    }

    @Test
    fun `log respects NEWEST_FIRST display order`() {
        val telemetry = EmuTelemetry()
        telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.NEWEST_FIRST

        telemetry.log().add("first")
        telemetry.log().add("second")

        assertEquals(listOf("second", "first"), telemetry.snapshot())
    }

    @Test
    fun `log clear empties the log without touching other telemetry items`() {
        val telemetry = EmuTelemetry()
        telemetry.addData("plain", 1)
        telemetry.log().add("entry")

        telemetry.log().clear()

        assertEquals(listOf("plain : 1"), telemetry.snapshot())
    }
}
