package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * A single digital I/O pin (e.g. a beam-break or a limit switch wired directly, not through a
 * [com.qualcomm.robotcore.hardware.TouchSensor]), with optional debouncing for input use -- set
 * [threshold] above `0.0`, call [update] once per loop, then read [isActive].
 */
class HaDigitalChannel(val digitalChannel: DigitalChannel, threshold: Double = 0.0) : com.qualcomm.robotcore.hardware.HardwareDevice by digitalChannel {

    constructor(hardwareMap: HardwareMap, id: String, threshold: Double = 0.0) :
            this(hardwareMap.get(DigitalChannel::class.java, id), threshold)

    var mode: DigitalChannel.Mode
        get() = digitalChannel.mode
        set(value) { digitalChannel.mode = value }

    var state: Boolean
        get() = digitalChannel.state
        set(value) { digitalChannel.setState(value) }

    /** debounce window in milliseconds -- `0.0` (default) disables debouncing entirely. */
    var threshold = threshold.coerceAtLeast(0.0)
        set(value) {
            field = value.coerceAtLeast(0.0)
        }

    private var lastChangeNanos = System.nanoTime()
    private var debouncedState = false
    private var lastState = false

    /** refreshes [isActive] against [threshold] -- call once per loop before reading it. */
    fun update() {
        val state = digitalChannel.state

        if (threshold == 0.0) {
            debouncedState = state
            lastState = state
            return
        }

        if (state) {
            if (!lastState) lastChangeNanos = System.nanoTime()
            if ((System.nanoTime() - lastChangeNanos) / 1e6 >= threshold) debouncedState = true
        } else {
            debouncedState = false
            lastChangeNanos = System.nanoTime()
        }

        lastState = state
    }

    /** [state], debounced by [threshold] if it's above `0.0` -- see [update]. */
    val isActive: Boolean
        get() = if (threshold == 0.0) digitalChannel.state else debouncedState && digitalChannel.state
}
