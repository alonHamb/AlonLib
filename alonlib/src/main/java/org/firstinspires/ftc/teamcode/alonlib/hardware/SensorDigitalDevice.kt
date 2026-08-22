package org.firstinspires.ftc.teamcode.alonlib.hardware

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

/** A [DigitalChannel] input, debounced by [threshold] milliseconds -- call [update] once per loop, then read [isActive]. */
open class SensorDigitalDevice(val digitalChannel: DigitalChannel, threshold: Double = 0.0) {

    constructor(hardwareMap: HardwareMap, name: String, threshold: Double = 0.0) :
            this(hardwareMap.get(DigitalChannel::class.java, name), threshold)

    var threshold = threshold.coerceAtLeast(0.0)
        set(value) {
            field = value.coerceAtLeast(0.0)
        }

    private var lastChangeNanos = System.nanoTime()
    private var debouncedState = false
    private var lastState = false

    init {
        digitalChannel.mode = DigitalChannel.Mode.INPUT
    }

    var mode: DigitalChannel.Mode
        get() = digitalChannel.mode
        set(value) { digitalChannel.mode = value }

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

    val isActive: Boolean
        get() = if (threshold == 0.0) digitalChannel.state else debouncedState && digitalChannel.state
}
