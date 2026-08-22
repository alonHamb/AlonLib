package org.firstinspires.ftc.teamcode.alonlib.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/** An absolute analog encoder (e.g. an Axon servo's feedback wire) read through an [AnalogInput], normalized to `[0, max)` of [angleUnit]. */
open class AbsoluteAnalogEncoder(
    private val encoder: AnalogInput,
    private val id: String = "",
    private val range: Double = DEFAULT_RANGE,
    val angleUnit: AngleUnit = AngleUnit.RADIANS,
) : HardwareDevice {

    constructor(hardwareMap: HardwareMap, id: String, range: Double = DEFAULT_RANGE, angleUnit: AngleUnit = AngleUnit.RADIANS) :
            this(hardwareMap.get(AnalogInput::class.java, id), id, range, angleUnit)

    private var offset = 0.0
    var reversed = false
        private set

    private var lastTimestamp = System.nanoTime() / 1e9
    private var lastPosition = 0.0
    private var velocity = 0.0

    private val fullRange get() = if (angleUnit == AngleUnit.DEGREES) 360.0 else 2 * Math.PI

    fun zero(offset: Double) = apply { this.offset = offset }
    fun setReversed(reversed: Boolean) = apply { this.reversed = reversed }

    /** This encoder's normalized position in `[0, max)` of [angleUnit], accounting for [zero]/[setReversed]. */
    fun getCurrentPosition(): Double {
        val raw = (if (!reversed) 1 - voltage / range else voltage / range) * fullRange - offset
        val normalized = ((raw % fullRange) + fullRange) % fullRange

        val now = System.nanoTime() / 1e9
        val dt = now - lastTimestamp
        if (dt > 0) velocity = (normalized - lastPosition) / dt
        lastTimestamp = now
        lastPosition = normalized

        return normalized
    }

    fun getVelocity() = velocity

    val voltage get() = encoder.voltage

    fun getEncoder() = encoder

    override fun disable() {
        // No-op, matching upstream -- AnalogInput.close() isn't called here.
    }

    override fun getDeviceType() = "Absolute Analog Encoder; $id"

    companion object {
        const val DEFAULT_RANGE = 3.3
    }
}
