package org.firstinspires.ftc.teamcode.alonlib.hardware.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoControllerEx
import org.firstinspires.ftc.teamcode.alonlib.hardware.HardwareDevice
import kotlin.math.abs

/** A [Servo] wrapper with a custom `[min, max]` output range and power-write caching. */
open class ServoEx(private val servo: Servo, private val id: String = "", private val min: Double = 0.0, private val max: Double = 1.0) : HardwareDevice {

    init {
        require(max >= min) { "Minimum angle should be less than maximum angle!" }
        require(min >= 0) { "Minimum angle should be greater than or equal to 0!" }
    }

    constructor(hardwareMap: HardwareMap, id: String, min: Double = 0.0, max: Double = 1.0) :
            this(hardwareMap.get(Servo::class.java, id), id, min, max)

    var cachingTolerance = 0.0001
    private var lastPos = Double.NaN

    /** [output] is in this servo's configured `[min, max]` range. */
    open fun set(output: Double) = setPosition((output - min) / (max - min))

    private fun setPosition(pos: Double) {
        if (lastPos.isNaN() || abs(pos - lastPos) > cachingTolerance) {
            servo.position = pos
            lastPos = pos
        }
    }

    /** The last position [set] wrote, in this servo's `[min, max]` range. */
    open fun get() = lastPos * (max - min) + min

    open fun getRawPosition(): Double = servo.position

    open fun setInverted(inverted: Boolean) = apply { servo.direction = if (inverted) Servo.Direction.REVERSE else Servo.Direction.FORWARD }
    open fun getInverted() = servo.direction == Servo.Direction.REVERSE

    fun setPwm(pwmRange: PwmControl.PwmRange) = apply { controller.setServoPwmRange(servo.portNumber, pwmRange) }

    fun getServo() = servo
    val controller: ServoControllerEx get() = servo.controller as ServoControllerEx
    val portNumber get() = servo.portNumber

    override fun disable() = servo.close()
    override fun getDeviceType() = "Extended Servo; $id from ${servo.portNumber}"
}
