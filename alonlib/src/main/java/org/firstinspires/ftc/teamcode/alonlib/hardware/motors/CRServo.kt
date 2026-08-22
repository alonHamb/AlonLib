package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.alonlib.hardware.HardwareDevice

/**
 * A continuous-rotation servo wrapper -- shaped like [Motor] (same `set`/`get`/`setInverted`
 * surface) but backed by the FTC SDK's own [com.qualcomm.robotcore.hardware.CRServo] rather than
 * a [DcMotor][com.qualcomm.robotcore.hardware.DcMotor], so it doesn't extend [Motor] directly.
 */
open class CRServo(val crServo: com.qualcomm.robotcore.hardware.CRServo, private val id: String = "") : HardwareDevice {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo::class.java, id), id)

    protected var lastPower = 0.0

    open fun set(output: Double) {
        crServo.power = output
        lastPower = output
    }

    open fun get() = lastPower
    open fun getRawPower(): Double = crServo.power

    open fun setInverted(isInverted: Boolean) = apply {
        crServo.direction = if (isInverted) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }

    open fun getInverted() = crServo.direction == DcMotorSimple.Direction.REVERSE

    override fun disable() = crServo.close()

    open fun stop() = set(0.0)
    open fun stopMotor() = stop()

    override fun getDeviceType() = "CR Servo; $id in port ${crServo.portNumber}"
}
