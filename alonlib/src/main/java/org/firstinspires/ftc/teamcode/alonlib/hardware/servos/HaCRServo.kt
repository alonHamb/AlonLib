package org.firstinspires.ftc.teamcode.alonlib.hardware.servos

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** A plain continuous-rotation servo, in the same style as [HaServo]/[org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor]. */
class HaCRServo(val crServo: com.qualcomm.robotcore.hardware.CRServo) : com.qualcomm.robotcore.hardware.HardwareDevice by crServo {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo::class.java, id))

    // --- state getters and setters ---

    /** software forward limit only for [percentOutput] */
    var forwardLimit = { false }

    /** software reverse limit only for [percentOutput] */
    var reverseLimit = { false }

    var maxPercentOutput = 1.0
        set(value) { field = value.coerceIn(0.0..1.0) }

    var minPercentOutput = -1.0
        set(value) { field = value.coerceIn(-1.0..maxPercentOutput) }

    var percentOutput: Double = 0.0
        set(value) {
            if (!(forwardLimit() && value > 0) && !(reverseLimit() && value < 0)) {
                field = value
                crServo.power = value.coerceIn(minPercentOutput..maxPercentOutput)
            } else {
                robotPrintError("limit reached")
            }
        }
        get() = crServo.power

    var inverted: Boolean
        get() = crServo.direction == DcMotorSimple.Direction.REVERSE
        set(value) { crServo.direction = if (value) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD }

    fun stop() {
        percentOutput = 0.0
    }
}
