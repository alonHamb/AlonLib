package org.firstinspires.ftc.teamcode.alonlib.math.control

/** A [PIDFController] with the feedforward term [f] fixed at zero. */
open class PIDController(kp: Double, ki: Double, kd: Double, sp: Double = 0.0, pv: Double = 0.0) :
        PIDFController(kp, ki, kd, 0.0, sp, pv) {

    fun setPID(kp: Double, ki: Double, kd: Double) = setPIDF(kp, ki, kd, 0.0)
}
