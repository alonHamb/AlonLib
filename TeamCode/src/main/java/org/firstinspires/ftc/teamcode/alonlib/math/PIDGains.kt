package org.firstinspires.ftc.teamcode.alonlib.math

import com.seattlesolvers.solverslib.controller.PIDController

/**
 * Contains the following gains:
 * - [kP] Proportional gain.
 * - [kI] Integral gain.
 * - [kD] Derivative gain.
 * - [kFF] Feed Forward gain calculation function.
 * - [kIZone] If the absolute error is above IZone, the integral accumulator is cleared
 * (making it ineffective). Motor controllers have this feature, but WPILib don't.
 **/
class PIDGains @JvmOverloads constructor(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kFF: Double = 0.0,
    var kS: Double = 0.0,
    var KV: Double = 0.0,
    var Ka: Double = 0.0,
    var kIZone: Double = 0.0,
                                        ) {
    override fun toString(): String {
        return "(kP: $kP ,kI: $kI ,Kd: $kD ,kFF: $kFF ,kS:$kS ,kV: $KV ,kA:$Ka )"
    }
}


fun PIDController.configPID(gains: PIDGains) {
    p = gains.kP
    i = gains.kI
    d = gains.kD
}
