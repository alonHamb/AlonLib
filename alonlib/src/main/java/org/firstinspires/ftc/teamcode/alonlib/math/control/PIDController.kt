package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains

/** A [PIDFController] with the feedforward term [f] fixed at zero. */

open class PIDController(gains: PIDFGains) : PIDFController(gains)
