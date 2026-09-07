package alonlib.math.control

import alonlib.math.PIDFGains

/** A [PIDFController] with the feedforward term [f] fixed at zero. */

open class PIDController(gains: PIDFGains) : PIDFController(gains)
