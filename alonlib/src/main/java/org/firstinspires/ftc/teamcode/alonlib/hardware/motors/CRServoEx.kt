package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoControllerEx
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.math.angleModulus
import kotlin.math.abs

/**
 * A [CRServo] with optional absolute-position closed-loop control (e.g. for an Axon servo with a
 * feedback wire) and power-write caching.
 *
 * Takes a plain [absolutePositionRadians] supplier rather than SolversLib's concrete
 * `AbsoluteAnalogEncoder` type -- wire up whatever analog-encoder wrapper you're using (this
 * port's generic-interface `Ha*` sensor wrappers, once built, or your own) to a lambda instead.
 */
open class CRServoEx(
    crServo: com.qualcomm.robotcore.hardware.CRServo,
    id: String = "",
    private var absolutePositionRadians: (() -> Double)? = null,
    private var runMode: RunMode = RunMode.RAW_POWER,
) : CRServo(crServo, id) {

    constructor(hardwareMap: HardwareMap, id: String, absolutePositionRadians: (() -> Double)? = null, runMode: RunMode = RunMode.RAW_POWER) :
            this(hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo::class.java, id), id, absolutePositionRadians, runMode)

    enum class RunMode {
        /** [set] takes a target angle (radians); the servo takes the shortest path to reach it. */
        OPTIMIZED_POSITIONAL_CONTROL,

        /** [set] takes a raw `[-1, 1]` power, same as a plain [CRServo]. */
        RAW_POWER,
    }

    private var pidf: PIDFController? = null
    var cachingTolerance = 0.0001

    fun setRunMode(mode: RunMode) = apply { runMode = mode }
    fun setPidf(controller: PIDFController) = apply { pidf = controller }
    fun setAbsolutePositionSource(source: () -> Double) = apply { absolutePositionRadians = source }

    fun getAbsolutePositionRadians(): Double =
        checkNotNull(absolutePositionRadians) { "This CRServoEx has no absolute position source configured" }()

    override fun set(output: Double) {
        if (runMode == RunMode.OPTIMIZED_POSITIONAL_CONTROL) {
            val positionSource = checkNotNull(absolutePositionRadians) { "Must have an absolute position source and PIDF controller for CR Servo positional control" }
            val controller = checkNotNull(pidf) { "Must have an absolute position source and PIDF controller for CR Servo positional control" }
            val error = angleModulus(output - positionSource())
            writePower(controller.calculate(0.0, error))
        } else {
            writePower(output)
        }
    }

    fun setPwm(pwmRange: PwmControl.PwmRange) = apply { controller.setServoPwmRange(crServo.portNumber, pwmRange) }

    val controller: ServoControllerEx get() = crServo.controller as ServoControllerEx

    private fun writePower(power: Double) {
        if (abs(power - lastPower) > cachingTolerance || (power == 0.0 && lastPower != 0.0)) {
            crServo.power = power
            lastPower = power
        }
    }

    override fun getDeviceType() = "Extended ${super.getDeviceType()}"
}
