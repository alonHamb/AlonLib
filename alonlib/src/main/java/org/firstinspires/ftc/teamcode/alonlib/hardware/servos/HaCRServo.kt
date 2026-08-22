package org.firstinspires.ftc.teamcode.alonlib.hardware.servos

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoControllerEx
import org.firstinspires.ftc.teamcode.alonlib.math.angleModulus
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.abs

/**
 * A continuous-rotation servo, in the same style as [HaServo]/[org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor] --
 * with optional absolute-position closed-loop control (e.g. for an Axon servo with a feedback
 * wire), power-write caching, and optional [followers] that mirror this servo's [percentOutput].
 */
class HaCRServo(
    val crServo: com.qualcomm.robotcore.hardware.CRServo,
    var absolutePositionRadians: (() -> Double)? = null,
    private vararg val followers: HaCRServo,
) : com.qualcomm.robotcore.hardware.HardwareDevice by crServo {

    constructor(hardwareMap: HardwareMap, id: String, absolutePositionRadians: (() -> Double)? = null, vararg followers: HaCRServo) :
            this(hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo::class.java, id), absolutePositionRadians, *followers)

    enum class RunMode {
        /** [percentOutput] takes a target angle (radians); [pidf] steers the servo the shortest way to it. */
        OPTIMIZED_POSITIONAL_CONTROL,

        /** [percentOutput] takes a raw `[-1, 1]` power. */
        RAW_POWER,
    }

    // --- state getters and setters ---

    /** which meaning [percentOutput] takes -- see [RunMode] */
    var runMode: RunMode = RunMode.RAW_POWER

    /** the closed-loop controller used in [RunMode.OPTIMIZED_POSITIONAL_CONTROL] */
    var pidf: PIDFController? = null

    fun getAbsolutePositionRadians(): Double =
        checkNotNull(absolutePositionRadians) { "This HaCRServo has no absolute position source configured" }()

    /** the minimum power delta (or exactly zero) before [percentOutput]'s setter actually writes to the servo */
    var cachingTolerance = 0.0001
    private var lastWrittenPower = 0.0

    /** software forward limit only for [percentOutput] */
    var forwardLimit = { false }

    /** software reverse limit only for [percentOutput] */
    var reverseLimit = { false }

    var maxPercentOutput = 1.0
        set(value) { field = value.coerceIn(0.0..1.0) }

    var minPercentOutput = -1.0
        set(value) { field = value.coerceIn(-1.0..maxPercentOutput) }

    /**
     * in [RunMode.RAW_POWER], a direct `[-1, 1]` power; in [RunMode.OPTIMIZED_POSITIONAL_CONTROL], a
     * target angle in radians that [pidf] steers [getAbsolutePositionRadians] towards.
     *
     * mirrored to every one of [followers] once applied here.
     */
    var percentOutput: Double = 0.0
        set(value) {
            if (!(forwardLimit() && value > 0) && !(reverseLimit() && value < 0)) {
                field = value
                val power = when (runMode) {
                    RunMode.OPTIMIZED_POSITIONAL_CONTROL -> {
                        val controller = checkNotNull(pidf) { "Must have a pidf controller configured for OPTIMIZED_POSITIONAL_CONTROL" }
                        controller.calculate(0.0, angleModulus(value - getAbsolutePositionRadians()))
                    }

                    RunMode.RAW_POWER                    -> value
                }
                val clamped = power.coerceIn(minPercentOutput, maxPercentOutput)
                writePower(clamped)
                followers.forEach { it.percentOutput = clamped }
            } else {
                robotPrintError("limit reached")
            }
        }
        get() = crServo.power

    private fun writePower(power: Double) {
        if (abs(power - lastWrittenPower) > cachingTolerance || (power == 0.0 && lastWrittenPower != 0.0)) {
            crServo.power = power
            lastWrittenPower = power
        }
    }

    fun setPwm(pwmRange: PwmControl.PwmRange) = apply { controller.setServoPwmRange(crServo.portNumber, pwmRange) }

    val controller: ServoControllerEx get() = crServo.controller as ServoControllerEx

    var inverted: Boolean
        get() = crServo.direction == DcMotorSimple.Direction.REVERSE
        set(value) { crServo.direction = if (value) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD }

    /**
     * stops the servo, and every one of [followers] too
     */
    fun stop() {
        percentOutput = 0.0
        followers.forEach { it.stop() }
    }
}
