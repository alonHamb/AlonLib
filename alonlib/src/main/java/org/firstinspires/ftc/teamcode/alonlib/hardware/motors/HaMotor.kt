package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.Direction
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.GoBILDA
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.RunMode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.ZeroPowerBehavior.FLOAT
import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.Percentage
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import kotlin.math.abs
import kotlin.math.sign

/**
 * AlonLib's motor hardware wrapper -- owns an SDK [DcMotorEx] directly (encoder position/velocity
 * read via [hub]'s bulk data, not the SDK's own per-call reads), with its own software PIDF loop
 * for [RunMode.POSITION_CONTROL]/[RunMode.VELOCITY_CONTROL] and software current limiting.
 *
 * Optional [followers] mirror this motor's [percentOutput] every time it's set (directly, or via
 * [voltage]/[update]) -- construct each one the way you want it to run (direction, zero-power
 * behavior, ...) and pass it in here; they never run their own PID.
 */
class HaMotor(hardwareMap: HardwareMap, id: String, cpr: Number, rpm: Number, private vararg val followers: HaMotor) : HardwareDevice {
    constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA, vararg followers: HaMotor) : this(
        hardwareMap,
        id,
        type.cpr,
        type.rpm,
        *followers
    )


    // --- hardware declaration ---
    private val ticksPerRev: Double = cpr.toDouble()

    /** This motor's configured free-run RPM (the [GoBILDA] preset's, or whatever was passed to the raw `cpr`/`rpm` constructor). */
    val maxRpm: Double = rpm.toDouble()

    val hub: LynxModule = hardwareMap.get(LynxModule::class.java, "Control Hub")
    val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, id).apply {
        val configType = motorType.clone()
        configType.maxRPM = maxRpm
        configType.ticksPerRev = ticksPerRev
        motorType = configType
    }
    private val batteryVoltage: Double
        get() = hub.getInputVoltage(VoltageUnit.VOLTS)

    val velocityController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, velocity.asRpm)
    val positionController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, position.normalizedDegrees)
    var feedForwardController = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private var lastWrittenPower = 0.0
    private var lastVelocityRotationsPerSecond = 0.0
    private var lastAccelerationTimestamp = System.nanoTime() / 1e9

    // --- motor configurations ---

    /**
     * the minimum power delta (or exactly zero) before [percentOutput]'s setter actually writes to the motor
     */
    var cachingTolerance = 0.0001

    /**
    sets the behavior of the motor when stop() is called or when you set [percentOutput] to zero
     */
    var zeroPowerBehavior = FLOAT
        set(value) {
            motor.zeroPowerBehavior = value.sdkBehavior
            followers.forEach { it.zeroPowerBehavior = value }
            field = value
        }


    /**
     * the direction the motor to rotates
     * @param Direction.FORWARD clockwise
     * @param Direction.REVERSE counterclockwise
     */
    var runningDirection: Direction
        get() {
            return when (motor.direction) {
                DcMotorSimple.Direction.REVERSE -> Direction.REVERSE
                else                            -> Direction.FORWARD
            }
        }
        set(value) {
            motor.direction = when (value) {
                Direction.FORWARD -> DcMotorSimple.Direction.FORWARD
                Direction.REVERSE -> DcMotorSimple.Direction.REVERSE
            }
        }

    /**
     * the way the [update] function is used to control the motor.
     * @param RunMode.RAW_POWER doesn't do anything
     * @param RunMode.POSITION_CONTROL sends [setPoint] to the pid controller as degrees between [minimumPosition] and [maximumPosition]
     * @param RunMode.VELOCITY_CONTROL sends [setPoint] to the pid controller as rpm between -[rpm] and [rpm]
     */
    var runMode: RunMode = RunMode.RAW_POWER


    // --- state getters and setters ---

    /**
     * sets the percent output of the motor.
     * is clamped between properties [minPercentOutput] and [maxPercentOutput],
     * further scaled down by [currentLimitScalar] when [currentLimit] is active.
     * default is -1.0 and 1.0
     *
     * mirrored to every one of [followers] once applied here.
     */
    var percentOutput: Percentage = 0.fraction
        get() = motor.power.fraction
        set(percentOutput) {
            if (!(forwardLimit() && percentOutput.asFraction > 0.0) && !(reverseLimit() && percentOutput.asFraction < 0.0)) {
                val clamped = percentOutput.coerceIn(effectiveMinPercentOutput, effectiveMaxPercentOutput)
                writePower(clamped.asFraction)
                field = clamped
                followers.forEach { it.percentOutput = clamped }
            } else {
                robotPrintError("limit reached")
            }

        }

    private fun writePower(power: Double) {
        if (abs(power - lastWrittenPower) > cachingTolerance || (power == 0.0 && lastWrittenPower != 0.0)) {
            motor.power = power
            lastWrittenPower = power
        }
    }

    /**
     * the voltage sent to the motor
     */
    var voltage: Double
        get() {
            return batteryVoltage * percentOutput.asFraction
        }
        set(value) {
            val batteryVoltage = batteryVoltage.coerceAtLeast(1.0)
            percentOutput = (value / batteryVoltage).fraction.coerceIn(minPercentOutput, maxPercentOutput)
        }

    /**
     * the current level of the motor in milliamps
     */
    val current: Double
        get() {
            return motor.getCurrent(CurrentUnit.MILLIAMPS)
        }

    /**
     * the current limit in milliamps, enforced purely by scaling down [percentOutput].
     *
     * every [update] call, if [current] is over this value [currentLimitScalar] is stepped down by
     * [currentLimitStep], shrinking the allowed [percentOutput] range; once [current] is back under
     * the limit, [currentLimitScalar] is stepped back up until it reaches 1.0 again.
     *
     * set to a value <= 0.0 to disable current limiting entirely.
     */
    var currentLimit: Double = 0.0

    /**
     * how much [currentLimitScalar] moves per [update] call while backing off or recovering.
     *
     * smaller values react more smoothly (less oscillation) but take longer to back off from an
     * over-current condition; larger values react faster but can hunt/oscillate around the limit.
     */
    var currentLimitStep: Double = 0.05
        set(value) {
            field = value.coerceIn(0.0, 1.0)
        }

    /**
     * the fraction (0.0 to 1.0) that [minPercentOutput]/[maxPercentOutput] are currently scaled by
     * because of [currentLimit]. 1.0 means no scaling is being applied.
     */
    var currentLimitScalar: Double = 1.0
        private set

    private val effectiveMinPercentOutput get() = minPercentOutput * currentLimitScalar
    private val effectiveMaxPercentOutput get() = maxPercentOutput * currentLimitScalar

    /**
     * steps [currentLimitScalar] towards backing off or recovering based on [current] vs [currentLimit],
     * then re-clamps the currently commanded [percentOutput] into the resulting range.
     *
     * called every loop by [update].
     */
    private fun limitCurrent() {
        currentLimitScalar = if (currentLimit <= 0.0) {
            1.0
        } else if (current > currentLimit) {
            (currentLimitScalar - currentLimitStep).coerceIn(0.0, 1.0)
        } else {
            (currentLimitScalar + currentLimitStep).coerceIn(0.0, 1.0)
        }
        percentOutput = percentOutput
    }

    /**
     * Software forward limit, ONLY for [percentOutput] control.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software reverse limit, ONLY for [percentOutput] control.
     */
    var reverseLimit: () -> Boolean = { false }


    /**
     * when called gives the current [position] from the motor encoder
     *
     * when set sets the position [setPoint] of the motor
     */
    var position: Rotation2d
        get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.portNumber) / ticksPerRev)).rotations
        set(position) {
            setPoint = position.degrees.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
        }

    /**
     * when called gives the current [velocity] from the motor encoder
     *
     * when set sets the velocity [setPoint] of the motor
     */
    var velocity: AngularVelocity
        get() = (runningDirection.multiplier * (hub.bulkData.getMotorVelocity(motor.portNumber) / ticksPerRev * 60)).rpm
        set(velocity) {
            when (velocity) {
                0.rpm -> {
                    motor.power = 0.0
                }

                else  -> setPoint = velocity.coerceIn((-maxRpm).rpm, maxRpm.rpm).asRpm
            }
        }

    /**
     * rotations/second^2, estimated from consecutive [velocity] reads across [update] calls -- fed
     * into [feedForwardController]'s acceleration term.
     */
    private fun estimateAcceleration(): Double {
        val now = System.nanoTime() / 1e9
        val dt = now - lastAccelerationTimestamp
        val velocityRotationsPerSecond = velocity.asRpm / 60.0
        val acceleration = if (dt > 1e-4) (velocityRotationsPerSecond - lastVelocityRotationsPerSecond) / dt else 0.0
        lastVelocityRotationsPerSecond = velocityRotationsPerSecond
        lastAccelerationTimestamp = now
        return acceleration
    }

    // --- pid properties ---

    /**
     * the [pidfGains] for the motor's PIDF controller used in any closed loop [runMode]
     */
    var pidfGains: PIDFGains = PIDFGains()
        set(gains) {
            velocityController.setCoefficients(PIDFCoefficients(gains.kP, gains.kI, gains.kD, 0.0))
            positionController.setCoefficients(PIDFCoefficients(gains.kP, gains.kI, gains.kD, 0.0))
            feedForwardController = SimpleMotorFeedforward(gains.kS, gains.KV, gains.Ka)
            field = gains
        }

    /**
    the current [setPoint] for the motors pid controller

    if the run mode is [RunMode.POSITION_CONTROL] the unit is degrees if the run mode is [RunMode.VELOCITY_CONTROL] the unit is rpm
     */
    var setPoint: Double = 0.0
        set(setPoint) {
            when (positionController.i > 0 || velocityController.i > 0) {
                true  -> {
                    positionController.reset()
                    velocityController.reset()
                }

                false -> {}
            }
            when (this.runMode) {
                RunMode.POSITION_CONTROL -> {
                    positionController.setPoint =
                        setPoint.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
                    field = setPoint
                }


                RunMode.VELOCITY_CONTROL -> {
                    velocityController.setPoint = setPoint.coerceIn(-maxRpm, maxRpm)
                    field = setPoint
                }

                RunMode.RAW_POWER        -> {}
            }
        }

    /**
     *  the current [error] of the pid controller
     *
     *  uses the same units as the controller
     */
    val error: Double
        get() {
            return when (runMode) {
                RunMode.POSITION_CONTROL -> {
                    positionController.positionError
                }

                RunMode.VELOCITY_CONTROL -> {
                    velocityController.positionError
                }

                RunMode.RAW_POWER        -> {
                    0.0
                }
            }

        }

    /**
     * the tolerance used for the [inTolerance] properties
     *
     * uses the units of the pid controller
     */
    var tolerance: Double = 0.0
        set(value) {
            when (runMode) {
                RunMode.POSITION_CONTROL -> positionController.setTolerance(value)
                RunMode.VELOCITY_CONTROL -> velocityController.setTolerance(value)
                RunMode.RAW_POWER        -> {}
            }
        }

    /**
     * @returns true if the error of the pid controller is within the tolerance
     */
    val inTolerance: Boolean
        get() {
            return when (runMode) {
                RunMode.POSITION_CONTROL -> {
                    positionController.atSetPoint()
                }

                RunMode.VELOCITY_CONTROL -> {
                    velocityController.atSetPoint()
                }

                RunMode.RAW_POWER        -> {
                    true
                }
            }
        }

    // --- limits ---

    /**
     * the smallest number you can sed to the motor with the [percentOutput] property
     */
    var minPercentOutput = (-1).fraction
        set(percentOutput) {
            field = percentOutput.coerceIn((-1).fraction, maxPercentOutput)
        }

    /**
     * the largest number you can send to the motor with the [percentOutput] property
     */
    var maxPercentOutput = 1.fraction
        set(percentOutput) {
            field = percentOutput.coerceIn(minPercentOutput, 1.fraction)
        }


    /**
     * sets the maximum position setpoint you can set to the motor
     */
    var maximumPosition: Rotation2d = 180.degrees
        set(value) {
            when (value > minimumPosition) {
                true  -> field = value
                false -> robotPrintError("maximum position smaller then minimum position")
            }
        }


    /**
     * sets the minimum position setpoint you can set to the motor
     */
    var minimumPosition: Rotation2d = (-180).degrees
        set(value) {
            when (value < maximumPosition) {
                true  -> field = value
                false -> robotPrintError("minimum position bigger then maximum position")
            }
        }

    // --- operations functions ---

    /**
     * stops the motor
     *
     * does the same as setting [percentOutput] to 0.0, and stops every one of [followers] too.
     */
    fun stop() {
        percentOutput = 0.fraction
        motor.power = 0.0
        lastWrittenPower = 0.0
        followers.forEach { it.stop() }
    }

    /**
     * updates the motors pid controller
     *
     * must be called every loop
     */
    fun update() {
        limitCurrent()
        when (this.runMode) {
            RunMode.VELOCITY_CONTROL -> voltage =
                velocityController.calculate(velocity.asRpm) + feedForwardController.calculate(
                    velocity.asRpm,
                    estimateAcceleration()
                ) + pidfGains.kFF * error.sign

            RunMode.POSITION_CONTROL -> voltage =
                positionController.calculate(position.degrees) + feedForwardController.calculate(
                    velocity.asRpm,
                    estimateAcceleration()
                ) + pidfGains.kFF * error.sign

            RunMode.RAW_POWER        -> {}
        }
    }

    // --- hardware device shit ---

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String {
        return "HaMotor"
    }

    override fun getConnectionInfo(): String {
        return ""
    }

    override fun getVersion(): Int {
        return 1
    }

    override fun resetDeviceConfigurationForOpMode() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        followers.forEach { it.resetDeviceConfigurationForOpMode() }
    }

    override fun close() {
        motor.close()
        followers.forEach { it.close() }
    }


}
