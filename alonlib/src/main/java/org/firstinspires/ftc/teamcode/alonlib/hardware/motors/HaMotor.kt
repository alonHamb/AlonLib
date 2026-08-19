package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.Motor.RunMode
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.alonlib.math.PIDFGains
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import kotlin.math.sign

class HaMotor(hardwareMap: HardwareMap, id: String, cpr: Number, rpm: Number) : HardwareDevice {
    constructor(hardwareMap: HardwareMap, id: String, type: Motor.GoBILDA) : this(
        hardwareMap,
        id,
        type.cpr,
        type.rpm
                                                                                 )


    // --- hardware declaration ---
    val hub: LynxModule = hardwareMap.get(LynxModule::class.java, "Control Hub")
    val motor = MotorEx(hardwareMap, id, cpr.toDouble(), rpm.toDouble()).apply {
        runMode = RunMode.RawPower
    }
    private val batteryVoltage: Double
        get() = hub.getInputVoltage(VoltageUnit.VOLTS)

    val velocityController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, velocity.asRpm)
    val positionController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, position.normalizedDegrees)
    var feedForwardController = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    // --- motor configurations ---

    /**
    sets the behavior of the motor when stop() is called or when you set [percentOutput] to zero
     */
    var zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        set(value) {
            motor.setZeroPowerBehavior(value)
            field = value
        }


    /**
     * the direction the motor to rotates
     * @param Motor.Direction.FORWARD clockwise
     * @param Motor.Direction.REVERSE counterclockwise
     */
    var runningDirection: Motor.Direction
        get() {
            return when (motor.inverted) {
                true  -> Motor.Direction.REVERSE
                false -> Motor.Direction.FORWARD
            }
        }
        set(value) {
            when (value) {
                Motor.Direction.FORWARD -> motor.inverted = false
                Motor.Direction.REVERSE -> motor.inverted = true
            }
        }

    /**
     * the way the [update] function is used to control the motor.
     * @param RunMode.RawPower doesn't do anything
     * @param RunMode.PositionControl sends [setPoint] to the pid controller as degrees between [minimumPosition] and [maximumPosition]
     * @param RunMode.VelocityControl sends [setPoint] to the pid controller as rpm between -[rpm] and [rpm]
     */
    var runMode: RunMode = RunMode.RawPower


    // --- state getters and setters ---

    /**
     * sets the percent output of the motor.
     * is clamped between properties [minPercentOutput] and [maxPercentOutput],
     * further scaled down by [currentLimitScalar] when [currentLimit] is active.
     * default is -1.0 and 1.0
     */
    var percentOutput: PercentOutput = 0.0
        get() = motor.motor.power
        set(percentOutput) {
            if (!(forwardLimit() && percentOutput > 0) && !(reverseLimit() && percentOutput < 0)) {
                motor.motor.power = percentOutput.coerceIn(effectiveMinPercentOutput, effectiveMaxPercentOutput)
                field = percentOutput
            } else {
                robotPrintError("limit reached")
            }

        }

    /**
     * the voltage sent to the motor
     */
    var voltage: Double
        get() {
            return batteryVoltage * percentOutput
        }
        set(value) {
            val batteryVoltage = batteryVoltage.coerceAtLeast(1.0)
            percentOutput = (((value / batteryVoltage).coerceIn(minPercentOutput, maxPercentOutput)))
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
        get() = (runningDirection.multiplier * (hub.bulkData.getMotorCurrentPosition(motor.motor.portNumber) / motor.cpr)).rotations
        set(position) {
            setPoint = position.degrees.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
        }

    /**
     * when called gives the current [velocity] from the motor encoder
     *
     * when set sets the velocity [setPoint] of the motor
     */
    var velocity: AngularVelocity
        get() = (runningDirection.multiplier * (hub.bulkData.getMotorVelocity(motor.motor.portNumber) / motor.cpr * 60)).rpm
        set(velocity) {
            when (velocity) {
                0.rpm -> {
                    motor.motor.power = 0.0
                }

                else  -> setPoint = velocity.coerceIn((-motor.maxRPM).rpm, motor.maxRPM.rpm).asRpm
            }
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

    if the run mode is [RunMode.PositionControl] the unit is degrees if the run mode is [RunMode.VelocityControl] the unit is rpm
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
                RunMode.PositionControl -> {
                    positionController.setPoint =
                        setPoint.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
                    field = setPoint
                }


                RunMode.VelocityControl -> {
                    velocityController.setPoint = setPoint.coerceIn(-motor.maxRPM, motor.maxRPM)
                    field = setPoint
                }

                RunMode.RawPower        -> {}
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
                RunMode.PositionControl -> {
                    positionController.positionError
                }

                RunMode.VelocityControl -> {
                    velocityController.positionError
                }

                RunMode.RawPower        -> {
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
                RunMode.PositionControl -> positionController.setTolerance(value)
                RunMode.VelocityControl -> velocityController.setTolerance(value)
                RunMode.RawPower        -> {}
            }
        }

    /**
     * @returns true if the error of the pid controller is within the tolerance
     */
    val inTolerance: Boolean
        get() {
            return when (runMode) {
                RunMode.PositionControl -> {
                    positionController.atSetPoint()
                }

                RunMode.VelocityControl -> {
                    velocityController.atSetPoint()
                }

                RunMode.RawPower        -> {
                    true
                }
            }
        }

    // --- limits ---

    /**
     * the smallest number you can sed to the motor with the [percentOutput] property
     */
    var minPercentOutput = -1.0
        set(percentOutput) {
            field = percentOutput.coerceIn(-1.0, maxPercentOutput)
        }

    /**
     * the largest number you can send to the motor with the [percentOutput] property
     */
    var maxPercentOutput = 1.0
        set(percentOutput) {
            field = percentOutput.coerceIn(minPercentOutput, 1.0)
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
     * does the same as setting [percentOutput] to 0.0
     */
    fun stop() {
        percentOutput = 0.0
        motor.stopMotor()
    }

    /**
     * updates the motors pid controller
     *
     * must be called every loop
     */
    fun update() {
        limitCurrent()
        when (this.runMode) {
            RunMode.VelocityControl -> voltage =
                velocityController.calculate(velocity.asRpm) + feedForwardController.calculate(
                    velocity.asRpm,
                    motor.acceleration / motor.cpr
                                                                                              ) + pidfGains.kFF * error.sign

            RunMode.PositionControl -> voltage =
                positionController.calculate(position.degrees) + feedForwardController.calculate(
                    velocity.asRpm,
                    motor.acceleration / motor.cpr
                                                                                                ) + pidfGains.kFF * error.sign

            RunMode.RawPower        -> {}
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
        motor.stopAndResetEncoder()
    }

    override fun close() {
        motor.disable()
    }


}
