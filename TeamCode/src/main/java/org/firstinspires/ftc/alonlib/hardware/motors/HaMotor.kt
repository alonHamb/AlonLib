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
    private val batteryVoltage = hub.getInputVoltage(VoltageUnit.VOLTS)

    val velocityController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, velocity.asRpm)
    val positonController = PIDFController(0.0, 0.0, 0.0, 0.0, 0.0, position.normalizedDegrees)
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
     * is clamped between properties [minPercentOutput] and [maxPercentOutput].
     * default is -1.0 and 1.0
     */
    var percentOutput: PercentOutput = 0.0
        get() = motor.motor.power
        set(percentOutput) {
            if (!(forwardLimit() && percentOutput > 0) or !(reverseLimit() && percentOutput < 0)) {
                motor.motor.power = percentOutput.coerceIn(minPercentOutput, maxPercentOutput)
                field = percentOutput
            }
            else {
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
     * a very crude current limiter for a motor
     *
     *if the set current is acceded it sets [maxPercentOutput] to the percent that made it go over the limit
     */
    var currentLimit: Double
        set(value) {
            motor.setCurrentAlert(value, CurrentUnit.MILLIAMPS)
        }
        get() {
            return motor.getCurrentAlert(CurrentUnit.MILLIAMPS)
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
            positonController.setCoefficients(PIDFCoefficients(gains.kP, gains.kI, gains.kD, 0.0))
            feedForwardController = SimpleMotorFeedforward(gains.kS, gains.KV, gains.Ka)
            field = gains
        }

    /**
    the current [setPoint] for the motors pid controller

    if the run mode is [RunMode.PositionControl] the unit is degrees if the run mode is [RunMode.VelocityControl] the unit is rpm
     */
    var setPoint: Double = 0.0
        set(setPoint) {
            when (positonController.i > 0 || velocityController.i > 0) {
                true  -> {
                    positonController.reset()
                    velocityController.reset()
                }

                false -> {}
            }
            when (this.runMode) {
                RunMode.PositionControl -> {
                    positonController.setPoint =
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
                    positonController.positionError
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
                RunMode.PositionControl -> positonController.setTolerance(value)
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
                    positonController.atSetPoint()
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
        when (motor.isOverCurrent) {
            true -> maxPercentOutput = percentOutput
            else -> {}
        }
        when (this.runMode) {
            RunMode.VelocityControl -> voltage =
                velocityController.calculate(velocity.asRpm) + feedForwardController.calculate(
                    velocity.asRpm,
                    motor.acceleration / motor.cpr
                                                                                              ) + pidfGains.kFF * error.sign

            RunMode.PositionControl -> voltage =
                positonController.calculate(position.degrees) + feedForwardController.calculate(
                    velocity.asRpm,
                    motor.acceleration / motor.cpr
                                                                                               ) + pidfGains.kFF * error.sign

            RunMode.RawPower        -> {}
        }
        when (motor.isOverCurrent) {
            true -> maxPercentOutput = percentOutput
            else -> {}
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
