package org.firstinspires.ftc.teamcode.alonlib.motors

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.Motor.RunMode
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

class HaMotor(hardwareMap: HardwareMap, id: String, cpr: Number, rpm: Number) : HardwareDevice {
    constructor(hardwareMap: HardwareMap, id: String, type: Motor.GoBILDA) : this(
        hardwareMap,
        id,
        type.cpr,
        type.rpm
    )

    // --- hardware declaration ---
    private val motor = MotorEx(hardwareMap, id, cpr.toDouble(), rpm.toDouble())
    private val voltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()
    private val currentTime = System.nanoTime()

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
                true -> Motor.Direction.REVERSE
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
        set(value) {
            motor.setRunMode(runMode)
            field = value
        }

    /**
     * the gains for the motor's PIDF controller used in any closed loop [runMode]
     */
    var pidfGains: PIDGains = PIDGains()
        set(gains) {
            motor.setCoefficients(PIDFCoefficients(gains.kP, gains.kI, gains.kD, gains.kFF))
            motor.setVeloCoefficients(gains.kP, gains.kI, gains.kD)
            motor.setFeedforwardCoefficients(gains.kS, gains.KV, gains.Ka)
            field = gains
        }

    // --- state getters and setters ---

    /**
     * sets the percent output of the motor.
     * is clamped between properties [minPercentOutput] and [maxPercentOutput].
     * default is -1.0 and 1.0
     */
    var percentOutput: PercentOutput = 0.0
        get() = motor.get()
        set(percentOutput) {
            if (!(forwardLimit() && percentOutput > 0) or !(reverseLimit() && percentOutput < 0)) {
                field = percentOutput
                motor.setRunMode(RunMode.RawPower)
                motor.set(percentOutput.coerceIn(minPercentOutput, maxPercentOutput))
                motor.setRunMode(runMode)
            } else {
                robotPrintError("limit reached")
            }

        }

    /**
     * the voltage sent to the motor
     */
    var voltage: Double
        get() {
            return voltageSensor.voltage * motor.get()
        }
        set(value) {
            val batteryVoltage = voltageSensor.voltage.coerceAtLeast(1.0)
            this.percentOutput = ((value.coerceIn(minPercentOutput * batteryVoltage, maxPercentOutput * batteryVoltage) / batteryVoltage))
        }

    /**
     * the current level of the motor in milliamps
     */
    val current: Double
        get() {
            return motor.getCurrent(CurrentUnit.MILLIAMPS)
        }

    var currentLimit: Double
        set(value) {
            motor.setCurrentAlert(value, CurrentUnit.MILLIAMPS)
        }
        get() {
            return motor.getCurrentAlert(CurrentUnit.MILLIAMPS)
        }

    /**
     * Software forward limit, ONLY for percent-output control.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software reverse limit, ONLY for percent-output control.
     */
    var reverseLimit: () -> Boolean = { false }

    /**
     * when called gives the current position from the motor encoder
     *
     * when set sets the position setpoint of the motor
     */
    var position: Rotation2d
        get() = (motor.currentPosition / motor.cpr).rotations
        set(position) {
            setPoint = position.degrees.coerceIn(minimumPosition.degrees, maximumPosition.degrees)
        }

    /**
     * when called gives the current velocity from the motor encoder
     *
     * when set sets the velocity setpoint of the motor
     */
    var velocity: AngularVelocity
        get() = (motor.correctedVelocity / motor.cpr * 60).rpm
        set(velocity) {
            when (velocity) {
                0.rpm -> {
                    motor.setRunMode(RunMode.RawPower)
                    motor.set(0.0)
                    motor.setRunMode(runMode)
                }

                else -> setPoint = velocity.coerceIn((-motor.maxRPM).rpm, motor.maxRPM.rpm).asRpm
            }
        }

    /**
    the current setpoint for the motors pid controller

    if the run mode is [RunMode.PositionControl] the unit is degrees if the run mode is [RunMode.VelocityControl] the unit is rpm
     */
    var setPoint: Double = 0.0
        set(setPoint) {
            when (this.runMode) {
                RunMode.PositionControl -> motor.setTargetPosition(
                    (setPoint.coerceIn(
                        minimumPosition.degrees,
                        maximumPosition.degrees
                    ) / 360 * motor.cpr).toInt()
                )

                RunMode.VelocityControl -> motor.velocity = (setPoint.coerceIn((-motor.maxRPM), motor.maxRPM)) * motor.cpr / 60
                RunMode.RawPower -> {}
            }
            field = setPoint
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
                true -> value
                false -> robotPrintError("maximum position smaller then minimum position")
            }
            field = value
        }


    /**
     * sets the minimum position setpoint you can set to the motor
     */
    var minimumPosition: Rotation2d = (-180).degrees
        set(value) {
            when (value < maximumPosition) {
                true -> value
                false -> robotPrintError("minimum position bigger then maximum position")
            }
            field = value
        }

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
        when (this.runMode) {
            RunMode.VelocityControl -> motor.set(setPoint / motor.maxRPM)
            RunMode.PositionControl -> motor.set(this.maxPercentOutput)
            RunMode.RawPower -> {}
        }
        when (motor.isOverCurrent) {
            true -> maxPercentOutput = percentOutput
            else -> {}
        }
    }

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
