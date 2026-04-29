package org.firstinspires.ftc.teamcode.alonlib.motors

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

class HaMotor(hardwareMap: HardwareMap, id: String, cpr: Double, rpm: Double) :
    MotorEx(hardwareMap, id, cpr, rpm) {
    constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA) : this(
        hardwareMap,
        id,
        type.cpr,
        type.rpm
    )

    // --- hardware declaration ---
    private val voltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    // --- motor configurations ---
    var runningDirection: Direction
        get() {
            return when (this.inverted) {
                true -> Direction.REVERSE
                false -> Direction.FORWARD
            }
        }
        set(value) {
            when (value) {
                Direction.FORWARD -> this.setInverted(false)
                Direction.REVERSE -> this.setInverted(true)
            }
        }

    var runMode: RunMode
        get() = this.runmode
        set(value) {
            this.setRunMode(value)
        }

    var positionTolerance: Rotation2d
        get() = positionController.tolerance[0].degrees
        set(value) = positionController.setTolerance(value.degrees)


    var velocityTolerance: AngularVelocity
        get() = veloController.tolerance[1].rpm
        set(value) = veloController.setTolerance(value.asRpm)

    val positionError get() = positionController.positionError

    val velocityError get() = veloController.velocityError
    var pidfGains: PIDGains
        get() {
            return when (runmode) {
                RunMode.RawPower -> PIDGains(
                    positionController.coefficients[0],
                    positionController.coefficients[1],
                    positionController.coefficients[2],
                    positionController.coefficients[3]
                )

                RunMode.PositionControl -> PIDGains(
                    positionController.coefficients[0],
                    positionController.coefficients[1],
                    positionController.coefficients[2],
                    positionController.coefficients[3]
                )

                RunMode.VelocityControl -> PIDGains(
                    veloController.coefficients[0],
                    veloController.coefficients[1],
                    veloController.coefficients[2],
                    veloController.coefficients[3]
                )
            }
        }
        set(value) {
            positionController.setPIDF(value.kP, value.kI, value.kD, value.kFF)
            veloController.setPIDF(value.kP, value.kI, value.kD, value.kFF)
            feedforward = SimpleMotorFeedforward(value.kS, value.KV, value.Ka)
        }

    // --- state getters and setters ---
    /**
     * percentOutput is clamped between properties minPercentOutput and maxPercentOutput.
     * default is 0.0 and 1.0
     */
    var percentOutput: PercentOutput
        get() = get()
        set(percentOutput) {
            if (!(forwardLimit() && percentOutput > 0) && !(reverseLimit() && percentOutput < 0)) {
                when (runmode) {
                    RunMode.RawPower -> super.set(
                        percentOutput.coerceAtMost(maxPercentOutput)
                            .coerceAtLeast(minPercentOutput) * runningDirection.multiplier
                    )

                    RunMode.PositionControl -> robotPrintError("motor isn't in raw power mode $runmode")
                    RunMode.VelocityControl -> robotPrintError("motor isn't in raw power mode $runmode")
                }
            } else robotPrintError("limit is reached: $position")

        }

    fun setVoltage(voltage: Double) {
        val battery = voltageSensor.voltage.coerceAtLeast(1.0)
        this.percentOutput = ((voltage.coerceIn(1.0, 15.0) / battery))
    }

    val inTolerance: Boolean
        get() = when (runmode) {
            RunMode.PositionControl -> positionController.atSetPoint()
            RunMode.VelocityControl -> veloController.atSetPoint()
            RunMode.RawPower -> false
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
     * when set sets the position setpoint of the motor
     */
    var position
        get() = (currentPosition / cpr).rotations
        set(position) {
            positionSetpoint =
                position.degrees.coerceIn(minimumPosition.degrees, maximumPosition.degrees).degrees
        }

    /**
     * when called gives the current velocity from the motor encoder
     * when set sets the velocity setpoint of the motor
     */
    var velocity: AngularVelocity
        get() = (super.correctedVelocity / cpr).rpm
        set(velocity) {
            when (velocity) {
                0.rpm -> super.set(0.0)
                else -> velocitySetpoint = velocity.coerceIn((maxRPM * -1).rpm, maxRPM.rpm)
            }
        }
    var positionSetpoint: Rotation2d
        get() = positionController.setPoint.degrees
        set(positionSetpoint) {
            when (runmode) {
                RunMode.PositionControl -> positionController.setPoint = positionSetpoint.degrees
                else -> robotPrintError("motor isn't in position control mode but in $runmode")
            }
        }

    var velocitySetpoint: AngularVelocity
        get() = veloController.setPoint.rpm
        set(velocitySetpoint) {
            when (runMode) {
                RunMode.VelocityControl -> veloController.setPoint = velocitySetpoint.asRpm
                else -> robotPrintError("motor isn't in velocity control mode but in $runmode")
            }
        }

    // --- limits ---

    /**
     * the smallest number you can send to the motor with the precent output property
     */
    var minPercentOutput = -1.0
        set(percentOutput) {
            field = percentOutput.coerceIn(-1.0, maxPercentOutput)
        }

    /**
     * the largest number you can send to the motor with the precent output property
     */
    var maxPercentOutput = 1.0
        set(percentOutput) {
            field = percentOutput.coerceIn(minPercentOutput, 1.0)
        }


    /**
     * sets the maximum position setpoint you can set to the motor
     */
    var maximumPosition: Rotation2d = 360.degrees
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
    var minimumPosition = 0.degrees
        set(value) {
            when (value < maximumPosition) {
                true -> value
                false -> robotPrintError("minimum position bigger then maximum position")
            }
            field = value
        }

    // --- periodic functions ---
    fun calculatePidF() {
        veloController.calculate()
        positionController.calculate()
        when (runmode) {
            RunMode.VelocityControl -> feedforward.calculate(velocitySetpoint.asRpm)

            RunMode.PositionControl -> feedforward.calculate(positionSetpoint.degrees)

            RunMode.RawPower -> feedforward.calculate(positionSetpoint.degrees)

        }
    }


}
