package org.firstinspires.ftc.teamcode.alonlib.servos

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm


class HaCrServo(hardwareMap: HardwareMap, var id: String) :
    CRServoEx(hardwareMap, id) {

    var runMode: RunMode = RunMode.RawPower
        set(runMode) {
            super.setRunMode(runMode)
        }

    val inTolerance: Boolean
        get() {
            return when (runMode) {
                RunMode.OptimizedPositionalControl -> positionController.atSetPoint()
                RunMode.RawPower -> veloController.atSetPoint()
            }
        }

    /**
     * Software forward limit, ONLY for percent-output control.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software forward limit, ONLY for percent-output control.
     */
    var reverseLimit: () -> Boolean = { false }

    var minPercentOutput = -1.0
        set(precentOutput) {
            field = precentOutput.coerceAtLeast(-1.0)
        }
    var maxPercentOutput = 1.0
        set(precentOutput) {
            field = precentOutput.coerceAtMost(1.0)
        }

    var position
        get() = (currentPosition / cpr).rotations
        set(position) {
            positionSetpoint =
                position.degrees.coerceIn(minimumPosition.degrees, maximumPosition.degrees).degrees
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

    var velocity: AngularVelocity
        get() = (super.correctedVelocity / cpr).rpm
        set(velocity) {
            when (velocity) {
                0.rpm -> super.set(0.0)
                else -> velocitySetpoint = velocity.coerceIn((maxRPM * -1).rpm, maxRPM.rpm)
            }
        }
    var velocitySetpoint: AngularVelocity
        get() = veloController.setPoint.rpm
        set(velocitySetpoint) {
            when (runMode) {
                RunMode.RawPower -> veloController.setPoint = velocitySetpoint.asRpm
                else -> robotPrintError("motor isn't in raw power control mode but in $runMode")
            }
        }

    var positionSetpoint: Rotation2d
        get() = positionController.setPoint.degrees
        set(positionSetpoint) {
            when (runMode) {
                RunMode.OptimizedPositionalControl -> positionController.setPoint =
                    positionSetpoint.degrees

                else -> robotPrintError("motor isn't in position control mode but in $runMode")
            }
        }


    /**
     * percentOutput is clamped between properties minPercentOutput and maxPercentOutput.
     */
    var precentOutput: PercentOutput
        get() = get()
        set(precentOutput) {
            if (runMode == RunMode.RawPower) {
                if ((forwardLimit() && precentOutput > 0.0) || (reverseLimit() && precentOutput < 0.0)) {
                    super.set(0.0)
                } else if (inverted) {
                    set(precentOutput * -1)
                } else {
                    set(precentOutput)
                }
            } else {
                robotPrintError("motor isn't in raw power mode $runMode")
            }
        }

    var pidfGains: PIDGains
        get() {
            return when (runMode) {
                RunMode.RawPower -> PIDGains(
                    veloController.coefficients[0],
                    veloController.coefficients[1],
                    veloController.coefficients[2],
                    veloController.coefficients[3]
                )

                RunMode.OptimizedPositionalControl -> PIDGains(
                    positionController.coefficients[0],
                    positionController.coefficients[1],
                    positionController.coefficients[2],
                    positionController.coefficients[3]
                )
            }
        }
        set(value) {
            positionController.setPIDF(value.kP, value.kI, value.kD, value.kFF)
            veloController.setPIDF(value.kP, value.kI, value.kD, value.kFF)
            feedforward = SimpleMotorFeedforward(value.kS, value.KV, value.Ka)
        }

}
