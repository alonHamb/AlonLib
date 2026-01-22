package org.firstinspires.ftc.teamcode.alonlib.motors

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor.Direction.FORWARD
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.alonlib.math.clamp
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rotations
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.alonlib.units.rps

class HaDcMotor(hardwareMap: HardwareMap, id: String, cpr: Double, rpm: Double) :
    MotorEx(hardwareMap, id, cpr, rpm) {
    constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA) : this(hardwareMap, id, type.cpr, type.rpm)

    /** the current position setpoint of the motor **/
    var positionSetpoint: Rotation2d = 0.0.degrees
    var positionError: Rotation2d = 0.0.degrees
    var direction: Direction = FORWARD
    var targetVelocity: AngularVelocity = 0.0.rpm
    var velocityError: AngularVelocity = 0.0.rpm

    /**
     * Software forward limit, ONLY for percent-output control.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software forward limit, ONLY for percent-output control.
     */
    var reverseLimit: () -> Boolean = { false }
    var minPercentOutput = -1.0
        set(value) {
            field = value.coerceAtLeast(-1.0)
        }
    var maxPercentOutput = 1.0
        set(value) {
            field = value.coerceAtMost(1.0)
        }


    fun configPID(gains: PIDGains) {
        positionCoefficient = gains.kP
        setVeloCoefficients(gains.kP, gains.kI, gains.kD)
        setFeedforwardCoefficients(gains.kS, gains.KV, gains.Ka)
    }

    /**
     * percentOutput is clamped between properties minPercentOutput and maxPercentOutput.
     */
    override fun set(output: PercentOutput) {
        if (maxPercentOutput <= minPercentOutput) {
            robotPrintError("maxPercentOutput is smaller or equal to minPercentOutput")
        } else {
            super.set(clamp(output * direction.multiplier, minPercentOutput, maxPercentOutput))
        }
    }

    fun setWithLimits(output: PercentOutput) {
        if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
            super.set(0.0)
        } else {
            set(output * direction.multiplier)
        }
    }

    fun setPositionSetPoint(setPoint: Rotation2d) {
        setTargetPosition((setPoint.rotations.toDouble() / cpr).toInt())
        positionSetpoint = setPoint
    }

    fun setVelocitySetpoint(setPoint: AngularVelocity) {
        targetVelocity = setPoint
        velocity = setPoint.asRps / cpr
    }

    fun getPositionSetPoint(): Rotation2d {
        return positionSetpoint
    }


    fun getCurrentVelocity(): AngularVelocity {
        return (velocity / cpr).rps
    }

}