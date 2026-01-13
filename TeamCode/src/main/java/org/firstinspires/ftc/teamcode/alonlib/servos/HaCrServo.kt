package org.firstinspires.ftc.teamcode.alonlib.servos

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import com.seattlesolvers.solverslib.hardware.motors.Motor.Direction.FORWARD
import org.firstinspires.ftc.teamcode.alonlib.math.clamp
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.alonlib.units.rps

class HaCrServo(hardwareMap: HardwareMap, var id: String, var runMode: RunMode) :
    CRServoEx(hardwareMap, id) {

    var positionSetpoint: Rotation2d = 0.0.degrees
    var positionError: Rotation2d = 0.0.degrees
    var direction: Direction = FORWARD
    var velocitySetpoint: AngularVelocity = 0.0.rpm
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

    fun getCurrentVelocity(): AngularVelocity {
        return (encoder.correctedVelocity / cpr).rps
    }


    fun configPIDF(gains: PIDGains) {
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
        if (runMode == RunMode.RawPower) {
            if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
                super.set(0.0)
            } else {
                set(output * direction.multiplier)
            }
        } else robotPrintError("servo ($id) is set to position control cannot set raw power")

    }

    fun setPositionSetPoint(setpoint: Rotation2d) {
        if (runMode == RunMode.OptimizedPositionalControl) {
            positionSetpoint = setpoint
            super.set(setpoint.degrees)
        } else {
            robotPrintError("you cant give raw power servo a position setpoint")
            set(0.0)
        }
    }


    fun setVelocitySetpoint(setPoint: AngularVelocity) {
        velocitySetpoint = setPoint
        super.veloController.setPoint = velocitySetpoint.asRps / cpr


    }

    fun getPositionSetPoint(): Rotation2d {
        return positionSetpoint
    }

}