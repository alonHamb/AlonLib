package org.firstinspires.ftc.teamcode.alonlib.motors

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

class HaDcMotor(hardwareMap: HardwareMap, id: String, cpr: Double, rpm: Double) :
    MotorEx(hardwareMap, id, cpr, rpm) {
    constructor(hardwareMap: HardwareMap, id: String, type: GoBILDA) : this(hardwareMap, id, type.cpr, type.rpm)


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
        veloController.setPIDF(gains.kP, gains.kI, gains.kD, gains.kFF)
    }

    /**
     * percentOutput is clamped between properties minPercentOutput and maxPercentOutput.
     */

    fun setPrecentOutput(output: PercentOutput) {
        if (runmode == RunMode.RawPower) {
            if (inverted) {
                set(output * -1)
            } else {
                set(output)
            }
        } else {
            robotPrintError("motor isn't in raw power mode $runmode")
        }
    }

    fun setPrecentOutputWithLimits(output: PercentOutput) {
        if (runmode == RunMode.RawPower) {
            if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
                super.set(0.0)
            } else if (inverted) {
                set(output * -1)
            } else {
                set(output)
            }
        } else {
            robotPrintError("motor isn't in raw power mode $runmode")
        }
    }

    fun setPositionSetPoint(setPoint: Rotation2d) {
        if (runmode == RunMode.PositionControl) {
            positionController.setPoint = setPoint.degrees
        } else {
            robotPrintError("motor isn't in position control mode $runmode")
        }
    }

    fun setVelocitySetpoint(setPoint: AngularVelocity) {
        if (runmode == RunMode.VelocityControl) {
            veloController.setPoint = setPoint.asRpm
        } else {
            robotPrintError("motor isn't in velocity control mode $runmode")
        }
    }

    fun getPositionSetPoint(): Rotation2d {
        return positionController.setPoint.degrees
    }


    fun getCurrentVelocity(): AngularVelocity {
        return veloController.setPoint.rpm
    }

}