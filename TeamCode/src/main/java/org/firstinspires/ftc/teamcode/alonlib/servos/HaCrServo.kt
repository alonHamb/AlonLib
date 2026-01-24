package org.firstinspires.ftc.teamcode.alonlib.servos

import com.hamosad1657.lib.math.PIDGains
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

class HaCrServo(hardwareMap: HardwareMap, var id: String) :
    CRServoEx(hardwareMap, id) {

    var runMode: RunMode = RunMode.RawPower


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
        if (runMode == RunMode.RawPower) {
            if (inverted) {
                set(output * -1)
            } else {
                set(output)
            }
        } else {
            robotPrintError("motor isn't in raw power mode $runMode")
        }
    }

    fun setPrecentOutputWithLimits(output: PercentOutput) {
        if (runMode == RunMode.RawPower) {
            if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
                super.set(0.0)
            } else if (inverted) {
                set(output * -1)
            } else {
                set(output)
            }
        } else {
            robotPrintError("motor isn't in raw power mode $runMode")
        }
    }

    fun setPositionSetPoint(setPoint: Rotation2d) {
        if (runMode == RunMode.OptimizedPositionalControl) {
            positionController.setPoint = setPoint.degrees
        } else {
            robotPrintError("motor isn't in position control mode $runMode")
        }
    }

    override fun setRunMode(runMode: RunMode): CRServoEx {
        super.setRunMode(runMode)
        this.runMode = runMode
        return this
    }


    fun getPositionSetPoint(): Rotation2d {
        return positionController.setPoint.degrees
    }


}
