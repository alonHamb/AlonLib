package org.firstinspires.ftc.teamcode.alonlib.servos

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rotations
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput

class HaCrServo(hardwareMap: HardwareMap, id: String, var runMode: RunMode) :
    CRServoEx(hardwareMap, id) {

    var targetPosition = 0.degrees

    init {
        setRunMode(runMode)
    }


    fun configPIDGains(gains: PIDGains) {
        super.setPIDF(PIDFCoefficients(gains.kP, gains.kI, gains.kD,gains.kFF(targetPosition.degrees)))
    }

    fun setPrecentOutput(outPut: PercentOutput) {
        if (runMode == RunMode.RawPower) {
            super.set(outPut)

        } else {
            robotPrintError("you cant give position servo a precent output")
        }
    }

    fun setPositionSetPoint(setpoint: Rotation2d) {
        if (runMode == RunMode.OptimizedPositionalControl) {
            super.set(setpoint.degrees)
        } else {
            robotPrintError("you cant give raw power servo a position setpoint")
            set(0.0)
        }
    }

    fun getPositionSetpoint(): Rotation2d{
        return targetPosition
    }

    fun getPosition(): Rotation2d {
        return encoder.position.rotations
    }


}