package org.firstinspires.ftc.teamcode.alonlib.motors

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rotations
import com.hamosad1657.lib.units.rpm
import com.hamosad1657.lib.units.rps
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.alonlib.math.clamp
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput

class HaDcMotor(hardwareMap: HardwareMap, id: String, type: GoBILDA) :
    MotorEx(hardwareMap, id, type) {
    /** the current position setpoint of the motor **/
    var positionSetpoint: Rotation2d = 0.0.degrees

    var targetVelocity: AngularVelocity = 0.0.rpm

    /**
     * Software forward limit, ONLY for percent-output control.
     * WILL NOT work in closed-loop control onboard the motor controller, since that
     * uses SparkMaxPIDController, which is package-private and cannot be extended by us.
     *
     * - If possible, use hardware limits by wiring switches to the data port.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software forward limit, ONLY for percent-output control.
     * WILL NOT work in closed-loop control onboard the motor controller, since that
     * uses SparkMaxPIDController, which is package-private and cannot be extended by us.
     *
     * - If possible, use hardware limits by wiring switches to the data port.
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
        if (runmode == RunMode.PositionControl) {
            positionCoefficient = (gains.kP)
        } else if (runmode == RunMode.VelocityControl) {
            setVeloCoefficients(gains.kP, gains.kI, gains.kD)
        }
    }

    /**
     * percentOutput is clamped between properties minPercentOutput and maxPercentOutput.
     */
    override fun set(output: PercentOutput) {
        if (maxPercentOutput <= minPercentOutput) {
            robotPrintError("maxPercentOutput is smaller or equal to minPercentOutput")
        } else {
            super.set(clamp(output, minPercentOutput, maxPercentOutput))
        }
    }

    fun setWithLimits(output: PercentOutput) {
        if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
            super.set(0.0)
        } else {
            set(output)
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