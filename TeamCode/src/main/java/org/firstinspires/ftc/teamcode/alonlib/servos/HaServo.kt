package org.firstinspires.ftc.teamcode.alonlib.servos

import com.hamosad1657.lib.units.degrees
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrint

class HaServo(
    hMap: HardwareMap?,
    id: String,
    var minPosition: Rotation2d = 0.degrees,
    var maxPosition: Rotation2d = 300.degrees
) : ServoEx(
    hMap,
    id,
    minPosition.degrees,
    minPosition.degrees
) {


    fun setPositionSetPoint(position: Rotation2d) {
        robotPrint("Ha servo function")
        if (position.degrees == null) {
            super.set(0.0)
        } else
            super.set(position.degrees)dadsd
    }

    fun setRunningDirection(direction: Direction) {
        if (direction == Direction.COUNTER_CLOCKWISE) {
            setInverted(true)
        } else {
            setInverted(false)
        }
    }

    fun getPositionSetPoint(): Rotation2d {
        return (rawPosition * maxPosition.degrees).degrees
    }

    enum class Direction {

        CLOCKWISE,
        COUNTER_CLOCKWISE

    }

}