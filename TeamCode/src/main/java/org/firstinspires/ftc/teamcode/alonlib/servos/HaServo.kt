package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.teamcode.alonlib.robotPrint
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

class HaServo(
    hMap: HardwareMap?,
    id: String,
    val range: Rotation2d
) : ServoEx(
    hMap,
    id,
    0.0,
    300.0
) {

    fun setPositionSetPoint(position: Rotation2d) {
        robotPrint("position: ${position.degrees} range: ${range.degrees} ")
        super.set(position.degrees)
    }

    fun setRunningDirection(direction: Direction) {
        if (direction == Direction.COUNTER_CLOCKWISE) {
            setInverted(true)
        } else {
            setInverted(false)
        }
    }

    fun getPositionSetPoint(): Rotation2d {
        return (rawPosition * range.degrees).degrees
    }

    enum class Direction {

        CLOCKWISE,
        COUNTER_CLOCKWISE

    }

}