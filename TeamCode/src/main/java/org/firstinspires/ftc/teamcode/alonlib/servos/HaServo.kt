package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.teamcode.alonlib.math.mapRange
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

class HaServo(
    hMap: HardwareMap?,
    id: String,
    val minPosition: Rotation2d,
    val maxPosition: Rotation2d
) : ServoEx(
    hMap,
    id,
) {


    fun setPositionSetPoint(position: Rotation2d) {
        super.set(mapRange(position.degrees, minPosition.degrees, maxPosition.degrees, 0.0, 1.0))
    }

    fun setRunningDirection(direction: Direction) {
        if (direction == Direction.COUNTER_CLOCKWISE) {
            setInverted(true)
        } else {
            setInverted(false)
        }
    }

    fun getPositionSetPoint(): Rotation2d {
        return (mapRange(rawPosition, 0.0, 1.0, minPosition.degrees, maxPosition.degrees)).degrees
    }

    enum class Direction {

        CLOCKWISE,
        COUNTER_CLOCKWISE

    }

}