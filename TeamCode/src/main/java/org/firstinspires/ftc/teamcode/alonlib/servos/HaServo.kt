package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.alonlib.math.mapRange
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

class HaServo(
    hMap: HardwareMap?,
    id: String,
    minPosition: Rotation2d,
    maxPosition: Rotation2d
) : ServoEx(
    hMap,
    id,
    MathUtils.normalizeDegrees(minPosition.degrees, true),
    MathUtils.normalizeDegrees(maxPosition.degrees, true)
) {

    val minPosition = MathUtils.normalizeDegrees(minPosition.degrees, true)
    val maxPosition = MathUtils.normalizeDegrees(maxPosition.degrees, true)


    fun setPositionSetPoint(position: Rotation2d) {
        val normalizedPosition = MathUtils.normalizeDegrees(position.degrees, true)
        super.set(normalizedPosition)

    }

    fun setRunningDirection(direction: Direction) {
        if (direction == Direction.COUNTER_CLOCKWISE) {
            setInverted(true)
        } else {
            setInverted(false)
        }
    }

    fun getPositionSetPoint(): Rotation2d {
        return (mapRange(rawPosition, 0.0, 1.0, minPosition, maxPosition)).degrees
    }

    enum class Direction {

        CLOCKWISE,
        COUNTER_CLOCKWISE

    }

}