package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.alonlib.math.mapRange
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.div

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
    constructor(hardwareMap: HardwareMap, id: String, range: Rotation2d) : this(hardwareMap, id, range / 2.0, -range / 2.0)
    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap, id, 0.degrees, 300.degrees)

    val position get() = mapRange(super.rawPosition, 0.0, 1.0, minPosition, maxPosition)
    val minPosition = MathUtils.normalizeDegrees(minPosition.degrees, true)
    val maxPosition = MathUtils.normalizeDegrees(maxPosition.degrees, true)


    fun setPositionSetPoint(position: Rotation2d) {
        super.set(MathUtils.normalizeDegrees(position.degrees, true))

    }

    fun setRunningDirection(direction: RunningDirection) {
        if (direction == RunningDirection.REVERSE) {
            setInverted(true)
        } else {
            setInverted(false)
        }
    }

    fun getPositionSetPoint(): Rotation2d {
        return (mapRange(rawPosition, 0.0, 1.0, minPosition, maxPosition)).degrees
    }

    enum class RunningDirection {

        FORWARD,
        REVERSE

    }

}