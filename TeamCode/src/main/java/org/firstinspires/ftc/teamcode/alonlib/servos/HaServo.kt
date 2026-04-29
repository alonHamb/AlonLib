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
    constructor(hardwareMap: HardwareMap, id: String, range: Rotation2d) : this(
        hardwareMap,
        id,
        range / 2.0,
        -range / 2.0
    )

    constructor(hardwareMap: HardwareMap, id: String) : this(
        hardwareMap,
        id,
        0.degrees,
        300.degrees
    )

    var position: Rotation2d
        get() = mapRange(
            super.rawPosition,
            0.0,
            1.0,
            minPosition.degrees,
            maxPosition.degrees
        ).degrees
        set(position) {
            super.set(
                MathUtils.normalizeDegrees(
                    position.degrees.coerceIn(
                        minPosition.degrees,
                        maxPosition.degrees
                    ), true
                )
            )
        }
    val minPosition = MathUtils.normalizeDegrees(minPosition.degrees, true).degrees
    val maxPosition = MathUtils.normalizeDegrees(maxPosition.degrees, true).degrees

    var runningDirection: RunningDirection
        get() {
            return if (super.inverted) {
                RunningDirection.REVERSE
            } else {
                RunningDirection.FORWARD
            }
        }
        set(runningDirection) {
            if (runningDirection == RunningDirection.FORWARD) {
                super.setInverted(true)
            } else {
                super.setInverted(false)
            }
        }

    enum class RunningDirection {

        FORWARD,
        REVERSE

    }

}
