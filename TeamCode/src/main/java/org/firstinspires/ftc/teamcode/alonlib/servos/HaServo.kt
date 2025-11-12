package org.firstinspires.ftc.teamcode.alonlib.servos

import com.hamosad1657.lib.units.degrees
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.teamcode.alonlib.math.mapRange

class HaServo(
    hMap: HardwareMap?,
    id: String,
    var minPosition: Rotation2d = (-150).degrees,
    var maxPosition: Rotation2d = 150.degrees
) : ServoEx(hMap, id) {


    fun setPosition(position: Rotation2d) {
        super.set(positionToServoPosition(position))
    }

    fun positionToServoPosition(position: Rotation2d): Double {
        return mapRange(position.degrees, minPosition.degrees, maxPosition.degrees, 0.0, 1.0)
    }

    fun getPosition(): Rotation2d {
        return mapRange(rawPosition, 0.0, 1.0, minPosition.degrees, maxPosition.degrees).degrees
    }

}