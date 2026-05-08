package org.firstinspires.ftc.teamcode.alonlib.hardware

import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

object Data {

    object Servos {
        enum class Mode {
            CR,
            FULL_RANGE
        }

        enum class Type(val range: Rotation2d, val maxSpeed: AngularVelocity) {
            Torque(300.degrees, 50.rpm),
            Speed(300.degrees, 111.11.rpm),
            SuperSpeed(300.degrees, 232.558.rpm),
            AxonMax(357.4.degrees, 86.905.rpm),
            AxonMini(357.4.degrees, 111.111.rpm),


        }
    }
}
