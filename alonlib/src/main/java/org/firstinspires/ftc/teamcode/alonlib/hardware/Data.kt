package org.firstinspires.ftc.teamcode.alonlib.hardware

import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

object Data {

    object Servos {
        enum class Mode {
            CR,
            FULL_RANGE
        }

        /**
         * @param range the servo's total mechanical sweep, in degrees. Stored as a plain [Double]
         * rather than a [com.seattlesolvers.solverslib.geometry.Rotation2d] because every built-in
         * servo's sweep (300deg/350deg) exceeds the (-180, 180] domain that Rotation2d normalizes into,
         * which would silently corrupt the value (e.g. 300deg -> -60deg).
         */
        enum class Type(val range: Double, val maxSpeed: AngularVelocity) {
            Torque(300.0, 50.rpm),
            Speed(300.0, 111.11.rpm),
            SuperSpeed(300.0, 232.558.rpm),
            AxonMax(350.0, 86.905.rpm),
            AxonMini(350.0, 111.111.rpm),


        }
    }
}
