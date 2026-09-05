package org.firstinspires.ftc.teamcode.alonlib.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.Time
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.microseconds
import org.firstinspires.ftc.teamcode.alonlib.units.rpm


object Data {

    object Motors {
        /** The direction the motor rotates. */
        enum class Direction(val multiplier: Int) { Forward(1), Reverse(-1) }

        /** GoBILDA yellow-jacket gearbox presets -- ticks/rev and free-run RPM per ratio. */
        enum class GoBILDA(val cpr: Double, val rpm: Double) {
            RPM_30(5264.0, 30.0), RPM_43(3892.0, 43.0), RPM_60(2786.0, 60.0), RPM_84(1993.6, 84.0),
            RPM_117(1425.2, 117.0), RPM_223(753.2, 223.0), RPM_312(537.6, 312.0), RPM_435(383.6, 435.0),
            RPM_1150(145.6, 1150.0), RPM_1620(103.6, 1620.0), BARE(28.0, 6000.0),
        }

        enum class RunMode { VelocityControl, PositionControl, RawPower }
	    enum class DistanceMode { LINEAR,ANGULAR }

        enum class ZeroPowerBehavior(val sdkBehavior: DcMotor.ZeroPowerBehavior) {
            Unknown(DcMotor.ZeroPowerBehavior.UNKNOWN),
            Brake(DcMotor.ZeroPowerBehavior.BRAKE),
            Float(DcMotor.ZeroPowerBehavior.FLOAT),
        }
    }

    object Servos {
        enum class Mode {
            Cr,
            FullRange
        }

        /**
         * @param range the servo's total mechanical sweep, in degrees. Stored as a plain [Double]
         * rather than a [org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d] because every built-in
         * servo's sweep (300deg/350deg) exceeds the (-180, 180] domain that Rotation2d normalizes into,
         * which would silently corrupt the value (e.g. 300deg -> -60deg).
         */
        enum class Type(val range: Rotation2d, val maxSpeed: AngularVelocity, val fullRangePwmRange: Pair<Time, Time>, val crPwmRange: Pair<Time, Time>) {
            Torque(300.degrees, 50.rpm, 500.microseconds to 2500.microseconds, 1000.microseconds to 2000.microseconds),
            Speed(300.degrees, 111.11.rpm, 500.microseconds to 2500.microseconds, 1000.microseconds to 2000.microseconds),
            SuperSpeed(300.degrees, 232.558.rpm, 500.microseconds to 2500.microseconds, 1000.microseconds to 2000.microseconds),
            AxonMax(350.degrees, 86.905.rpm, 500.microseconds to 2500.microseconds, 500.microseconds to 2500.microseconds),
            AxonMini(350.degrees, 111.111.rpm, 500.microseconds to 2500.microseconds, 500.microseconds to 2500.microseconds),


        }
    }
}
