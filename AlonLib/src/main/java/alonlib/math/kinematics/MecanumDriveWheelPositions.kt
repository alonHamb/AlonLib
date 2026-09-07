package alonlib.math.kinematics

import alonlib.math.interpolate
import alonlib.math.interpolation.Interpolatable

/** Cumulative encoder distance traveled by each of a mecanum drivetrain's four wheels, in meters. */
data class MecanumDriveWheelPositions(
    val frontLeft: Double = 0.0,
    val frontRight: Double = 0.0,
    val rearLeft: Double = 0.0,
    val rearRight: Double = 0.0,
) : Interpolatable<MecanumDriveWheelPositions> {

    override fun interpolate(endValue: MecanumDriveWheelPositions, t: Double) = MecanumDriveWheelPositions(
        interpolate(frontLeft, endValue.frontLeft, t),
        interpolate(frontRight, endValue.frontRight, t),
        interpolate(rearLeft, endValue.rearLeft, t),
        interpolate(rearRight, endValue.rearRight, t),
    )
}
