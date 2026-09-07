package alonlib.math.kinematics

import alonlib.math.interpolate
import alonlib.math.interpolation.Interpolatable

/** Cumulative encoder distance traveled by each side of a differential drivetrain, in meters. */
data class DifferentialDriveWheelPositions(val left: Double = 0.0, val right: Double = 0.0) :
        Interpolatable<DifferentialDriveWheelPositions> {

    override fun interpolate(endValue: DifferentialDriveWheelPositions, t: Double) =
        DifferentialDriveWheelPositions(interpolate(left, endValue.left, t), interpolate(right, endValue.right, t))
}
