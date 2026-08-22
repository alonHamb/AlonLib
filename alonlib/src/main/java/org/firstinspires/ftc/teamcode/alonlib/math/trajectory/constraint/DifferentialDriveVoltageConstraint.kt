package org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint

import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/** Caps trajectory acceleration so no wheel of a differential drivetrain ever needs more than [maxVoltage]. */
class DifferentialDriveVoltageConstraint(
    private val feedforward: SimpleMotorFeedforward,
    private val kinematics: DifferentialDriveKinematics,
    private val maxVoltage: Double,
) : TrajectoryConstraint {

    override fun getMaxVelocityMetersPerSecond(pose: Pose2d, curvatureRadPerMeter: Double, velocityMetersPerSecond: Double) =
        Double.POSITIVE_INFINITY

    override fun getMinMaxAccelerationMetersPerSecondSq(
        pose: Pose2d,
        curvatureRadPerMeter: Double,
        velocityMetersPerSecond: Double,
    ): TrajectoryConstraint.MinMax {
        val wheelSpeeds = kinematics.toWheelSpeeds(
            ChassisSpeeds(velocityMetersPerSecond, 0.0, velocityMetersPerSecond * curvatureRadPerMeter),
        )

        val maxWheelSpeed = max(wheelSpeeds.left, wheelSpeeds.right)
        val minWheelSpeed = min(wheelSpeeds.left, wheelSpeeds.right)

        val maxWheelAcceleration = feedforward.maxAchievableAcceleration(maxVoltage, maxWheelSpeed)
        val minWheelAcceleration = feedforward.minAchievableAcceleration(maxVoltage, minWheelSpeed)

        var maxChassisAcceleration: Double
        var minChassisAcceleration: Double

        if (velocityMetersPerSecond == 0.0) {
            maxChassisAcceleration = maxWheelAcceleration / (1 + kinematics.trackWidthMeters * abs(curvatureRadPerMeter) / 2)
            minChassisAcceleration = minWheelAcceleration / (1 + kinematics.trackWidthMeters * abs(curvatureRadPerMeter) / 2)
        } else {
            maxChassisAcceleration = maxWheelAcceleration /
                    (1 + kinematics.trackWidthMeters * abs(curvatureRadPerMeter) * sign(velocityMetersPerSecond) / 2)
            minChassisAcceleration = minWheelAcceleration /
                    (1 - kinematics.trackWidthMeters * abs(curvatureRadPerMeter) * sign(velocityMetersPerSecond) / 2)
        }

        if (kinematics.trackWidthMeters / 2 > 1 / abs(curvatureRadPerMeter)) {
            if (velocityMetersPerSecond > 0) {
                minChassisAcceleration = -minChassisAcceleration
            } else if (velocityMetersPerSecond < 0) {
                maxChassisAcceleration = -maxChassisAcceleration
            }
        }

        return TrajectoryConstraint.MinMax(minChassisAcceleration, maxChassisAcceleration)
    }
}
