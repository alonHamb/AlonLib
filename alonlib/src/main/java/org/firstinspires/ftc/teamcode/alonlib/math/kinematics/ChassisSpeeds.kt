package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d

/**
 * A robot chassis's velocity: [vx]/[vy] meters/sec and [omega] radians/sec.
 *
 * Similar fields to [Twist2d] but a different meaning -- a `Twist2d` is a pose *delta*, this is a
 * *velocity*. A non-holonomic drivetrain (differential) should never have a nonzero [vy]; a
 * holonomic one (mecanum, swerve) usually has all three.
 */
class ChassisSpeeds(var vx: Double = 0.0, var vy: Double = 0.0, var omega: Double = 0.0) {

    fun toTwist2d(dtSeconds: Double) = Twist2d(vx * dtSeconds, vy * dtSeconds, omega * dtSeconds)

    operator fun plus(other: ChassisSpeeds) = ChassisSpeeds(vx + other.vx, vy + other.vy, omega + other.omega)
    operator fun minus(other: ChassisSpeeds) = ChassisSpeeds(vx - other.vx, vy - other.vy, omega - other.omega)
    operator fun unaryMinus() = ChassisSpeeds(-vx, -vy, -omega)
    operator fun times(scalar: Double) = ChassisSpeeds(vx * scalar, vy * scalar, omega * scalar)
    operator fun div(scalar: Double) = ChassisSpeeds(vx / scalar, vy / scalar, omega / scalar)

    override fun equals(other: Any?): Boolean {
        if (other !is ChassisSpeeds) return false
        return vx == other.vx && vy == other.vy && omega == other.omega
    }

    override fun hashCode() = arrayOf(vx, vy, omega).contentHashCode()

    override fun toString() = "ChassisSpeeds(vx=$vx m/s, vy=$vy m/s, omega=$omega rad/s)"

    companion object {
        /**
         * Converts continuous-time [vx]/[vy]/[omega] into the discrete-time speeds that, applied
         * for one [dtSeconds] timestep, move the robot exactly `vx*dt`/`vy*dt`/`omega*dt` --
         * compensating for the translational skew a holonomic drivetrain gets from translating and
         * rotating at once. Scaling the result down afterwards (e.g. desaturating swerve module
         * speeds) reintroduces a skew this doesn't account for.
         */
        fun discretize(vx: Double, vy: Double, omega: Double, dtSeconds: Double): ChassisSpeeds {
            val desiredDeltaPose = Pose2d(vx * dtSeconds, vy * dtSeconds, Rotation2d(omega * dtSeconds))
            val twist = Pose2d.kZero.log(desiredDeltaPose)
            return ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds)
        }

        fun discretize(continuousSpeeds: ChassisSpeeds, dtSeconds: Double) =
            discretize(continuousSpeeds.vx, continuousSpeeds.vy, continuousSpeeds.omega, dtSeconds)

        /** Converts field-relative speeds (facing [robotAngle]) into robot-relative speeds. */
        fun fromFieldRelativeSpeeds(vx: Double, vy: Double, omega: Double, robotAngle: Rotation2d): ChassisSpeeds {
            val rotated = Translation2d(vx, vy).rotateBy(-robotAngle)
            return ChassisSpeeds(rotated.x, rotated.y, omega)
        }

        fun fromFieldRelativeSpeeds(fieldRelativeSpeeds: ChassisSpeeds, robotAngle: Rotation2d) =
            fromFieldRelativeSpeeds(fieldRelativeSpeeds.vx, fieldRelativeSpeeds.vy, fieldRelativeSpeeds.omega, robotAngle)

        /** Converts robot-relative speeds (facing [robotAngle]) into field-relative speeds. */
        fun fromRobotRelativeSpeeds(vx: Double, vy: Double, omega: Double, robotAngle: Rotation2d): ChassisSpeeds {
            val rotated = Translation2d(vx, vy).rotateBy(robotAngle)
            return ChassisSpeeds(rotated.x, rotated.y, omega)
        }

        fun fromRobotRelativeSpeeds(robotRelativeSpeeds: ChassisSpeeds, robotAngle: Rotation2d) =
            fromRobotRelativeSpeeds(robotRelativeSpeeds.vx, robotRelativeSpeeds.vy, robotRelativeSpeeds.omega, robotAngle)
    }
}
