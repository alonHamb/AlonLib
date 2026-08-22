package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.Trajectory
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * A nonlinear time-varying feedback controller that drives a unicycle-model (differential drive)
 * robot along a trajectory using its *global* pose, rather than just per-wheel PID -- so it can
 * still converge even after the robot has drifted off the path. Named for the Italian acronym in
 * the paper it's from ("Robotica Articolata e Mobile per i SErvizi e le TEcnologie").
 *
 * See section 8.2.2 ("Ramsete unicycle controller") of *Controls Engineering in the FIRST
 * Robotics Competition* for the derivation.
 */
class RamseteController(private val b: Double = 2.0, private val zeta: Double = 0.7) {

    var poseError = Pose2d.kZero
        private set

    var poseTolerance = Pose2d.kZero

    var enabled = true

    fun atReference(): Boolean {
        val eTranslate = poseError.translation
        val eRotate = poseError.rotation
        val tolTranslate = poseTolerance.translation
        val tolRotate = poseTolerance.rotation
        return abs(eTranslate.x) < tolTranslate.x && abs(eTranslate.y) < tolTranslate.y &&
                abs(eRotate.radians) < tolRotate.radians
    }

    /**
     * @returns the chassis speeds that drive [currentPose] towards [poseRef], tracking the
     * reference [linearVelocityRefMeters]/[angularVelocityRefRadiansPerSecond].
     */
    fun calculate(
        currentPose: Pose2d,
        poseRef: Pose2d,
        linearVelocityRefMeters: Double,
        angularVelocityRefRadiansPerSecond: Double,
    ): ChassisSpeeds {
        if (!enabled) {
            return ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond)
        }

        poseError = poseRef.relativeTo(currentPose)

        val eX = poseError.x
        val eY = poseError.y
        val eTheta = poseError.rotation.radians
        val vRef = linearVelocityRefMeters
        val omegaRef = angularVelocityRefRadiansPerSecond

        // k = 2*zeta*sqrt(omega_ref^2 + b*v_ref^2)
        val k = 2.0 * zeta * sqrt(omegaRef.pow(2) + b * vRef.pow(2))

        // v_cmd = v_ref*cos(e_theta) + k*e_x
        // omega_cmd = omega_ref + k*e_theta + b*v_ref*sinc(e_theta)*e_y
        return ChassisSpeeds(
            vRef * poseError.rotation.cos + k * eX,
            0.0,
            omegaRef + k * eTheta + b * vRef * sinc(eTheta) * eY,
        )
    }

    /** @returns the chassis speeds that drive [currentPose] towards [desiredState]. */
    fun calculate(currentPose: Pose2d, desiredState: Trajectory.State) = calculate(
        currentPose,
        desiredState.pose,
        desiredState.velocityMetersPerSecond,
        desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter,
    )

    private fun sinc(x: Double) = if (abs(x) < 1e-9) 1.0 - 1.0 / 6.0 * x * x else kotlin.math.sin(x) / x
}
