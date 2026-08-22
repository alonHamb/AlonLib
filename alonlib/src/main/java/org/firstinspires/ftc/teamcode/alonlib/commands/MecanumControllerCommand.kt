package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDController
import org.firstinspires.ftc.teamcode.alonlib.math.control.ProfiledPIDController
import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveMotorVoltages
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveWheelSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.Trajectory

/**
 * Follows [trajectory] with a mecanum drive, using x/y/theta PID controllers to correct pose
 * error plus (optionally) per-wheel velocity PID/feedforward -- a complete, "just wire it up"
 * trajectory follower. The robot turns towards the trajectory's *final* heading throughout, not
 * the heading at each sampled state.
 *
 * Omit [feedforward]/[maxWheelVelocityMetersPerSecond]'s per-wheel controllers
 * ([frontLeftController] etc.) and [currentWheelSpeeds] for the lighter-weight mode: [outputWheelSpeeds]
 * then receives raw target wheel speeds instead of [outputDriveVoltages] receiving voltages.
 */
class MecanumControllerCommand(
    private val trajectory: Trajectory,
    private val pose: () -> Pose2d,
    private val kinematics: MecanumDriveKinematics,
    private val xController: PIDController,
    private val yController: PIDController,
    private val thetaController: ProfiledPIDController,
    private val maxWheelVelocityMetersPerSecond: Double,
    private val feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0),
    private val frontLeftController: PIDController? = null,
    private val rearLeftController: PIDController? = null,
    private val frontRightController: PIDController? = null,
    private val rearRightController: PIDController? = null,
    private val currentWheelSpeeds: (() -> MecanumDriveWheelSpeeds)? = null,
    private val outputDriveVoltages: ((MecanumDriveMotorVoltages) -> Unit)? = null,
    private val outputWheelSpeeds: ((MecanumDriveWheelSpeeds) -> Unit)? = null,
) : CommandBase() {

    private val usePid = frontLeftController != null

    private lateinit var prevSpeeds: MecanumDriveWheelSpeeds
    private var prevTime = 0.0
    private lateinit var finalPose: Pose2d
    private var startNanos = 0L

    override fun initialize() {
        val initialState = trajectory.sample(0.0)
        finalPose = trajectory.sample(trajectory.totalTimeSeconds).pose

        val initialXVelocity = initialState.velocityMetersPerSecond * initialState.pose.rotation.cos
        val initialYVelocity = initialState.velocityMetersPerSecond * initialState.pose.rotation.sin

        prevSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds(initialXVelocity, initialYVelocity, 0.0))
        prevTime = 0.0
        startNanos = System.nanoTime()
    }

    override fun execute() {
        val curTime = (System.nanoTime() - startNanos) / 1e9
        val dt = curTime - prevTime

        val desiredState = trajectory.sample(curTime)
        val desiredPose = desiredState.pose
        val currentPose = pose()

        val poseError = desiredPose.relativeTo(currentPose)

        var targetXVel = xController.calculate(currentPose.translation.x, desiredPose.translation.x)
        var targetYVel = yController.calculate(currentPose.translation.y, desiredPose.translation.y)
        val targetAngularVel = thetaController.calculate(currentPose.rotation.radians, finalPose.rotation.radians)

        val vRef = desiredState.velocityMetersPerSecond
        targetXVel += vRef * poseError.rotation.cos
        targetYVel += vRef * poseError.rotation.sin

        val targetWheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds(targetXVel, targetYVel, targetAngularVel))
        targetWheelSpeeds.desaturate(maxWheelVelocityMetersPerSecond)

        if (usePid) {
            val currentSpeeds = currentWheelSpeeds!!()

            fun output(target: Double, prev: Double, current: Double, controller: PIDController) =
                feedforward.calculate(target, (target - prev) / dt) + controller.calculate(current, target)

            outputDriveVoltages!!(
                MecanumDriveMotorVoltages(
                    frontLeft = output(targetWheelSpeeds.frontLeft, prevSpeeds.frontLeft, currentSpeeds.frontLeft, frontLeftController!!),
                    frontRight = output(targetWheelSpeeds.frontRight, prevSpeeds.frontRight, currentSpeeds.frontRight, frontRightController!!),
                    rearLeft = output(targetWheelSpeeds.rearLeft, prevSpeeds.rearLeft, currentSpeeds.rearLeft, rearLeftController!!),
                    rearRight = output(targetWheelSpeeds.rearRight, prevSpeeds.rearRight, currentSpeeds.rearRight, rearRightController!!),
                ),
            )
        } else {
            outputWheelSpeeds!!(targetWheelSpeeds)
        }

        prevTime = curTime
        prevSpeeds = targetWheelSpeeds
    }

    override fun isFinished() = (System.nanoTime() - startNanos) / 1e9 > trajectory.totalTimeSeconds
}
