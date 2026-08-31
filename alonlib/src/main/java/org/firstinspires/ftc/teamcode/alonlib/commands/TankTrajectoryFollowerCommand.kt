package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDController
import org.firstinspires.ftc.teamcode.alonlib.math.control.RamseteController
import org.firstinspires.ftc.teamcode.alonlib.math.control.SimpleMotorFeedforward
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveWheelSpeeds
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.Trajectory

/**
 * Follows [trajectory] with a differential drive, using a [RamseteController] plus (optionally)
 * per-side velocity PID/feedforward -- a complete, "just wire it up" trajectory follower.
 *
 * Omit [feedforward]/[wheelSpeeds]/[leftController]/[rightController] for the lighter-weight mode
 * (e.g. if a smart motor controller already does onboard velocity PID): [output] then receives raw
 * wheel speeds from the RAMSETE controller instead of a voltage-like unitless output.
 */
class TankTrajectoryFollowerCommand(
	private val trajectory: Trajectory,
	private val pose: () -> Pose2d,
	private val follower: RamseteController,
	private val kinematics: DifferentialDriveKinematics,
	private val feedforward: SimpleMotorFeedforward? = null,
	private val wheelSpeeds: (() -> DifferentialDriveWheelSpeeds)? = null,
	private val leftController: PIDController? = null,
	private val rightController: PIDController? = null,
	private val output: (left: Double, right: Double) -> Unit,
) : CommandBase() {

	private val usePid = feedforward != null

	private var prevSpeeds = DifferentialDriveWheelSpeeds()
	private var prevTime = 0.0
	private var startNanos = 0L

	override fun initialize() {
		prevTime = 0.0
		val initialState = trajectory.sample(0.0)
		prevSpeeds = kinematics.toWheelSpeeds(
			ChassisSpeeds(initialState.velocityMetersPerSecond, 0.0, initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond),
		)
		startNanos = System.nanoTime()
		leftController?.reset()
		rightController?.reset()
	}

	override fun execute() {
		val curTime = (System.nanoTime() - startNanos) / 1e9
		val dt = curTime - prevTime

		val targetWheelSpeeds = kinematics.toWheelSpeeds(follower.calculate(pose(), trajectory.sample(curTime)))
		val leftSetpoint = targetWheelSpeeds.left
		val rightSetpoint = targetWheelSpeeds.right

		val (leftOutput, rightOutput) = if (usePid) {
			val leftFeedforward = feedforward!!.calculate(leftSetpoint, (leftSetpoint - prevSpeeds.left) / dt)
			val rightFeedforward = feedforward.calculate(rightSetpoint, (rightSetpoint - prevSpeeds.right) / dt)
			val currentSpeeds = wheelSpeeds!!()
			leftFeedforward + leftController!!.calculate(currentSpeeds.left, leftSetpoint) to
					rightFeedforward + rightController!!.calculate(currentSpeeds.right, rightSetpoint)
		} else {
			leftSetpoint to rightSetpoint
		}

		output(leftOutput, rightOutput)

		prevTime = curTime
		prevSpeeds = targetWheelSpeeds
	}

	override fun end(interrupted: Boolean) = output(0.0, 0.0)

	override fun isFinished() = (System.nanoTime() - startNanos) / 1e9 > trajectory.totalTimeSeconds
}
