package org.firstinspires.ftc.teamcode.alonlib.drives.coaxialSwerve

import org.firstinspires.ftc.teamcode.alonlib.drives.HaRobotDrive
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.RunMode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Motors.ZeroPowerBehavior
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Vector2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * A standard 4-module coaxial swerve drivetrain: 4 drive motors + 4 pod-rotation servos (Axon-style,
 * with absolute encoders read via [swervoAngles]). [motors]/[swervos]/[swervoAngles] are ordered
 * starting from front-right, going counterclockwise.
 */
class HaCoaxialSwerveDrivetrain(
	private val trackWidth: Double,
	private val wheelBase: Double,
	private val maxSpeed: Double,
	swervoPidf: PIDFController,
	motors: Array<HaMotor>,
	swervos: Array<HaServo>,
	swervoAngles: Array<() -> Double>,
) : HaRobotDrive() {

	init {
		require(motors.size == 4 && swervos.size == 4 && swervoAngles.size == 4) { "Hardware lists for swerve modules must have exactly 4 objects each" }
		require(trackWidth > 0 && wheelBase > 0 && maxSpeed > 0) { "trackWidth, wheelBase, and maxSpeed must have positive values" }
		for (motor in motors) {
			motor.runMode = RunMode.RAW_POWER
			motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
		}
	}

	private val maxAngularSpeed = maxSpeed / hypot(trackWidth / 2, wheelBase / 2)

	val modules = arrayOf(
		HaCoaxialSwerveModule(motors[0], swervos[0], swervoAngles[0], Vector2d(trackWidth / 2, wheelBase / 2), maxSpeed, swervoPidf),
		HaCoaxialSwerveModule(motors[1], swervos[1], swervoAngles[1], Vector2d(-trackWidth / 2, wheelBase / 2), maxSpeed, swervoPidf),
		HaCoaxialSwerveModule(motors[2], swervos[2], swervoAngles[2], Vector2d(-trackWidth / 2, -wheelBase / 2), maxSpeed, swervoPidf),
		HaCoaxialSwerveModule(motors[3], swervos[3], swervoAngles[3], Vector2d(trackWidth / 2, -wheelBase / 2), maxSpeed, swervoPidf),
	)

	var targetVelocity = ChassisSpeeds()
		private set

	/** Sets the robot-centric target velocity, scaled down (preserving direction) if it exceeds [maxSpeed]/[getMaxOutput]. */
	fun setTargetVelocity(velocity: ChassisSpeeds) {
		val maxAllowedLinearSpeed = maxSpeed * maxOutput
		val maxAllowedAngularSpeed = maxAngularSpeed * maxOutput

		var scale = 1.0
		scale = maxOf(scale, abs(velocity.vx / maxAllowedLinearSpeed))
		scale = maxOf(scale, abs(velocity.vy / maxAllowedLinearSpeed))
		scale = maxOf(scale, abs(velocity.omega / maxAllowedAngularSpeed))

		targetVelocity = ChassisSpeeds(velocity.vx / scale, velocity.vy / scale, velocity.omega / scale)
	}

	/** Drives every module towards the current [targetVelocity]. */
	fun update(): Array<Vector2d> {
		val moduleVelocities = Array(modules.size) { modules[it].calculateVectorRobotCentric(targetVelocity) }
		val magnitudes = DoubleArray(moduleVelocities.size) { moduleVelocities[it].magnitude() }

		normalize(magnitudes)

		for (i in modules.indices) {
			val scaled = moduleVelocities[i].scale(magnitudes[i] / moduleVelocities[i].magnitude())
			modules[i].updateModuleWithVelocity(scaled)
			moduleVelocities[i] = scaled
		}

		return moduleVelocities
	}

	fun updateWithTargetVelocity(velocity: ChassisSpeeds) {
		setTargetVelocity(velocity)
		update()
	}

	/** Points every module diagonally outward (an X shape), to resist being pushed. */
	fun updateWithXLock() {
		for (i in modules.indices) {
			val angle = -PI / 4 + PI / 2 * i
			modules[i].updateModuleWithVelocity(Vector2d(cos(angle), sin(angle)).scale(0.0001))
		}
	}

	override fun stop() = modules.forEach { it.stop() }
}
