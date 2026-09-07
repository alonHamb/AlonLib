package org.firstinspires.ftc.teamcode.alonlib.drives.coaxialSwerve

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.math.angleModulus
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDFController
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Vector2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

/**
 * One coaxial swerve module: a drive [motor] plus a pod-rotation [servo] (a [HaServo] in
 * [HaServo.Mode.CR]) steered in software towards [absolutePositionRadians] (an absolute encoder
 * reading -- e.g. an Axon servo's feedback wire -- where 0 means "wheel facing forward, positive
 * motor power drives the robot forward"). [offset] is this module's position relative to the
 * robot's center, in inches.
 */
class HaCoaxialSwerveModule(
	private val motor: HaMotor,
	private val servo: HaServo,
	private val absolutePositionRadians: () -> Double,
	offset: Vector2d,
	private val maxSpeed: Double,
	private var servoPidf: PIDFController,
) {

	// The angle tangential to the circle traced by the 4 modules, relative to the robot -- and
	// that circle's circumference. Both are constant per module, so computed once here.
	private val tangentialAngle = offset.angle()
	private val circumference = offset.magnitude() * 2 * PI

	var targetVelocity = Vector2d()
		private set

	var angleError = 0.0
		private set

	var wheelFlipped = false
		private set

	/** The (unnormalized) module velocity vector for tracking [target], robot-centric. See https://www.desmos.com/calculator/8sm94so6ud. */
	fun calculateVectorRobotCentric(target: ChassisSpeeds): Vector2d {
		val turningMagnitude = circumference * target.omega / (2 * PI)
		val turningVector = Vector2d(cos(tangentialAngle) * turningMagnitude, sin(tangentialAngle) * turningMagnitude)
		return turningVector + Vector2d(target.vx, target.vy)
	}

	fun setTargetVelocity(velocity: Vector2d) {
		targetVelocity = velocity
	}

	/** Drives the hardware to follow [targetVelocity] (set via [setTargetVelocity] or [updateModuleWithVelocity]). */
	fun updateModule() {
		wheelFlipped = false
		angleError = angleModulus(angleModulus(targetVelocity.angle()) - absolutePositionRadians())
		if (abs(angleError) > PI / 2) {
			angleError += PI * -sign(angleError)
			wheelFlipped = true
		}

		motor.percentOutput = ((if (wheelFlipped) -1.0 else 1.0) * targetVelocity.magnitude() / maxSpeed * cos(angleError)).fraction
		servo.percentOutput = servoPidf.calculate(0.0, angleError)
	}

	fun updateModuleWithVelocity(velocity: Vector2d) {
		setTargetVelocity(velocity)
		updateModule()
	}

	fun stop() {
		this@HaCoaxialSwerveModule.servo.stop()
		motor.stop()
	}

	fun getPowerTelemetry() =
		"Motor=${"%.3f".format(motor.percentOutput.asFraction)},Servo=${"%.3f".format(servo.percentOutput)},Absolute Encoder=${"%.3f".format(absolutePositionRadians())}"

	fun setSwervoPidf(pidf: PIDFController) {
		servoPidf = pidf
	}
}
