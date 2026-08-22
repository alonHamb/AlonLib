package org.firstinspires.ftc.teamcode.alonlib.drives.swerve.coaxial

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaCRServo
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
 * One coaxial swerve module: a drive [motor] plus a pod-rotation [swervo] (a [HaCRServo] with an
 * absolute encoder configured so 0 means "wheel facing forward, positive motor power drives the
 * robot forward"). [offset] is this module's position relative to the robot's center, in inches.
 */
class CoaxialSwerveModule(
    private val motor: HaMotor,
    private val swervo: HaCRServo,
    offset: Vector2d,
    private val maxSpeed: Double,
    swervoPidf: PIDFController,
) {
    private var swervoPidf = swervoPidf

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
        angleError = angleModulus(angleModulus(targetVelocity.angle()) - swervo.getAbsolutePositionRadians())
        if (abs(angleError) > PI / 2) {
            angleError += PI * -sign(angleError)
            wheelFlipped = true
        }

        motor.percentOutput = ((if (wheelFlipped) -1.0 else 1.0) * targetVelocity.magnitude() / maxSpeed * cos(angleError)).fraction
        swervo.percentOutput = swervoPidf.calculate(0.0, angleError)
    }

    fun updateModuleWithVelocity(velocity: Vector2d) {
        setTargetVelocity(velocity)
        updateModule()
    }

    fun stop() {
        swervo.stop()
        motor.stop()
    }

    fun setCachingTolerance(motorCachingTolerance: Double, swervoCachingTolerance: Double) = apply {
        motor.cachingTolerance = motorCachingTolerance
        swervo.cachingTolerance = swervoCachingTolerance
    }

    fun getPowerTelemetry() = "Motor=${"%.3f".format(motor.percentOutput.asFraction)},Servo=${"%.3f".format(swervo.percentOutput)},Absolute Encoder=${"%.3f".format(swervo.getAbsolutePositionRadians())}"

    fun setSwervoPidf(pidf: PIDFController) {
        swervoPidf = pidf
    }
}
