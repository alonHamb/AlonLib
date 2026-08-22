package org.firstinspires.ftc.teamcode.alonlib.drives

import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Vector2d
import org.firstinspires.ftc.teamcode.alonlib.units.fraction
import kotlin.math.cos
import kotlin.math.sin

/** A holonomic drivebase, either the classic 3-motor "H-drive" (two angled drive wheels plus a perpendicular slide wheel) or, given four motors, a mecanum-like layout. */
class HDrive(
    private val motors: Array<HaMotor>,
    private val leftMotorAngleRadians: Double = DEFAULT_LEFT_MOTOR_ANGLE,
    private val rightMotorAngleRadians: Double = DEFAULT_RIGHT_MOTOR_ANGLE,
    private val slideMotorAngleRadians: Double = DEFAULT_SLIDE_MOTOR_ANGLE,
) : RobotDrive() {

    constructor(left: HaMotor, right: HaMotor, slide: HaMotor, leftMotorAngleRadians: Double = DEFAULT_LEFT_MOTOR_ANGLE, rightMotorAngleRadians: Double = DEFAULT_RIGHT_MOTOR_ANGLE, slideMotorAngleRadians: Double = DEFAULT_SLIDE_MOTOR_ANGLE) :
            this(arrayOf(left, right, slide), leftMotorAngleRadians, rightMotorAngleRadians, slideMotorAngleRadians)

    override fun stop() = motors.forEach { it.stop() }

    fun driveFieldCentric(strafeSpeed: Double, forwardSpeed: Double, turn: Double, headingRadians: Double) {
        val strafe = clipRange(strafeSpeed)
        val forward = clipRange(forwardSpeed)
        val turnClipped = clipRange(turn)

        var vector = Vector2d(strafe, forward)
        vector = vector.rotateBy(-headingRadians)
        val theta = vector.angle()

        val speeds = DoubleArray(motors.size)

        if (speeds.size == 3) {
            val leftVec = Vector2d(cos(leftMotorAngleRadians), sin(leftMotorAngleRadians))
            val rightVec = Vector2d(cos(rightMotorAngleRadians), sin(rightMotorAngleRadians))
            val slideVec = Vector2d(cos(slideMotorAngleRadians), sin(slideMotorAngleRadians))

            speeds[MotorType.LEFT.value] = vector.scalarProject(leftVec) + turnClipped
            speeds[MotorType.RIGHT.value] = vector.scalarProject(rightVec) + turnClipped
            speeds[MotorType.SLIDE.value] = vector.scalarProject(slideVec) + turnClipped

            normalize(speeds)

            // Matches upstream SolversLib exactly -- the left/right slots are swapped here (not a
            // transcription error in this port).
            motors[MotorType.LEFT.value].percentOutput = (speeds[MotorType.RIGHT.value] * maxOutput).fraction
            motors[MotorType.RIGHT.value].percentOutput = (speeds[MotorType.LEFT.value] * maxOutput).fraction
            motors[MotorType.SLIDE.value].percentOutput = (speeds[MotorType.SLIDE.value] * maxOutput).fraction
        } else {
            speeds[MotorType.FRONT_LEFT.value] = sin(theta + Math.PI / 4)
            speeds[MotorType.FRONT_RIGHT.value] = sin(theta - Math.PI / 4)
            speeds[MotorType.BACK_LEFT.value] = sin(theta - Math.PI / 4)
            speeds[MotorType.BACK_RIGHT.value] = sin(theta + Math.PI / 4)

            normalize(speeds, vector.magnitude())

            speeds[MotorType.FRONT_LEFT.value] += turnClipped
            speeds[MotorType.FRONT_RIGHT.value] -= turnClipped
            speeds[MotorType.BACK_LEFT.value] += turnClipped
            speeds[MotorType.BACK_RIGHT.value] -= turnClipped

            motors[MotorType.FRONT_LEFT.value].percentOutput = (speeds[MotorType.FRONT_LEFT.value] * maxOutput).fraction
            motors[MotorType.FRONT_RIGHT.value].percentOutput = (speeds[MotorType.FRONT_RIGHT.value] * -maxOutput).fraction
            motors[MotorType.BACK_LEFT.value].percentOutput = (speeds[MotorType.BACK_LEFT.value] * maxOutput).fraction
            motors[MotorType.BACK_RIGHT.value].percentOutput = (speeds[MotorType.BACK_RIGHT.value] * -maxOutput).fraction
        }
    }

    fun driveRobotCentric(strafeSpeed: Double, forwardSpeed: Double, turn: Double) = driveFieldCentric(strafeSpeed, forwardSpeed, turn, 0.0)

    companion object {
        const val DEFAULT_RIGHT_MOTOR_ANGLE = Math.PI / 3
        const val DEFAULT_LEFT_MOTOR_ANGLE = 2 * Math.PI / 3
        const val DEFAULT_SLIDE_MOTOR_ANGLE = 3 * Math.PI / 2
    }
}
