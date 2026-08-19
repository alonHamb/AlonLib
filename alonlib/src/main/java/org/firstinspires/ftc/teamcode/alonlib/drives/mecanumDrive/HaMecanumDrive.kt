package org.firstinspires.ftc.teamcode.alonlib.drives.mecanumDrive

import com.seattlesolvers.solverslib.drivebase.RobotDrive
import com.seattlesolvers.solverslib.geometry.Vector2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import kotlin.math.sin

class HaMecanumDrive(
    var motors: Array<Motor>,
    var rightSideMultiplier: Double = -1.0
                    ) : RobotDrive() {
    /**
     * Sets up the constructor for the mecanum drive.
     * Automatically inverts right side by default
     *
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    constructor(frontLeft: HaMotor, frontRight: HaMotor, backLeft: HaMotor, backRight: HaMotor) : this(
        arrayOf(
            frontLeft.motor,
            frontRight.motor,
            backLeft.motor,
            backRight.motor
               ), -1.0
                                                                                                      )


    /**
     * Checks if the right side motors are inverted.
     *
     * @return true if the multiplier for the right side is equal to -1.
     */
    fun isRightSideInverted(): Boolean {
        return rightSideMultiplier == -1.0
    }

    /**
     * Sets the right side inversion factor to the specified boolean.
     *
     * @param isInverted If true, sets the right side multiplier to -1 or 1 if false.
     */
    fun setRightSideInverted(isInverted: Boolean) {
        rightSideMultiplier = if (isInverted) -1.0 else 1.0
    }

    /**
     * Stop the motors.
     */
    override fun stop() {
        for (x in motors) {
            x.stopMotor()
        }
    }


    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     */
    fun driveRobotCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double) {
        driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0)
    }


    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param squareInputs Square joystick inputs for finer control
     */
    fun driveRobotCentric(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean) {
        var strafeSpeed = strafeSpeed
        var forwardSpeed = forwardSpeed
        var turnSpeed = turnSpeed
        strafeSpeed = if (squareInputs) clipRange(squareInput(strafeSpeed)) else clipRange(strafeSpeed)
        forwardSpeed = if (squareInputs) clipRange(squareInput(forwardSpeed)) else clipRange(forwardSpeed)
        turnSpeed = if (squareInputs) clipRange(squareInput(turnSpeed)) else clipRange(turnSpeed)

        driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed)
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     */
    fun driveFieldCentric(
        strafeSpeed: Double, forwardSpeed: Double,
        turnSpeed: Double, gyroAngle: Double
                         ) {
        var strafeSpeed = strafeSpeed
        var forwardSpeed = forwardSpeed
        var turnSpeed = turnSpeed
        strafeSpeed = clipRange(strafeSpeed)
        forwardSpeed = clipRange(forwardSpeed)
        turnSpeed = clipRange(turnSpeed)

        var input = Vector2d(strafeSpeed, forwardSpeed)
        input = input.rotateBy(-gyroAngle)

        val theta = input.angle()

        val wheelSpeeds = DoubleArray(4)
        wheelSpeeds[MotorType.kFrontLeft.value] = sin(theta + Math.PI / 4)
        wheelSpeeds[MotorType.kFrontRight.value] = sin(theta - Math.PI / 4)
        wheelSpeeds[MotorType.kBackLeft.value] = sin(theta - Math.PI / 4)
        wheelSpeeds[MotorType.kBackRight.value] = sin(theta + Math.PI / 4)

        normalize(wheelSpeeds, input.magnitude())

        wheelSpeeds[MotorType.kFrontLeft.value] += turnSpeed
        wheelSpeeds[MotorType.kFrontRight.value] -= turnSpeed
        wheelSpeeds[MotorType.kBackLeft.value] += turnSpeed
        wheelSpeeds[MotorType.kBackRight.value] -= turnSpeed

        normalize(wheelSpeeds)

        driveWithMotorPowers(
            wheelSpeeds[MotorType.kFrontLeft.value],
            wheelSpeeds[MotorType.kFrontRight.value],
            wheelSpeeds[MotorType.kBackLeft.value],
            wheelSpeeds[MotorType.kBackRight.value]
                            )
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param xSpeed       the horizontal speed of the robot, derived from input
     * @param ySpeed       the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     * @param squareInputs Square the value of the input to allow for finer control
     */
    fun driveFieldCentric(xSpeed: Double, ySpeed: Double, turnSpeed: Double, gyroAngle: Double, squareInputs: Boolean) {
        var xSpeed = xSpeed
        var ySpeed = ySpeed
        var turnSpeed = turnSpeed
        xSpeed = if (squareInputs) clipRange(squareInput(xSpeed)) else clipRange(xSpeed)
        ySpeed = if (squareInputs) clipRange(squareInput(ySpeed)) else clipRange(ySpeed)
        turnSpeed = if (squareInputs) clipRange(squareInput(turnSpeed)) else clipRange(turnSpeed)

        driveFieldCentric(xSpeed, ySpeed, turnSpeed, gyroAngle)
    }

    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param frontLeftSpeed    the speed of the front left motor
     * @param frontRightSpeed   the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed    the speed of the back right motor
     */
    fun driveWithMotorPowers(
        frontLeftSpeed: Double, frontRightSpeed: Double,
        backLeftSpeed: Double, backRightSpeed: Double
                            ) {

        motors[MotorType.kFrontLeft.value]
            .set(frontLeftSpeed * maxOutput)
        motors[MotorType.kFrontRight.value]
            .set(frontRightSpeed * rightSideMultiplier * maxOutput)
        motors[MotorType.kBackLeft.value]
            .set(backLeftSpeed * maxOutput)
        motors[MotorType.kBackRight.value]
            .set(backRightSpeed * rightSideMultiplier * maxOutput)
    }

}
