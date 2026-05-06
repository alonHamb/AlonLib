package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HEADING_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HEADING_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HOOD_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.TOP_FLYWHEEL_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.TOP_FLYWHEEL_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.div
import org.firstinspires.ftc.teamcode.alonlib.units.rpm
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.ANGLE_INTERPOLATION_TABLE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HEADING_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HEADING_RATIO
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HEADING_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HOOD_SERVO_RANGE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.MAXIMUM_HEADING
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.MINIMUM_HEADING
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_INTERPOLATION_TABLE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants as Constants

@Config
class ShooterSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {
    // --- hardware declaration and configuration ---
    val limelight = VisionSubsystem(hardwareMap, telemetry)
    val topFlywheelMotor = HaMotor(hardwareMap, TOP_FLYWHEEL_MOTOR_ID, TOP_FLYWHEEL_MOTOR_TYPE).apply {
        runMode = Motor.RunMode.VelocityControl
        zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        runningDirection = Motor.Direction.FORWARD
        tolerance = VELOCITY_TOLERANCE.asRpm
        pidfGains = VELOCITY_PID_GAINS
    }
    val bottomFlywheelMotor = HaMotor(hardwareMap, TOP_FLYWHEEL_MOTOR_ID, TOP_FLYWHEEL_MOTOR_TYPE).apply {
        runMode = topFlywheelMotor.runMode
        zeroPowerBehavior = topFlywheelMotor.zeroPowerBehavior
        runningDirection = topFlywheelMotor.runningDirection
        tolerance = topFlywheelMotor.tolerance
        pidfGains = topFlywheelMotor.pidfGains
    }
    val headingMotor = HaMotor(hardwareMap, HEADING_MOTOR_ID, HEADING_MOTOR_TYPE).apply {
        runMode = Motor.RunMode.PositionControl
        zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
        runningDirection = Motor.Direction.FORWARD
        tolerance = HEADING_TOLERANCE.degrees
        pidfGains = HEADING_PID_GAINS
    }
    val hoodServo = HaServo(hardwareMap, HOOD_SERVO_ID, HOOD_SERVO_RANGE).apply { // TODO add stock servo ranges to library
        runningDirection = Motor.Direction.FORWARD
    }
    // --- state getters and setters ---

    val currentAngle: Rotation2d
        get() = hoodServo.position
    var currentAngleSetPoint
        get() = hoodServo.position
        set(value) {
            hoodServo.position = value
        }
    val currentHeading: Rotation2d
        get() = (headingMotor.position.degrees * HEADING_RATIO).degrees

    var currentHeadingSetPoint: Rotation2d
        get() = headingMotor.position / HEADING_RATIO
        set(value) {
            headingMotor.setPoint = value.degrees * HEADING_RATIO
        }
    val currentVelocity: AngularVelocity
        get() = topFlywheelMotor.velocity

    var currentVelocitySetPoint: AngularVelocity
        get() = topFlywheelMotor.setPoint.rpm
        set(value) {
            topFlywheelMotor.velocity = value
        }
    val isAtMaxHeading get() = currentHeading >= MAXIMUM_HEADING
    val isAtMinHeading get() = currentHeading >= MINIMUM_HEADING
    val isInVelocityTolerance get() = topFlywheelMotor.inTolerance
    val isInHeadingTolerance get() = headingMotor.inTolerance
    val latestBotPosition get() = limelight.latestBotPose2d
    var state: Constants.ShooterState
        get() = Constants.ShooterState(currentAngle, currentHeading, currentVelocity)
        set(value) {
            currentAngleSetPoint = value.angle
            currentHeadingSetPoint = value.heading
            currentVelocitySetPoint = value.velocity
        }

    // --- operation functions ---

    fun stop() {
        stopShooterMotor()
        stopHeadingMotor()
        stopHoodServo()
    }

    fun stopShooterMotor() {
        topFlywheelMotor.stop()
    }

    fun stopHeadingMotor() {
        headingMotor.stop()
    }

    fun stopHoodServo() {
        hoodServo.servo.disable()
    }

    // --- dynamic shooting ---

    fun getDynamicShootingVelocityCalc(alliance: Alliance): AngularVelocity {
        return when (alliance) {
            Alliance.Red -> VELOCITY_INTERPOLATION_TABLE.getOutputFor(limelight.distanceToRedTarget.asMeters).rpm
            Alliance.Blue -> VELOCITY_INTERPOLATION_TABLE.getOutputFor(limelight.distanceToBlueTarget.asMeters).rpm
        }
    }

    fun getDynamicHeadingCalc(alliance: Alliance): Rotation2d {
        return when (alliance) {
            Alliance.Red -> state.heading + limelight.angleToRedTarget
            Alliance.Blue -> state.heading + limelight.angleToBlueTarget

        }


    }

    fun getDynamicHoodAngle(alliance: Alliance): Rotation2d {
        return when (alliance) {
            Alliance.Blue -> ANGLE_INTERPOLATION_TABLE.getOutputFor(limelight.distanceToBlueTarget.asMeters).degrees
            Alliance.Red -> ANGLE_INTERPOLATION_TABLE.getOutputFor(limelight.distanceToRedTarget.asMeters).degrees
        }
    }


    // --- Testing & Manual overrides ---

    fun setFlywheelMotorPower(power: PercentOutput) {
        topFlywheelMotor.percentOutput = power
    }

    fun increaseVelocitySetPointBy(velocity: AngularVelocity) {
        currentVelocitySetPoint += velocity
    }

    fun setHeadingMotorOutput(output: PercentOutput) {
        if (output > 0.0 && isAtMaxHeading || output < 0.0 && isAtMinHeading) {
            headingMotor.percentOutput = 0.0
        }
        headingMotor.percentOutput = output
    }

    fun increaseAngleSetpointBy(angle: Rotation2d) {
        currentAngleSetPoint += angle
    }

    // --- Telemetry ---

    fun addTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addLine("--- shooter subsystem ---")
                telemetry.addData("running command", super.currentCommand)
                telemetry.addData("current angle: ", currentAngle)
                telemetry.addData("current heading: ", currentHeading)
                telemetry.addData("heading setpoint: ", currentHeadingSetPoint)
                telemetry.addData("heading error: ", headingMotor.setPoint - headingMotor.position.degrees * HEADING_RATIO)
                telemetry.addData("current velocity: ", currentVelocity)
                telemetry.addData("current velocity setpoint: ", currentVelocitySetPoint)
                telemetry.addData("current velocity error: ", topFlywheelMotor.setPoint - topFlywheelMotor.velocity.asRpm)
                telemetry.addData("is At Max Heading: ", isAtMaxHeading)
                telemetry.addData("is at min heading: ", isAtMinHeading)
                telemetry.addData("is within velocity tolerance: ", isInVelocityTolerance)
                telemetry.addData("is within heading tolerance: ", isInHeadingTolerance)
            }
        }
    }

    // --- periodic hardware functions ---
    override fun periodic() {
        topFlywheelMotor.update()
        bottomFlywheelMotor.update()
        headingMotor.update()
    }


}
