package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.FLYWHEEL_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.FLYWHEEL_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HEADING_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HEADING_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HOOD_SERVO_ID
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
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.MAXIMUM_HEADING
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.MINIMUM_HEADING
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_INTERPOLATION_TABLE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants as Constants

@Config
class ShooterSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    @JvmField

    // --- hardware decleration and configuration ---
    val limelight = VisionSubsystem(hardwareMap, telemetry)
    val flywheelMotor = HaMotor(hardwareMap, FLYWHEEL_MOTOR_ID, FLYWHEEL_MOTOR_TYPE).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
        runningDirection = Motor.Direction.FORWARD
        velocityTolerance = VELOCITY_TOLERANCE
        PIDFGains = VELOCITY_PID_GAINS
    }
    val headingMotor = HaMotor(hardwareMap, HEADING_MOTOR_ID, HEADING_MOTOR_TYPE).apply {
        setRunMode(Motor.RunMode.PositionControl)
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        runningDirection = Motor.Direction.FORWARD
        positionTolerance = HEADING_TOLERANCE
        PIDFGains = HEADING_PID_GAINS
    }
    val hoodServo = HaServo(hardwareMap, HOOD_SERVO_ID).apply { runningDirection = HaServo.RunningDirection.FORWARD }
    // --- state getters and setters ---

    val currentAngle: Rotation2d
        get() = hoodServo.position
    var currentAngleSetPoint
        get() = hoodServo.position
        set(value) {
            hoodServo.position = value
        }
    val currentHeading: Rotation2d
        get() = (headingMotor.currentPosition * HEADING_RATIO).degrees
    var currentHeadingSetPoint: Rotation2d
        get() = headingMotor.position / HEADING_RATIO
        set(value) {
            headingMotor.positionSetpoint = value * HEADING_RATIO
        }
    val currentVelocity: AngularVelocity
        get() = flywheelMotor.velocity

    var currentVelocitySetPoint
        get() = flywheelMotor.velocitySetpoint
        set(value) {
            flywheelMotor.velocity = value
        }
    val isAtMaxHeading get() = currentHeading >= MAXIMUM_HEADING
    val isAtMinHeading get() = currentHeading >= MINIMUM_HEADING
    val isInVelocityTolerance get() = flywheelMotor.inTolerance
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
        flywheelMotor.disable()
    }

    fun stopHeadingMotor() {
        headingMotor.disable()
    }

    fun stopHoodServo() {
        hoodServo.disable()
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
        flywheelMotor.precentOutput = power
    }

    fun increaseVelocitySetPointBy(velocity: AngularVelocity) {
        currentVelocitySetPoint += velocity
    }

    fun setHeadingMotorOutput(output: PercentOutput) {
        if (output > 0.0 && isAtMaxHeading) {
            headingMotor.precentOutput = 0.0
        }
        if (output < 0.0 && isAtMinHeading) {
            headingMotor.precentOutput = 0.0
        }
        headingMotor.precentOutput = output
    }

    fun increaseAngleSetpointBy(angle: Rotation2d) {
        currentAngleSetPoint += angle
    }

    // --- Telemetry ---

    fun addTelemetry() {
        telemetry.addLine("--- shooter subsystem ---")
        telemetry.addData("running command", super.currentCommand)
        telemetry.addData("current angle: ", currentAngle)
        telemetry.addData("current heading: ", currentHeading)
        telemetry.addData("heading setpoint: ", currentHeadingSetPoint)
        telemetry.addData("heading error: ", headingMotor.positionError * HEADING_RATIO)
        telemetry.addData("current velocity: ", currentVelocity)
        telemetry.addData("current velocity setpoint: ", currentVelocitySetPoint)
        telemetry.addData("current velocity error: ", flywheelMotor.velocityError)
        telemetry.addData("is At Max Heading: ", isAtMaxHeading)
        telemetry.addData("is at min heading: ", isAtMinHeading)
        telemetry.addData("is within velocity tolerance: ", isInVelocityTolerance)
        telemetry.addData("is within heading tolerance: ", isInHeadingTolerance)
    }

    // --- periodic subsystem functions ---
    override fun periodic() {
        super.periodic()
        flywheelMotor.calculatePid()
        headingMotor.calculatePid()
    }


}