package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.FLYWHEEL_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HEADING_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.HOOD_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Shooter.LIMELIGHT_ID
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.div
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HEADING_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HEADING_RATIO
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.HEADING_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.MAXIMUM_HEADING
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.MINIMUM_HEADING
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_PID_GAINS
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.VELOCITY_TOLERANCE
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants as Constants


class ShooterSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {

    // --- hardware decleration

    val limelight = hardwareMap.get(Limelight3A::class.java, LIMELIGHT_ID)
    val flywheelMotor = HaDcMotor(hardwareMap, FLYWHEEL_MOTOR_ID, Motor.GoBILDA.BARE).apply {
        setRunMode(Motor.RunMode.VelocityControl)
        velocityTolerance = VELOCITY_TOLERANCE
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
        PIDFGains = VELOCITY_PID_GAINS
    }

    val headingMotor = HaDcMotor(hardwareMap, HEADING_MOTOR_ID, Motor.GoBILDA.RPM_1150).apply {
        setRunMode(Motor.RunMode.PositionControl)
        positionTolerance = HEADING_TOLERANCE
        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        PIDFGains = HEADING_PID_GAINS
    }

    val hoodServo = HaServo(hardwareMap, HOOD_SERVO_ID).apply { runningDirection = HaServo.RunningDirection.FORWARD }


    // state getters and setters
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

    val isWithinVelocityTolerance get() = flywheelMotor.inTolerance

    val isWithinHeadingTolerance get() = headingMotor.inTolerance

    val angleToGoal
        get() {
            limelight.latestResult.botpose -

        }

    var shooterState: Constants.ShooterState
        get() = Constants.ShooterState(currentAngle, currentHeading, currentVelocity)
        set(value) {
            currentAngleSetPoint = value.angle
            currentHeadingSetPoint = value.heading
            currentVelocitySetPoint = value.velocity

        }

    fun setShooterState(state: Constants.ShooterState) {
        currentAngleSetPoint = state.angle
        currentHeadingSetPoint = state.heading
        currentVelocitySetPoint = state.velocity
    }

    fun stopShooterMotor() {
        flywheelMotor.disable()
    }

    fun stopHeadingMotor() {
        headingMotor.disable()
    }

    // Testing & Manual overrides

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
        telemetry.addData("current angle: ", currentAngle)
        telemetry.addData("current heading: ", currentHeading)
        telemetry.addData("heading setpoint: ", currentHeadingSetPoint)
        telemetry.addData("heading error: ", headingMotor.positionError * HEADING_RATIO)
        telemetry.addData("current velocity: ", currentVelocity)
        telemetry.addData("current velocity setpoint: ", currentVelocitySetPoint)
        telemetry.addData("current velocity error: ", flywheelMotor.velocityError)
        telemetry.addData("is At Max Heading: ", isAtMaxHeading)
        telemetry.addData("is at min heading: ", isAtMinHeading)
        telemetry.addData("is within velocity tolerance: ", isWithinVelocityTolerance)
        telemetry.addData("is within heading tolerance: ", isWithinHeadingTolerance)
    }

}