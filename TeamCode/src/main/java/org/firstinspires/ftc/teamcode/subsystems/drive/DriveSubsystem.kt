package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.DRIVE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PINPOINT_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_X_OFFSET
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_Y_OFFSET
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.Telemetry.Limelight
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.Telemetry.PinPoint
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.X_POD_DIRECTION
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.Y_POD_DIRECTION
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem

@Config
class DriveSubsystem(val hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {
    @JvmField
    // --- hardware declaration ---
    val frontLeftMotor = HaMotor(hardwareMap, FRONT_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val frontRightMotor = HaMotor(hardwareMap, FRONT_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val backLeftMotor = HaMotor(hardwareMap, BACK_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val backRightMotor = HaMotor(hardwareMap, BACK_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val pinPoint = HaPinPoint(hardwareMap, PINPOINT_ID, PINPOINT_ODOMETRY_PODS).apply {
        setOffset(
            PINPOINT_X_OFFSET,
            PINPOINT_Y_OFFSET
        )
        setEncoderDirections(
            X_POD_DIRECTION,
            Y_POD_DIRECTION
        )
    }

    // --- functional properties ---
    val limelight = VisionSubsystem(hardwareMap, telemetry)
    val drive = MecanumDrive(
        frontLeftMotor.motor,
        frontRightMotor.motor,
        backLeftMotor.motor,
        backRightMotor.motor
    )

    // --- state getters and setters ---
    var currentLocalizer = PinPoint

    // --- operation functions ---
    fun fieldCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        drive.driveFieldCentric(xSpeed, ySpeed, turnSpeed, pinPoint.heading.degrees)
    }

    fun robotCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        drive.driveRobotCentric(xSpeed, ySpeed, turnSpeed)
    }


    // --- all subsystem periodic functions ---
    fun updateLocalizer() {
        if (limelight.isInLimelightAccuracyRange) {
            pinPoint.position = limelight.latestBotPose2d
            currentLocalizer = Limelight
        } else {
            currentLocalizer = PinPoint
            pinPoint.update()
        }
    }

    override fun periodic() {
        super.periodic()
        updateLocalizer()

    }

    // --- Telemetry ---

    fun addTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addLine("--- Drive subsystem ---")
                telemetry.addData("Running Command", super.currentCommand)
                telemetry.addData("Robot pose", pinPoint.position.toString())
                telemetry.addData("Robot heading", pinPoint.heading)
                telemetry.addData("localizer", currentLocalizer)
            }
        }


    }


}
