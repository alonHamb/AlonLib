package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PIN_POINT_ID
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.DRIVE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PIN_POINT_X_OFFSET
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PIN_POINT_Y_OFFSET

class DriveSubsystem(val hardwareMap: HardwareMap, telemetry: Telemetry) : SubsystemBase() {
    // --- hardware decleration ---
    val frontLeftMotor = HaDcMotor(hardwareMap, FRONT_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val frontRightMotor = HaDcMotor(hardwareMap, FRONT_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val backLeftMotor = HaDcMotor(hardwareMap, BACK_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val backRightMotor = HaDcMotor(hardwareMap, BACK_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE)

    val localizer = HaPinPoint(hardwareMap, PIN_POINT_ID, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD).apply {
        setOffset(PIN_POINT_X_OFFSET, PIN_POINT_Y_OFFSET)
    }

    val drive = MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)

    fun driveFieldCentric(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        drive.driveFieldCentric(xSpeed, ySpeed, turnSpeed, localizer.heading.degrees)
    }

    fun driveRobotCentric(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        drive.driveRobotCentric(xSpeed, ySpeed, turnSpeed)
    }

    override fun periodic() {
        super.periodic()
        localizer.update()

    }

}