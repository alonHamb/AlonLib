package org.firstinspires.ftc.teamcode.alonlib.drives.mecanumDrive

import com.qualcomm.robotcore.hardware.IMU
import com.seattlesolvers.solverslib.command.Subsystem
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor

class HaMecanumDrive(
    var frontLeftMotor: HaMotor,
    var frontRightMotor: HaMotor,
    var backLeftMotor: HaMotor,
    var backRightMotor: HaMotor,
    var gyro: IMU
                    ) : Subsystem {

    fun drive() {


    }

}
