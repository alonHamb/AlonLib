package org.firstinspires.ftc.teamcode.subsystems.example

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_CR_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_SERVO_ID
import org.firstinspires.ftc.teamcode.alonlib.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaCrServo
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo

class ExampleSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    // --- hardware initialization and configuration ---
    /*
        here you declare all the hardware used by your subsystem and configure them
        each hardware device has its own value
     */
    val motor = HaMotor(hardwareMap, EXAMPLE_MOTOR_ID, Motor.GoBILDA.NONE)

    val servo = HaServo(hardwareMap, EXAMPLE_SERVO_ID)

    val crServo = HaCrServo(hardwareMap, EXAMPLE_CR_SERVO_ID)

    // --- state getters ---
    /*
      here you create a properties for any state that you want eiter to receive in other areas of the code or to let other areas of the code set and modify
     */
    val stateComponent1 get() = servo.position

    val stateComponent2 get() = motor.velocity


    //  --- subsystemFunctions --

    /*
    here you can put any function your subsystem might need but beware! most things can be getters and setters and don't require a function
     */
    fun setExamplePosition(position: Rotation2d) {
        motor.position = position
    }


    override fun periodic() {
        /*
        here you set all the actions you want to run every run loop
         */
    }

}
