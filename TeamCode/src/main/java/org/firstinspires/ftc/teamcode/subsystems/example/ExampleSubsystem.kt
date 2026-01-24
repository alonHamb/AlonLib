package org.firstinspires.ftc.teamcode.subsystems.example

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.ExampleSubsystem.EXAMPLE_SERVO_ID
import org.firstinspires.ftc.teamcode.alonlib.motors.HaDcMotor
import org.firstinspires.ftc.teamcode.alonlib.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity

class ExampleSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    // --- hardware initialization and configuration ---
    /*
     *  each hardware device has its own value variable
     */
    var motor = HaDcMotor(hardwareMap, EXAMPLE_MOTOR_ID, Motor.GoBILDA.NONE)

    var servo = HaServo(hardwareMap, EXAMPLE_SERVO_ID)

    // --- state getters ---
    val stateComponent1 get() = servo.position

    val stateComponent2 get() = motor.velocity

    //  --- subsystemFunctions --
    fun setExamplePosition(position: Rotation2d) {
        motor.setPositionSetPoint(position)
    }

    // --- states class ---
    enum class ExampleSubsystemState(var stateComponent1: Rotation2d, var stateComponent2: AngularVelocity)

}