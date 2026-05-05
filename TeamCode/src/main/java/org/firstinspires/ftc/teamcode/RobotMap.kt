package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.alonlib.units.degrees

object RobotMap {

    /** here you set the hardware ids for your subsystems
     * each subsystem has her own object with its devices and their ids
     * an id declaration looks like this
     *  val DEVICE_NAME = "deviceId"
     */

    object ExampleSubsystem {
        const val EXAMPLE_MOTOR_ID = "motor 3"
        const val EXAMPLE_SERVO_ID = "servo 0"
        val EXAMPLE_SERVO_RANGE = 355.degrees
        const val EXAMPLE_CR_SERVO_ID = "servo 1"
    }

}
