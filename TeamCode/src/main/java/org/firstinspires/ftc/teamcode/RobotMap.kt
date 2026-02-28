package org.firstinspires.ftc.teamcode

import com.seattlesolvers.solverslib.hardware.motors.Motor

object RobotMap {

    /** here you set the hardware ids for your subsystems
     * each subsystem has her own object with its devices and their ids
     * an id decleration looks like this
     *  var DEVICE_NAME = "deviceId"
     */

    object Drive {
        const val PINPOINT_ID = "pinpoint"
        const val FRONT_LEFT_MOTOR_ID = "front left motor"
        const val FRONT_RIGHT_MOTOR_ID = "front right motor"
        const val BACK_LEFT_MOTOR_ID = "back left motor id"
        const val BACK_RIGHT_MOTOR_ID = "back right motor id"

        val DRIVE_MOTOR_TYPE = Motor.GoBILDA.RPM_435
    }

    object Shooter {
        const val FLYWHEEL_MOTOR_ID = "flywheel motor" // port 0 on the control hub
        val FLYWHEEL_MOTOR_TYPE = Motor.GoBILDA.BARE
        const val HEADING_MOTOR_ID = "heading motor"   // port 1 on the control hub
        val HEADING_MOTOR_TYPE = Motor.GoBILDA.RPM_435
        const val HOOD_SERVO_ID = "hood servo"    // servo port 0 on the control hub

    }

    object Transfer {
        const val TRANSFER_MOTOR_ID = "transfer motor"
        val TRANSFER_MOTOR_TYPE = Motor.GoBILDA.RPM_1150
    }

    object Intake {
        const val INTAKE_MOTOR_ID = "intake motor"
        val INTAKE_MOTOR_TYPE = Motor.GoBILDA.RPM_1150
    }

    object Vision {
        const val LIMELIGHT_ID = "limelight"
    }


}
