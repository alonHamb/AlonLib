package org.firstinspires.ftc.teamcode.subsystems.transfer

import org.firstinspires.ftc.teamcode.alonlib.hardware.Data

object TransferConstants {

    val TRANSFER_SERVO_MODE = Data.Servos.Mode.CR
    val TRANSFER_SERVO_TYPE = Data.Servos.Type.AxonMax

    const val TRANSFER_GEAR_RATIO = (60 / 15) * (50 / 19)

    const val INTAKE_POWER = 1.0
    const val SHOOTING_POWER = 1.0
    const val EJECT_POWER = -1.0

}
