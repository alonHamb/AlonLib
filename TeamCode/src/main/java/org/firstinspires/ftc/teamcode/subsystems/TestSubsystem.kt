package org.firstinspires.ftc.teamcode.subsystems

import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.servos.ServoEx

object TestSubsystem : SubsystemBase() {

    var testServo = ServoEx(map, "test servo")

    fun setServoAngle(angle: Double) {
        testServo.set(angle)
    }


}