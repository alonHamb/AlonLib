package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx

class HaCrServo(hardwareMap: HardwareMap, id: String) : CRServoEx(hardwareMap, id) {

    fun setEncoder(encoder: Encoder) {
        this.setEncoder(encoder)
    }

    fun setPIDGains(gains: PIDGains) {
    }
}