package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Mode
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data.Servos.Type
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

class HaServo(
    hardwareMap: HardwareMap,
    id: String,
    val mode: Mode,
    val type: Type
) : HardwareDevice {
    val servo: Servo = hardwareMap.get(Servo::class.java, id)


    var position: Rotation2d = 0.0.degrees
        set(position) {
            when (mode) {
                Mode.CR -> robotPrintError("cannot set position in CR mode")
                Mode.FULL_RANGE -> servo.position = MathUtils.normalizeDegrees(
                    (position.degrees / (type.range.degrees)),
                    true
                ) / 360.0
            }
            field = position
        }

    var velocity: AngularVelocity = 0.0.rpm
        set(value) {
            when (mode) {
                Mode.CR -> servo.position = (value.asRpm / type.maxSpeed.asRpm)
                Mode.FULL_RANGE -> robotPrintError("cannot set velocity in CR mode")

            }
            field = value
        }

    var runningDirection: Motor.Direction
        get() {
            return when (servo.direction) {
                Servo.Direction.FORWARD -> Motor.Direction.REVERSE
                Servo.Direction.REVERSE -> Motor.Direction.FORWARD
            }
        }
        set(runningDirection) {
            when (runningDirection) {
                Motor.Direction.FORWARD -> servo.direction = Servo.Direction.FORWARD
                Motor.Direction.REVERSE -> servo.direction = Servo.Direction.REVERSE
            }
        }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String {
        return "HaServo"
    }

    override fun getConnectionInfo(): String {
        return ""
    }

    override fun getVersion(): Int {
        return 1
    }

    override fun resetDeviceConfigurationForOpMode() {
    }

    override fun close() {
        servo.close()
    }


}
