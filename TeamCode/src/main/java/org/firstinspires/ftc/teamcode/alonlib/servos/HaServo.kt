package org.firstinspires.ftc.teamcode.alonlib.servos

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

class HaServo(
    hardwareMap: HardwareMap?,
    id: String,
    val minPosition: Rotation2d,
    val maxPosition: Rotation2d
) : HardwareDevice {
    constructor(hardwareMap: HardwareMap, id: String, range: Rotation2d) : this(
        hardwareMap,
        id,
        0.degrees,
        range
    )

    constructor(hardwareMap: HardwareMap, id: String) : this(
        hardwareMap,
        id,
        0.degrees,
        357.4.degrees
    )

    val servo = ServoEx(
        hardwareMap,
        id,
        MathUtils.normalizeDegrees(minPosition.degrees, true),
        MathUtils.normalizeDegrees(maxPosition.degrees, true).coerceIn(0.0, 359.999)
    ).apply { setPwm(PwmControl.PwmRange(500.0, 2500.0)) }


    var position: Rotation2d = 0.0.degrees
        set(position) {
            servo.set(
                MathUtils.normalizeDegrees(position.degrees, true)
            )
            field = position
        }

    var runningDirection: Motor.Direction
        get() {
            return when (servo.inverted) {
                true -> Motor.Direction.REVERSE
                false -> Motor.Direction.FORWARD
            }
        }
        set(runningDirection) {
            when (runningDirection) {
                Motor.Direction.FORWARD -> servo.setInverted(false)
                Motor.Direction.REVERSE -> servo.setInverted(true)
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
        servo.disable()
    }


}
