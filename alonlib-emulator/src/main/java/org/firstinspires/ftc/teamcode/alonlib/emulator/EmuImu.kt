package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.IMU
import emulator.hardware.SimImu
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import kotlin.math.cos
import kotlin.math.sin

/**
 * An [IMU] backed by a [SimImu]'s [SimImu.headingRad] -- e.g. mirrored from
 * [emulator.sim.MecanumRobot.pose]'s heading in your `onTick`, the same way a real IMU tracks the
 * simulated chassis. Only yaw is modeled (matching [SimImu]'s own shape): pitch/roll always read
 * zero, and [getRobotAngularVelocity] always reads zero -- there's no simulated angular rate to
 * report, only a heading value your test code sets directly.
 */
class EmuImu(private val sim: SimImu) : IMU {
    private var yawOffsetRad = 0.0

    private val yawRad get() = sim.headingRad - yawOffsetRad

    override fun initialize(parameters: IMU.Parameters): Boolean = true

    override fun resetYaw() {
        yawOffsetRad = sim.headingRad
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles =
        YawPitchRollAngles(AngleUnit.RADIANS, yawRad, 0.0, 0.0, System.nanoTime())

    override fun getRobotOrientation(reference: AxesReference, order: AxesOrder, angleUnit: AngleUnit): Orientation =
        Orientation(reference, order, angleUnit, angleUnit.fromUnit(AngleUnit.RADIANS, yawRad).toFloat(), 0f, 0f, System.nanoTime())

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        val halfYaw = yawRad / 2.0
        return Quaternion(cos(halfYaw).toFloat(), 0f, 0f, sin(halfYaw).toFloat(), System.nanoTime())
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity =
        AngularVelocity(angleUnit, 0f, 0f, 0f, System.nanoTime())

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "EmuImu"
    override fun getConnectionInfo(): String = sim.port.toString()
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {
        yawOffsetRad = 0.0
    }

    override fun close() {}
}
