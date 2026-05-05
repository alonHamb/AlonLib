package org.firstinspires.ftc.teamcode.alonlib.sensors

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.alonlib.units.Length
import org.firstinspires.ftc.teamcode.alonlib.units.degPs
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.millimeters

class HaPinPoint(var hardwareMap: HardwareMap, id: String, var pod: GoBildaOdometryPods) :
    HardwareDevice {

    private var pinPoint: GoBildaPinpointDriver =
        hardwareMap.get(GoBildaPinpointDriver::class.java, id)

    private var ticksPerMm =
        when (pod) {
            GoBildaOdometryPods.goBILDA_SWINGARM_POD -> 13.262912f
            GoBildaOdometryPods.goBILDA_4_BAR_POD -> 19.894367f
        }

    init {
        pinPoint.setEncoderResolution(pod)
    }

    val deviceStatus get() = pinPoint.deviceStatus

    val loopTime get() = pinPoint.loopTime

    val frequency get() = pinPoint.frequency

    val encoderXTicks get() = pinPoint.encoderX

    val encoderYTicks get() = pinPoint.encoderY

    var position: Pose2d
        get() = Pose2d(
            Translation2d(pinPoint.position.getX(DistanceUnit.METER), pinPoint.position.getY(DistanceUnit.METER)),
            pinPoint.position.getHeading(AngleUnit.DEGREES).degrees
        )
        set(value) {
            pinPoint.setPosX(value.x, DistanceUnit.METER)
            pinPoint.setPosY(value.y, DistanceUnit.METER)
        }

    var positionX: Length
        get() = pinPoint.getPosX(DistanceUnit.MM).millimeters
        set(value) {
            pinPoint.setPosX(value.asMillimeters, DistanceUnit.MM)
        }

    var positionY: Length
        get() = pinPoint.getPosY(DistanceUnit.MM).millimeters
        set(value) {
            pinPoint.setPosY(value.asMillimeters, DistanceUnit.MM)
        }

    var heading: Rotation2d
        get() = pinPoint.getHeading(AngleUnit.DEGREES).degrees
        set(value) {
            pinPoint.setHeading(value.degrees, AngleUnit.DEGREES)
        }

    val countedHeading get() = pinPoint.getHeading(UnnormalizedAngleUnit.DEGREES).degrees

    /**
     * the velocity in the x-axis in units per second
     */
    val xVelocity get() = pinPoint.getVelX(DistanceUnit.METER).millimeters

    /**
     * the velocity in the y-axis in units per second
     */
    val yVelocity get() = pinPoint.getVelY(DistanceUnit.MM).millimeters

    val headingVelocity get() = pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES).degPs

    val xOffset get() = pinPoint.getXOffset(DistanceUnit.MM).millimeters

    val yOffset get() = pinPoint.getYOffset(DistanceUnit.MM).millimeters

    fun setEncoderDirections(
        xEncoderDirection: GoBildaPinpointDriver.EncoderDirection,
        yEncoderDirection: GoBildaPinpointDriver.EncoderDirection
    ) {
        pinPoint.setEncoderDirections(xEncoderDirection, yEncoderDirection)
    }

    /**
     * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    fun resetPoseAndIMU() {
        pinPoint.resetPosAndIMU()
    }

    /**
     * Recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    fun resetIMU() {
        pinPoint.recalibrateIMU()
    }

    /**
     * Sets the odometry pod positions relative to the point that the odometry computer tracks around.<br><br>
     * The most common tracking position is the center of the robot. <br> <br>
     * The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. <br>
     * the Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.<br>
     *
     * @param xOffset      how sideways from the center of the robot is the X (forward) pod? Left increases
     * @param yOffset      how far forward from the center of the robot is the Y (Strafe) pod? forward increases
     */
    fun setOffset(xOffset: Length, yOffset: Length) {
        pinPoint.setOffsets(xOffset.asMillimeters, yOffset.asMillimeters, DistanceUnit.MM)
    }

    fun update() {
        pinPoint.update()
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.GoBilda
    }

    override fun getDeviceName(): String {
        return pinPoint.deviceName
    }

    override fun getConnectionInfo(): String {
        return pinPoint.connectionInfo
    }

    override fun getVersion(): Int {
        return pinPoint.version
    }

    override fun resetDeviceConfigurationForOpMode() {
        pinPoint.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        pinPoint.close()
    }
}
