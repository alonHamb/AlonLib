package org.firstinspires.ftc.teamcode.alonlib.sensors

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.units.degrees

class HaLimelight3A(hardwareMap: HardwareMap, id: String) : HardwareDevice {
    private val limelight = hardwareMap.get(Limelight3A::class.java, id)

    val isPolling get() = limelight.isRunning

    val currentlyConnected get() = limelight.isConnected

    val status get() = limelight.status

    val latestResult get() = limelight.latestResult

    val detectedTags get() = limelight.latestResult.fiducialResults

    val firstDetectedTag get() = limelight.latestResult.fiducialResults[0]

    val firstDetectedTagId get() = firstDetectedTag.fiducialId

    val latestPose2d get() = Pose2d(Translation2d(latestResult.botpose.position.x, latestResult.botpose.position.y), latestResult.botpose.orientation.yaw.degrees)


    var pipeLine = 1
        set(value) {
            field = value
            limelight.pipelineSwitch(value)
        }


    var pollRate
        get() = limelight.timeSinceLastUpdate
        set(Hz) {
            limelight.setPollRateHz(Hz.toInt())
        }

    fun reloadCurrentPipeline() {
        limelight.reloadPipeline()
    }

    fun startPolling() {
        limelight.start()
    }

    fun pausePolling() {
        limelight.pause()
    }

    fun stopPolling() {
        limelight.stop()
    }

    fun captureSnapshot(name: String) {
        limelight.captureSnapshot(name)
    }

    fun deleteAllSnapshots() {
        limelight.deleteSnapshots()
    }

    fun deleteSnapShot(name: String) {
        limelight.deleteSnapshot(name)
    }

    fun UpdateMegaTag2RobotHeading(yaw: Rotation2d) {
        limelight.updateRobotOrientation(yaw.degrees)
    }


    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        return limelight.manufacturer
    }

    override fun getDeviceName(): String {
        return limelight.deviceName
    }

    override fun getConnectionInfo(): String {
        return limelight.connectionInfo
    }

    override fun getVersion(): Int {
        return limelight.version
    }

    override fun resetDeviceConfigurationForOpMode() {
        limelight.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        limelight.close()
    }

}