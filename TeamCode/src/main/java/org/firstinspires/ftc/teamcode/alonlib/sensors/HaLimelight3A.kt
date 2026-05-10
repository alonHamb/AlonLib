package org.firstinspires.ftc.teamcode.alonlib.sensors

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.LLStatus
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees

class HaLimelight3A(hardwareMap: HardwareMap, id: String) : HardwareDevice {
    private val limelight = hardwareMap.get(Limelight3A::class.java, id)

    val isPolling: Boolean
        get() = limelight.isRunning

    val currentlyConnected: Boolean
        get() = limelight.isConnected

    val status: LLStatus
        get() = limelight.status

    val latestResult: LLResult?
        get() = limelight.latestResult

    val detectedTags: List<LLResultTypes.FiducialResult>?
        get() = limelight.latestResult.fiducialResults

    val firstDetectedTag: LLResultTypes.FiducialResult? get() = limelight.latestResult.fiducialResults[0]

    val firstDetectedTagId: Int?
        get() = firstDetectedTag?.fiducialId

    val latestPose2d: Pose2d
        get() = Pose2d(
            Translation2d(latestResult?.botpose_MT2?.position?.x ?: 0.0, latestResult?.botpose_MT2?.position?.y ?: 0.0),
            latestResult?.botpose_MT2?.orientation?.yaw?.degrees ?: 0.0.degrees
                      )


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
        limelight.updateRobotOrientation(yaw.normalizedDegrees)
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
