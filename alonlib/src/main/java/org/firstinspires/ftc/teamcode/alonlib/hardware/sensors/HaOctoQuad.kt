package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import com.qualcomm.robotcore.hardware.HardwareMap

/** The DFRobot/Digital Chicken Labs OctoQuad: an 8-channel quadrature/pulse-width encoder reader. Covers channel reading/reset/direction/bank config -- see [octoQuad] directly for the localizer and pulse-width-specific features. */
class HaOctoQuad(val octoQuad: OctoQuad) : com.qualcomm.robotcore.hardware.HardwareDevice by octoQuad {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(OctoQuad::class.java, id))

    val firmwareVersion: String get() = octoQuad.firmwareVersionString

    fun setEncoderDirection(channel: Int, direction: OctoQuad.EncoderDirection) = octoQuad.setSingleEncoderDirection(channel, direction)
    fun getEncoderDirection(channel: Int): OctoQuad.EncoderDirection = octoQuad.getSingleEncoderDirection(channel)
    fun setAllEncoderDirections(reversed: BooleanArray) = octoQuad.setAllEncoderDirections(reversed)

    fun setChannelBankConfig(config: OctoQuad.ChannelBankConfig) = octoQuad.setChannelBankConfig(config)
    fun getChannelBankConfig(): OctoQuad.ChannelBankConfig = octoQuad.channelBankConfig

    /** Every channel's position/velocity in one I2C transaction -- prefer this over per-channel reads when polling all 8. */
    fun readAllEncoderData(): OctoQuad.EncoderDataBlock = octoQuad.readAllEncoderData()

    fun readPosition(channel: Int) = octoQuad.readSinglePosition(channel)
    fun readVelocity(channel: Int) = octoQuad.readSingleVelocity(channel)

    fun resetPosition(channel: Int) = octoQuad.resetSinglePosition(channel)
    fun resetAllPositions() = octoQuad.resetAllPositions()

    fun setCachingMode(mode: OctoQuad.CachingMode) = octoQuad.setCachingMode(mode)
    fun refreshCache() = octoQuad.refreshCache()

    fun saveParametersToFlash() = octoQuad.saveParametersToFlash()
    fun resetEverything() = octoQuad.resetEverything()
}
