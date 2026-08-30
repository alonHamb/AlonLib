package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.CompassSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import emulator.hardware.SimI2cDevice
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.mockito.Mockito
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * A [NormalizedColorSensor]/[ColorSensor]/[DistanceSensor] all in one, backed by a single
 * [SimI2cDevice] -- matching a real REV Color Sensor V3, which
 * [org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaColorSensor] force-casts to both
 * of the latter two. [SimI2cDevice] has no color/distance physics of its own (see its doc
 * comment) -- drive it from your test/adapter code with [SimI2cDevice.setReading]: `"red"`/
 * `"green"`/`"blue"`/`"alpha"` (normalized, `[0,1)`, matching [NormalizedRGBA]'s own range) and
 * `"distanceMm"`.
 */
class EmuColorSensor(private val sim: SimI2cDevice) : NormalizedColorSensor, ColorSensor, DistanceSensor {

	private var gainField = 1f
	private var address = I2cAddr.zero()

	override fun getNormalizedColors(): NormalizedRGBA = NormalizedRGBA().apply {
		red = sim.getReading("red").toFloat()
		green = sim.getReading("green").toFloat()
		blue = sim.getReading("blue").toFloat()
		alpha = sim.getReading("alpha").toFloat()
	}

	override fun getGain(): Float = gainField
	override fun setGain(newGain: Float) {
		gainField = newGain
	}

	override fun red(): Int = (sim.getReading("red") * 255).toInt().coerceIn(0, 255)
	override fun green(): Int = (sim.getReading("green") * 255).toInt().coerceIn(0, 255)
	override fun blue(): Int = (sim.getReading("blue") * 255).toInt().coerceIn(0, 255)
	override fun alpha(): Int = (sim.getReading("alpha") * 255).toInt().coerceIn(0, 255)
	override fun argb(): Int = (alpha() shl 24) or (red() shl 16) or (green() shl 8) or blue()

	override fun enableLed(enable: Boolean) {}
	override fun setI2cAddress(newAddress: I2cAddr) {
		address = newAddress
	}

	override fun getI2cAddress(): I2cAddr = address

	override fun getDistance(unit: DistanceUnit): Double = unit.fromUnit(DistanceUnit.MM, sim.getReading("distanceMm"))

	override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
	override fun getDeviceName(): String = "EmuColorSensor"
	override fun getConnectionInfo(): String = sim.port.toString()
	override fun getVersion(): Int = 1
	override fun resetDeviceConfigurationForOpMode() {}
	override fun close() {}
}

/** A [CompassSensor] backed by one [SimI2cDevice]'s `"headingDeg"` reading, driven via [SimI2cDevice.setReading]. */
class EmuCompassSensor(private val sim: SimI2cDevice) : CompassSensor {

	private var mode = CompassSensor.CompassMode.MEASUREMENT_MODE

	override fun getDirection(): Double = ((sim.getReading("headingDeg") % 360.0) + 360.0) % 360.0
	override fun status(): String = "emulated"
	override fun setMode(mode: CompassSensor.CompassMode) {
		this.mode = mode
	}

	override fun calibrationFailed(): Boolean = false

	override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
	override fun getDeviceName(): String = "EmuCompassSensor"
	override fun getConnectionInfo(): String = sim.port.toString()
	override fun getVersion(): Int = 1
	override fun resetDeviceConfigurationForOpMode() {}
	override fun close() {}
}

/**
 * The register file of an emulated goBILDA® Pinpoint Odometry Computer, driven by one
 * [SimI2cDevice]'s `"xPositionMm"`/`"yPositionMm"`/`"headingRad"` (ground-truth pose, in the
 * device's own wire units) and `"xVelocityMmPerSec"`/`"yVelocityMmPerSec"`/
 * `"headingVelocityRadPerSec"` readings, driven via [SimI2cDevice.setReading] -- e.g. mirrored
 * from your simulated chassis's pose/velocity each tick, the same way [SimImu.headingRad] is
 * mirrored for [EmuImu]. Fault injection is supported via a `"deviceStatusFaultBits"` reading,
 * OR-ed onto the always-set READY bit and decoded by [GoBildaPinpointDriver.getDeviceStatus] the
 * same way real firmware bits are: `1 shl 1` CALIBRATING, `1 shl 2` X pod fault, `1 shl 3` Y pod
 * fault, `1 shl 4` IMU runaway, `1 shl 5` bad read; 0 (the default) reads as READY.
 *
 * Use [emulatedPinpointDriver] to turn one of these into a real [GoBildaPinpointDriver] to hand to
 * [org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaPinPoint] -- this class only encodes/
 * decodes that driver's I2C register protocol, it isn't itself an SDK device type.
 *
 * Two simplifications versus real hardware: encoder ticks are derived from the reported position
 * and the configured ticks-per-mm rather than modeled as independent raw pod counts (the real
 * device fuses ticks+IMU into position; this emulator only has the fused ground truth to work
 * from), and [GoBildaPinpointDriver.setBulkReadScope] isn't honored -- BULK_READ always reports
 * the device's own default ten-register scope, the only one
 * [org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaPinPoint] ever reads.
 */
class PinPointSensor(private val sim: SimI2cDevice) {

	private var xOffsetMm = 0.0
	private var yOffsetMm = 0.0
	private var headingOffsetRad = 0.0

	private var xEncoderDirection = 1
	private var yEncoderDirection = 1

	private var ticksPerMm = 13.26291192f
	private var xPodOffsetMm = 0f
	private var yPodOffsetMm = 0f
	private var yawScalar = 1f
	var loopTimeUs = 1000

	private val rawXMm get() = sim.getReading("xPositionMm")
	private val rawYMm get() = sim.getReading("yPositionMm")
	private val rawHeadingRad get() = sim.getReading("headingRad")

	private val xPositionMm get() = (rawXMm - xOffsetMm).toFloat()
	private val yPositionMm get() = (rawYMm - yOffsetMm).toFloat()
	private val headingRad get() = (rawHeadingRad - headingOffsetRad).toFloat()

	private val xVelocityMmPerSec get() = sim.getReading("xVelocityMmPerSec").toFloat()
	private val yVelocityMmPerSec get() = sim.getReading("yVelocityMmPerSec").toFloat()
	private val headingVelocityRadPerSec get() = sim.getReading("headingVelocityRadPerSec").toFloat()

	private val xEncoderTicks get() = (xPositionMm * ticksPerMm).toInt() * xEncoderDirection
	private val yEncoderTicks get() = (yPositionMm * ticksPerMm).toInt() * yEncoderDirection

	private val deviceStatusBits get() = READY_BIT or sim.getReading("deviceStatusFaultBits").toInt()

	/** Emulates a register read as [GoBildaPinpointDriver]'s deviceClient would perform it: [creg] bytes starting at [ireg]. */
	fun read(ireg: Int, creg: Int): ByteArray {
		val data = if (ireg == BULK_READ_REG) encodeBulkRead() else encodeRegister(ireg)
		return if (creg > data.size) data + crc8(data) else data
	}

	/** Emulates a register write as [GoBildaPinpointDriver]'s deviceClient would perform it. */
	fun write(ireg: Int, data: ByteArray) {
		when (ireg) {
			DEVICE_CONTROL_REG -> applyControl(intFrom(data))
			X_POSITION_REG -> xOffsetMm = rawXMm - floatFrom(data)
			Y_POSITION_REG -> yOffsetMm = rawYMm - floatFrom(data)
			H_ORIENTATION_REG -> headingOffsetRad = rawHeadingRad - floatFrom(data)
			MM_PER_TICK_REG -> ticksPerMm = floatFrom(data)
			X_POD_OFFSET_REG -> xPodOffsetMm = floatFrom(data)
			Y_POD_OFFSET_REG -> yPodOffsetMm = floatFrom(data)
			YAW_SCALAR_REG -> yawScalar = floatFrom(data)
			// SET_BULK_READ: not honored, see class doc comment.
		}
	}

	private fun applyControl(bits: Int) {
		// RECALIBRATE_IMU (bit 0) is treated as instantaneous, with no CALIBRATING window modeled.
		if (bits and (1 shl 1) != 0) { // RESET_POS_AND_IMU
			xOffsetMm = rawXMm
			yOffsetMm = rawYMm
			headingOffsetRad = rawHeadingRad
		}
		if (bits and (1 shl 5) != 0) xEncoderDirection = 1  // SET_X_ENCODER_FORWARD
		if (bits and (1 shl 4) != 0) xEncoderDirection = -1 // SET_X_ENCODER_REVERSED
		if (bits and (1 shl 3) != 0) yEncoderDirection = 1  // SET_Y_ENCODER_FORWARD
		if (bits and (1 shl 2) != 0) yEncoderDirection = -1 // SET_Y_ENCODER_REVERSED
	}

	private fun encodeRegister(ireg: Int): ByteArray = when (ireg) {
		DEVICE_ID_REG -> intBytes(1)
		DEVICE_VERSION_REG -> intBytes(DEVICE_VERSION)
		DEVICE_STATUS_REG -> intBytes(deviceStatusBits)
		LOOP_TIME_REG -> intBytes(loopTimeUs)
		X_ENCODER_VALUE_REG -> intBytes(xEncoderTicks)
		Y_ENCODER_VALUE_REG -> intBytes(yEncoderTicks)
		X_POSITION_REG -> floatBytes(xPositionMm)
		Y_POSITION_REG -> floatBytes(yPositionMm)
		H_ORIENTATION_REG -> floatBytes(headingRad)
		X_VELOCITY_REG -> floatBytes(xVelocityMmPerSec)
		Y_VELOCITY_REG -> floatBytes(yVelocityMmPerSec)
		H_VELOCITY_REG -> floatBytes(headingVelocityRadPerSec)
		X_POD_OFFSET_REG -> floatBytes(xPodOffsetMm)
		Y_POD_OFFSET_REG -> floatBytes(yPodOffsetMm)
		YAW_SCALAR_REG -> floatBytes(yawScalar)
		QUATERNION_W_REG -> floatBytes(kotlin.math.cos(headingRad / 2.0).toFloat())
		QUATERNION_X_REG -> floatBytes(0f)
		QUATERNION_Y_REG -> floatBytes(0f)
		QUATERNION_Z_REG -> floatBytes(kotlin.math.sin(headingRad / 2.0).toFloat())
		PITCH_REG -> floatBytes(0f)
		ROLL_REG -> floatBytes(0f)
		else -> intBytes(0)
	}

	private fun encodeBulkRead(): ByteArray = BULK_READ_ORDER.fold(ByteArray(0)) { acc, reg -> acc + encodeRegister(reg) }

	companion object {
		private const val DEVICE_VERSION = 3
		private const val READY_BIT = 1

		private const val DEVICE_ID_REG = 1
		private const val DEVICE_VERSION_REG = 2
		private const val DEVICE_STATUS_REG = 3
		private const val DEVICE_CONTROL_REG = 4
		private const val LOOP_TIME_REG = 5
		private const val X_ENCODER_VALUE_REG = 6
		private const val Y_ENCODER_VALUE_REG = 7
		private const val X_POSITION_REG = 8
		private const val Y_POSITION_REG = 9
		private const val H_ORIENTATION_REG = 10
		private const val X_VELOCITY_REG = 11
		private const val Y_VELOCITY_REG = 12
		private const val H_VELOCITY_REG = 13
		private const val MM_PER_TICK_REG = 14
		private const val X_POD_OFFSET_REG = 15
		private const val Y_POD_OFFSET_REG = 16
		private const val YAW_SCALAR_REG = 17
		private const val BULK_READ_REG = 18
		private const val QUATERNION_W_REG = 19
		private const val QUATERNION_X_REG = 20
		private const val QUATERNION_Y_REG = 21
		private const val QUATERNION_Z_REG = 22
		private const val PITCH_REG = 23
		private const val ROLL_REG = 24

		private val BULK_READ_ORDER = intArrayOf(
			DEVICE_STATUS_REG, LOOP_TIME_REG, X_ENCODER_VALUE_REG, Y_ENCODER_VALUE_REG,
			X_POSITION_REG, Y_POSITION_REG, H_ORIENTATION_REG, X_VELOCITY_REG, Y_VELOCITY_REG, H_VELOCITY_REG
		)

		private fun intBytes(value: Int): ByteArray = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(value).array()
		private fun floatBytes(value: Float): ByteArray = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value).array()
		private fun intFrom(bytes: ByteArray): Int = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).int
		private fun floatFrom(bytes: ByteArray): Float = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).float

		/** Replicates GoBildaPinpointDriver's private CRC8 (poly 0x31, init 0x90) so its CRC error-detection mode validates our reads. */
		private fun crc8(bytes: ByteArray): Byte {
			var crc = 0x90
			for (b in bytes) {
				crc = crc xor (b.toInt() and 0xFF)
				repeat(8) {
					crc = if (crc and 0x80 != 0) ((crc shl 1) xor 0x31) and 0xFF else (crc shl 1) and 0xFF
				}
			}
			return crc.toByte()
		}
	}
}

/**
 * Builds a real [GoBildaPinpointDriver] whose I2C register file is a [PinPointSensor] backed by
 * [sim] -- see that class's doc comment for the readings/limitations. [GoBildaPinpointDriver]
 * itself is constructed normally (its constructor takes any [com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple]),
 * but the driver casts its device client to [LynxI2cDeviceSynch] during initialization (to set the
 * I2C bus speed), so the device client has to actually be one -- there's no way to construct a real
 * one outside a physical REV hub, so, matching [emulatedLynxModule]'s approach to the analogous
 * problem with [com.qualcomm.hardware.lynx.LynxModule], it's mocked with Mockito instead. Every
 * unstubbed method is a safe no-op, which covers the rest of [LynxI2cDeviceSynch]'s surface that
 * [GoBildaPinpointDriver] touches.
 */
fun emulatedPinpointDriver(sim: SimI2cDevice): GoBildaPinpointDriver {
	val registers = PinPointSensor(sim)

	val deviceClient = Mockito.mock(LynxI2cDeviceSynch::class.java)
	Mockito.`when`(deviceClient.read(Mockito.anyInt(), Mockito.anyInt())).thenAnswer { invocation ->
		registers.read(invocation.getArgument(0), invocation.getArgument(1))
	}
	Mockito.doAnswer { invocation ->
		registers.write(invocation.getArgument(0), invocation.getArgument(1))
		null
	}.`when`(deviceClient).write(Mockito.anyInt(), Mockito.any(ByteArray::class.java))
	Mockito.`when`(deviceClient.connectionInfo).thenReturn(sim.port.toString())

	return GoBildaPinpointDriver(deviceClient, true)
}
