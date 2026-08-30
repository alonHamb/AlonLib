package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.hardware.HardwareMap
import emulator.config.SimulatedRobot
import emulator.hardware.HubId
import emulator.hardware.PortId
import emulator.hardware.PortType
import emulator.hardware.SimAnalogDevice
import emulator.hardware.SimDevice
import emulator.hardware.SimDigitalDevice
import emulator.hardware.SimI2cDevice
import emulator.hardware.SimImu
import emulator.hardware.SimMotor
import emulator.hardware.SimServo

/**
 * One physical hub's worth of emulated devices, keyed by REV port index (or I2C bus, for [imus]/
 * [i2cDevices]) -- matching how you'd describe a real robot's wiring. Build one of these per hub
 * with the same device names your `RobotMap` (or equivalent) already uses, e.g.:
 *
 * ```
 * val controlHub = EmulatedHub(
 *     HubId.CONTROL,
 *     motors = mapOf(0 to RobotMap.Drive.FRONT_LEFT_MOTOR_ID, 1 to RobotMap.Drive.FRONT_RIGHT_MOTOR_ID),
 *     servos = mapOf(0 to RobotMap.Shooter.HOOD_SERVO_ID)
 * )
 * ```
 */
class EmulatedHub(
    hub: HubId,
    motors: Map<Int, String> = emptyMap(),
    servos: Map<Int, String> = emptyMap(),
    digitalDevices: Map<Int, String> = emptyMap(),
    analogDevices: Map<Int, String> = emptyMap(),
    imus: Map<Int, String> = emptyMap(),
    i2cDevices: Map<Int, String> = emptyMap(),
) {
    val motors: Map<Int, SimMotor> = motors.mapValues { (port, name) -> SimMotor(PortId(hub, PortType.MOTOR, port), name) }
    val servos: Map<Int, SimServo> = servos.mapValues { (port, name) -> SimServo(PortId(hub, PortType.SERVO, port), name) }
    val digitalDevices: Map<Int, SimDigitalDevice> =
        digitalDevices.mapValues { (port, name) -> SimDigitalDevice(PortId(hub, PortType.DIGITAL, port), name) }
    val analogDevices: Map<Int, SimAnalogDevice> =
        analogDevices.mapValues { (port, name) -> SimAnalogDevice(PortId(hub, PortType.ANALOG, port), name) }
    val imus: Map<Int, SimImu> = imus.mapValues { (bus, name) -> SimImu(PortId(hub, PortType.I2C, bus), name) }
    val i2cDevices: Map<Int, SimI2cDevice> = i2cDevices.mapValues { (bus, name) -> SimI2cDevice(PortId(hub, PortType.I2C, bus), name) }

    /** Every device on this hub, for dynamics ticking and the port monitor -- see [EmulatedRobot]. */
    val devices: List<SimDevice>
        get() = this.motors.values + this.servos.values + this.digitalDevices.values +
            this.analogDevices.values + this.imus.values + this.i2cDevices.values
}

/**
 * A real [HardwareMap] -- with a `null` app context and notifier, which is the SDK's own
 * documented way to build one "that won't be used by user code" outside the normal event loop --
 * except for [tryGet]/[get], which are reimplemented rather than inherited.
 *
 * [HardwareMap.tryGet] unconditionally calls `Device.isRevControlHub()` (to decide whether to
 * print a BNO055-vs-BHI260 IMU warning), whose static-init chain reaches
 * `System.loadLibrary("RobotCore")` -- a real Android-ARM `.so` with no desktop build, so merely
 * *calling* `hardwareMap.get(...)` crashes with an `UnsatisfiedLinkError` on a desktop JVM, no
 * matter what's actually registered in the map. Every other member ([put], [get] by name alone,
 * [getAll], [size], [iterator], ...) is left as the real, inherited implementation, since none of
 * them touch that code path.
 */
private class EmulatedHardwareMapImpl : HardwareMap(null, null) {
    override fun <T> tryGet(classOrInterface: Class<out T>, deviceName: String): T? {
        val list = allDevicesMap[deviceName.trim()] ?: return null
        for (device in list) {
            if (classOrInterface.isInstance(device)) return classOrInterface.cast(device)
        }
        return null
    }

    override fun <T> get(classOrInterface: Class<out T>, deviceName: String): T =
        tryGet(classOrInterface, deviceName)
            ?: throw IllegalArgumentException(
                "Unable to find a hardware device with name \"$deviceName\" and type ${classOrInterface.simpleName}"
            )
}

/**
 * Builds an [EmulatedHardwareMapImpl] pre-populated with [controlHub] and (if given)
 * [expansionHub]'s emulated devices, so `hardwareMap.get(DcMotorEx::class.java, "name")`,
 * `hardwareMap.get(Servo::class.java, "name")`, `hardwareMap.get(CRServo::class.java, "name")`,
 * `hardwareMap.get(TouchSensor::class.java, "name")` (and `DigitalChannel`, `AnalogInput`,
 * `OpticalDistanceSensor`, `IMU`, `ColorSensor`/`NormalizedColorSensor`/`DistanceSensor`,
 * `CompassSensor`, `GoBildaPinpointDriver`), and `hardwareMap.get(LynxModule::class.java, "Control
 * Hub")` all work exactly as they would against real hardware -- see [wireDevices].
 */
fun buildEmulatedHardwareMap(
    controlHub: EmulatedHub,
    expansionHub: EmulatedHub? = null,
    batteryVoltage: () -> Double
): HardwareMap {
    val hardwareMap = EmulatedHardwareMapImpl()

    fun wireHub(hub: EmulatedHub, hubName: String) {
        hub.motors.forEach { (_, sim) -> hardwareMap.put(sim.name, EmuDcMotorEx(sim)) }

        val servoController = EmuServoController(hub.servos)
        hub.servos.forEach { (port, sim) -> hardwareMap.put(sim.name, emulatedServo(servoController, port)) }

        wireDevices(hardwareMap, hub.servos.values, hub.digitalDevices.values, hub.analogDevices.values, hub.imus.values, hub.i2cDevices.values)

        hardwareMap.put(hubName, emulatedLynxModule(hub.motors, batteryVoltage))
    }

    wireHub(controlHub, "Control Hub")
    expansionHub?.let { wireHub(it, "Expansion Hub") }

    return hardwareMap
}

/**
 * Registers every non-motor/servo-position adapter this module ships (see [buildEmulatedHardwareMap]'s
 * doc comment for the full list) under each device's own name -- shared between the [EmulatedHub]-
 * and [SimulatedRobot]-driven overloads of `buildEmulatedHardwareMap`. Two adapters are registered
 * per device wherever the real config format can't tell which SDK interface an OpMode will actually
 * ask for (a `<Servo>`/`<CRServo>` tag, or an unrecognized I2C sensor tag) -- `hardwareMap.get`
 * simply returns whichever one matches the requested class; see [EmuCRServo], [EmuTouchSensor]/
 * [EmuDigitalChannel], [emulatedAnalogInput]/[EmuOpticalDistanceSensor], and [EmuColorSensor]/
 * [EmuCompassSensor]/[emulatedPinpointDriver].
 */
private fun wireDevices(
    hardwareMap: EmulatedHardwareMapImpl,
    servos: Collection<SimServo>,
    digitalDevices: Collection<SimDigitalDevice>,
    analogDevices: Collection<SimAnalogDevice>,
    imus: Collection<SimImu>,
    i2cDevices: Collection<SimI2cDevice>,
) {
    servos.forEach { hardwareMap.put(it.name, EmuCRServo(it)) }
    digitalDevices.forEach { sim ->
        hardwareMap.put(sim.name, EmuTouchSensor(sim))
        hardwareMap.put(sim.name, EmuDigitalChannel(sim))
    }
    analogDevices.forEach { sim ->
        hardwareMap.put(sim.name, emulatedAnalogInput(sim))
        hardwareMap.put(sim.name, EmuOpticalDistanceSensor(sim))
    }
    imus.forEach { hardwareMap.put(it.name, EmuImu(it)) }
    i2cDevices.forEach { sim ->
        hardwareMap.put(sim.name, EmuColorSensor(sim))
        hardwareMap.put(sim.name, EmuCompassSensor(sim))
        hardwareMap.put(sim.name, emulatedPinpointDriver(sim))
    }
}

/**
 * Builds an [EmulatedHardwareMapImpl] pre-populated straight from [simulatedRobot] -- everything
 * [emulator.config.buildSimulatedRobot] resolved from a real hardware config XML file -- the same
 * way [buildEmulatedHardwareMap] above does for hand-declared [EmulatedHub]s, including every
 * device type listed in that overload's doc comment (motors, servos/CRServos, digital/analog/IMU/
 * I2C devices via [wireDevices]). [HubId.CONTROL] is always wired up (a real robot always has
 * one), [HubId.EXPANSION] only if [simulatedRobot] actually has a device on it.
 *
 * [SimulatedRobot]'s digital/analog/IMU/I2C maps aren't split by hub the way motors/servos are
 * (the config format doesn't need to distinguish where an I2C device's bus lives from which hub
 * it's wired to for `hardwareMap.get` to work), so they're wired once, outside the per-hub loop.
 */
fun buildEmulatedHardwareMap(simulatedRobot: SimulatedRobot, batteryVoltage: () -> Double): HardwareMap {
    val hardwareMap = EmulatedHardwareMapImpl()
    val hubsPresent = simulatedRobot.allDevices.map { it.port.hub }.toSet() + HubId.CONTROL

    for (hub in hubsPresent) {
        val motors = simulatedRobot.motors.values.filter { it.port.hub == hub }
        val servos = simulatedRobot.servos.values.filter { it.port.hub == hub }

        motors.forEach { hardwareMap.put(it.name, EmuDcMotorEx(it)) }

        val servoController = EmuServoController(servos.associateBy { it.port.index })
        servos.forEach { hardwareMap.put(it.name, emulatedServo(servoController, it.port.index)) }

        hardwareMap.put(hub.label, emulatedLynxModule(motors.associateBy { it.port.index }, batteryVoltage))
    }

    wireDevices(
        hardwareMap,
        simulatedRobot.servos.values,
        simulatedRobot.digitalDevices.values,
        simulatedRobot.analogDevices.values,
        simulatedRobot.imus.values,
        simulatedRobot.i2cDevices.values
    )

    return hardwareMap
}
