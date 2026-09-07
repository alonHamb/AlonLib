package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeHarness
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import emulator.config.SimulatedRobot
import emulator.hardware.SimDevice
import emulator.hardware.SimMotor
import emulator.input.GamepadSnapshot
import emulator.sim.BatteryModel
import emulator.sim.MecanumRobot
import emulator.sim.Pose
import emulator.ui.PortRowView
import emulator.ui.runRunnerShellAndBlock

/**
 * Ties a simulated robot -- either one or two [EmulatedHub]s you declared by hand, or a
 * [SimulatedRobot] built straight from your project's real hardware config XML (see
 * [EmulatorAutoLauncher], which is the zero-code way to get one of these) -- to
 * `emulator.ui.RunnerShellApp`, and drives whichever OpMode is selected through the exact same
 * lifecycle a real Driver Station would, via [OpModeHarness]. This is the one class most consumers
 * need to touch -- see the alonlib-emulator README for a full `src/test` example.
 */
class EmulatedRobot private constructor(private val parts: Parts) {
    /** Which four [SimMotor]s to feed into field-pose tracking; entirely optional. */
    data class DriveWheels(val frontLeft: SimMotor, val frontRight: SimMotor, val backLeft: SimMotor, val backRight: SimMotor)

    private class Parts(
        val hardwareMap: HardwareMap,
        val allDevices: List<SimDevice>,
        val battery: BatteryModel,
        val mecanum: MecanumRobot?
    )

    constructor(controlHub: EmulatedHub, expansionHub: EmulatedHub? = null, driveWheels: DriveWheels? = null) : this(
        buildParts(controlHub.devices + (expansionHub?.devices ?: emptyList()), driveWheels) { battery ->
            buildEmulatedHardwareMap(controlHub, expansionHub) { battery.voltage }
        }
    )

    /** Builds against every device [emulator.config.buildSimulatedRobot] resolved from a real hardware config XML file. */
    constructor(simulatedRobot: SimulatedRobot, driveWheels: DriveWheels? = null) : this(
        buildParts(simulatedRobot.allDevices, driveWheels) { battery ->
            buildEmulatedHardwareMap(simulatedRobot) { battery.voltage }
        }
    )

    /** The fake [HardwareMap] every emulated OpMode is given. */
    val hardwareMap: HardwareMap get() = parts.hardwareMap

    /**
     * Blocks the calling thread, showing the emulator window, until it's closed. [opModes] maps a
     * name shown in the OpMode dropdown to a factory for a fresh instance -- the real SDK
     * constructs a new OpMode instance on every Init too, so this mirrors that.
     */
    fun launch(title: String, opModes: Map<String, () -> OpMode>) {
        require(opModes.isNotEmpty()) { "launch() needs at least one OpMode" }
        val names = opModes.keys.toList()
        val allDevices = parts.allDevices
        val battery = parts.battery
        val mecanum = parts.mecanum
        var harness: OpModeHarness? = null

        runRunnerShellAndBlock(
            title = title,
            opModeNames = names,
            onInit = { index ->
                val fresh = OpModeHarness(opModes.getValue(names[index])())
                fresh.init(hardwareMap)
                harness = fresh
            },
            onStart = { harness?.start() },
            onStop = { harness?.stop() },
            onResetField = { mecanum?.resetPose(Pose(0.0, 0.0, 0.0)) },
            onTick = { dt, gamepads ->
                allDevices.forEach { it.update(dt) }
                battery.update(allDevices.sumOf { it.currentDrawAmps() })
                mecanum?.update(dt)
                harness?.tick(gamepads.gamepad1.toRealGamepad(), gamepads.gamepad2.toRealGamepad())
            },
            poseSupplier = { mecanum?.pose ?: Pose(0.0, 0.0, 0.0) },
            portRowsSupplier = { allDevices.map { it.toPortRowView() } },
            telemetrySupplier = { harness?.telemetry?.snapshot() ?: emptyList() },
            crashSupplier = { harness?.crash },
            statusSupplier = { if (harness == null) "STOPPED" else "RUNNING" },
            batteryVoltageSupplier = { battery.voltage }
        )
    }

    private companion object {
        private fun buildParts(devices: List<SimDevice>, driveWheels: DriveWheels?, buildMap: (BatteryModel) -> HardwareMap): Parts {
            val battery = BatteryModel()
            val mecanum = driveWheels?.let { MecanumRobot(it.frontLeft, it.frontRight, it.backLeft, it.backRight) }
            return Parts(buildMap(battery), devices, battery, mecanum)
        }
    }
}

private fun GamepadSnapshot.toRealGamepad(): Gamepad = Gamepad().apply {
    left_stick_x = leftStickX
    left_stick_y = leftStickY
    right_stick_x = rightStickX
    right_stick_y = rightStickY
    left_trigger = leftTrigger
    right_trigger = rightTrigger
    a = this@toRealGamepad.a
    b = this@toRealGamepad.b
    x = this@toRealGamepad.x
    y = this@toRealGamepad.y
    left_bumper = leftBumper
    right_bumper = rightBumper
    dpad_up = dpadUp
    dpad_down = dpadDown
    dpad_left = dpadLeft
    dpad_right = dpadRight
    start = this@toRealGamepad.start
    back = this@toRealGamepad.back
    options = this@toRealGamepad.options
}

private fun SimDevice.toPortRowView(): PortRowView =
    PortRowView(hub = port.hub.label, type = port.type.label, port = port.index, name = name, activitySummary = { activitySummary() })
