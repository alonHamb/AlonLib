package com.qualcomm.robotcore.eventloop.opmode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.robocol.TelemetryMessage
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices
import org.firstinspires.ftc.teamcode.alonlib.emulator.EmuTelemetry
import java.util.concurrent.Executors

/**
 * Drives an [OpMode] (or [LinearOpMode], which is just an `OpMode` whose `internalRunOpMode()`
 * blocks on `runOpMode()` on a background thread until told to stop) through exactly the same
 * package-private lifecycle hooks that the real FTC SDK's `OpModeManagerImpl` uses: an
 * `internalInit()`-equivalent (see [init]), `internalStart()` (`start()`),
 * `internalOnEventLoopIteration()` (`loop()` plus fresh gamepad data), and `internalStop()`
 * (`stop()`, blocking until the OpMode thread exits). This class lives in `OpModeInternal`'s own
 * package purely to reach those hooks -- see `OpModeInternal.java` in the FTC SDK sources for what
 * each one does; nothing here re-implements OpMode lifecycle *semantics*, it only triggers the
 * SDK's own -- except for spawning the OpMode thread itself, see [init].
 */
class OpModeHarness(private val opMode: OpMode) {
    val telemetry = EmuTelemetry()

    private val services = object : OpModeServices {
        override fun refreshUserTelemetry(telemetry: TelemetryMessage?, sInterval: Double) {
            // EmuTelemetry captures data directly in addData()/update(); nothing to relay here.
        }

        override fun requestOpModeStop(opModeToStopIfActive: OpMode) {
            opModeToStopIfActive.stopRequested = true
        }
    }

    /**
     * Wires hardware/telemetry/gamepads and starts the OpMode thread running `init()`.
     *
     * This deliberately does NOT call the real `internalInit()`: it spawns its OpMode thread via
     * `ThreadPool.newSingleThreadExecutor`, whose thread-tracking bookkeeping touches
     * `android.util.LongSparseArray` -- a real Android collection class with no working
     * implementation on a desktop JVM, `returnDefaultValues` or not. This reimplements exactly
     * what `internalInit()` does (see `OpModeInternal.java`), using a plain
     * `Executors.newSingleThreadExecutor` instead.
     */
    fun init(hardwareMap: HardwareMap) {
        opMode.hardwareMap = hardwareMap
        opMode.gamepad1 = Gamepad()
        opMode.gamepad2 = Gamepad()
        opMode.telemetry = telemetry
        opMode.internalOpModeServices = services

        opMode.exception = null
        opMode.noClassDefFoundError = null
        opMode.isStarted = false
        opMode.stopRequested = false
        opMode.opModeThreadFinished = false

        opMode.gamepad1.resetEdgeDetection()
        opMode.gamepad2.resetEdgeDetection()
        opMode.gamepad1.setTriggerThreshold(Gamepad.DEFAULT_TRIGGER_THRESHOLD)
        opMode.gamepad2.setTriggerThreshold(Gamepad.DEFAULT_TRIGGER_THRESHOLD)

        val executor = Executors.newSingleThreadExecutor { runnable -> Thread(runnable, "EmulatedOpModeThread").apply { isDaemon = true } }
        opMode.executorService = executor
        executor.execute {
            try {
                opMode.internalRunOpMode()
            } catch (interrupted: InterruptedException) {
                opMode.requestOpModeStop()
            } catch (cancelled: java.util.concurrent.CancellationException) {
                opMode.requestOpModeStop()
            } catch (e: RuntimeException) {
                opMode.exception = e
            } catch (e: NoClassDefFoundError) {
                opMode.noClassDefFoundError = e
            } finally {
                opMode.opModeThreadFinished = true
            }
        }
    }

    /** Play button: transitions from Init to Run (`start()`, then `loop()` starts being called). */
    fun start() = opMode.internalStart()

    /** Call once per emulator tick: pushes fresh gamepad state and runs one event-loop iteration. */
    fun tick(gamepad1: Gamepad, gamepad2: Gamepad) {
        opMode.newGamepadDataAvailable(gamepad1, gamepad2)
        opMode.internalOnEventLoopIteration()
    }

    /** Stop button (or window close): requests a stop and blocks until the OpMode thread exits. */
    fun stop() = opMode.internalStop()

    /** An exception thrown by user code, if any -- surfaced by the emulator's crash panel. */
    val crash: Throwable? get() = opMode.exception ?: opMode.noClassDefFoundError
}
