package org.firstinspires.ftc.teamcode.alonlib.emulator

import com.qualcomm.hardware.lynx.LynxModule
import emulator.hardware.SimMotor
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.mockito.Mockito
import org.mockito.invocation.InvocationOnMock

/**
 * Builds a [LynxModule] whose bulk-read data and input voltage come from simulated motors and a
 * simulated battery, instead of a real REV hub over USB.
 *
 * Unlike `DcMotorEx`/`Servo`, [LynxModule] is a concrete class with no way to be constructed
 * without a real `LynxUsbDevice`, and its `BulkData` nested class has a private constructor -- so
 * rather than implementing it, this mocks it with Mockito (already a dependency for testing
 * AlonLib itself). Every unstubbed method (`setBulkCachingMode`, `clearBulkCache`, ...) is a safe
 * no-op, which covers the rest of [LynxModule]'s surface that
 * [org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor] and typical OpMode code touch.
 *
 * [motorsByPort] maps REV hub motor port index (0-3) to the [SimMotor] plugged into that port --
 * see [EmulatedHub].
 */
fun emulatedLynxModule(motorsByPort: Map<Int, SimMotor>, batteryVoltage: () -> Double): LynxModule {
    val bulkData = Mockito.mock(LynxModule.BulkData::class.java)
    Mockito.`when`(bulkData.getMotorCurrentPosition(Mockito.anyInt())).thenAnswer { invocation: InvocationOnMock ->
        motorsByPort[invocation.getArgument(0)]?.getCurrentPosition() ?: 0
    }
    Mockito.`when`(bulkData.getMotorVelocity(Mockito.anyInt())).thenAnswer { invocation: InvocationOnMock ->
        motorsByPort[invocation.getArgument(0)]?.getVelocity()?.toInt() ?: 0
    }

    val hub = Mockito.mock(LynxModule::class.java)
    Mockito.`when`(hub.bulkData).thenReturn(bulkData)
    Mockito.`when`(hub.getInputVoltage(Mockito.any(VoltageUnit::class.java))).thenAnswer { invocation: InvocationOnMock ->
        val unit = invocation.getArgument<VoltageUnit>(0)
        unit.convert(batteryVoltage(), VoltageUnit.VOLTS)
    }
    return hub
}
