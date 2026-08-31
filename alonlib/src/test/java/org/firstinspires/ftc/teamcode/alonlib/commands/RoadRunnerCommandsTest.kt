package org.firstinspires.ftc.teamcode.alonlib.commands

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Test

/**
 * Doesn't exercise [ActionCommand.execute] -- it unconditionally calls
 * `FtcDashboard.getInstance()`, which (per the README's "Known limitations") reaches a real
 * Android `Context` and isn't available in a plain JVM unit test.
 */
class RoadRunnerCommandsTest {

	private class DummySubsystem : SubsystemBase()

	private class FakeAction(private val running: Boolean) : Action {

		override fun run(telemetryPacket: TelemetryPacket) = running
	}

	@Test
	fun `asCommand wraps the Action in an ActionCommand`() {
		val command = FakeAction(true).asCommand()
		assertEquals(ActionCommand::class.java, command.javaClass)
	}

	@Test
	fun `asCommand adds the given subsystems as requirements`() {
		val sub1 = DummySubsystem()
		val sub2 = DummySubsystem()

		val command = FakeAction(true).asCommand(sub1, sub2)

		assertEquals(setOf(sub1, sub2), command.requirement)
	}

	@Test
	fun `isFinished defaults to false before the command has run`() {
		val command = FakeAction(true).asCommand()
		assertFalse(command.isFinished())
	}

	@Test
	fun `initialize resets isFinished to false`() {
		val command = FakeAction(true).asCommand()
		command.initialize()
		assertFalse(command.isFinished())
	}
}
