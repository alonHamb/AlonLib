package org.firstinspires.ftc.teamcode.alonlib.commands

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class ExtentionsTest {

    private fun noopCommand() = InstantCommand({})
    private class DummySubsystem : SubsystemBase()

    /** Never finishes on its own, so `until`'s race is decided purely by the given condition. */
    private class NeverEndingCommand : CommandBase()

    @Test
    fun `until ends the race once the condition becomes true`() {
        val command = NeverEndingCommand() until { true }
        command.initialize()
        command.execute()
        assertTrue(command.isFinished())
    }

    @Test
    fun `until does not end the command when the condition is false`() {
        val command = NeverEndingCommand() until { false }
        command.initialize()
        command.execute()
        assertTrue(!command.isFinished())
    }

    @Test
    fun `andThen with a Command unions the requirements of both commands`() {
        val firstSub = DummySubsystem()
        val secondSub = DummySubsystem()
        val first = InstantCommand({}, firstSub)
        val second = InstantCommand({}, secondSub)

        val chained = first andThen second

        assertTrue(firstSub in chained.requirements)
        assertTrue(secondSub in chained.requirements)
    }

    @Test
    fun `andThen with a supplier invokes it eagerly and chains the result`() {
        var supplierCalled = false
        val secondSub = DummySubsystem()

        val chained = noopCommand() andThen {
            supplierCalled = true
            InstantCommand({}, secondSub)
        }

        assertTrue("the supplier is invoked immediately to build the chain, not deferred", supplierCalled)
        assertTrue(secondSub in chained.requirements)
    }

    @Test
    fun `finallyDo with a callback runs it when the command ends`() {
        var interruptedFlag: Boolean? = null
        val command = noopCommand().finallyDo { interrupted -> interruptedFlag = interrupted }

        command.initialize()
        command.end(false)

        assertEquals(false, interruptedFlag)
    }

    @Test
    fun `withTimeout wraps the command, preserving its requirements`() {
        val sub = DummySubsystem()
        val command = InstantCommand({}, sub) withTimeout 5.0
        assertTrue(sub in command.requirements)
    }

    @Test
    fun `alongWith and raceWith union the requirements of both commands`() {
        val firstSub = DummySubsystem()
        val secondSub = DummySubsystem()

        val along = InstantCommand({}, firstSub) alongWith InstantCommand({}, secondSub)
        assertTrue(firstSub in along.requirements)
        assertTrue(secondSub in along.requirements)

        val raced = InstantCommand({}, firstSub) raceWith InstantCommand({}, secondSub)
        assertTrue(firstSub in raced.requirements)
        assertTrue(secondSub in raced.requirements)
    }

    @Test
    fun `withName on a Command sets its name`() {
        val command = noopCommand() withName "my command"
        assertEquals("my command", command.name)
    }

    @Test
    fun `top-level withName builds a command via the supplier and names it`() {
        val command = withName("built command") { InstantCommand({}) }
        assertEquals("built command", command.name)
    }

    @Test
    fun `SubsystemBase withName appends the subsystem's name`() {
        val subsystem = DummySubsystem()
        val command = subsystem.withName("my command") { InstantCommand({}) }

        assertEquals("my command : ${subsystem.name}", command.name)
    }
}
