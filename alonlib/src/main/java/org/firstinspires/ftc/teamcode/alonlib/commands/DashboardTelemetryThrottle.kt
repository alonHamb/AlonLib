package org.firstinspires.ftc.teamcode.alonlib.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket

/**
 * Rate-limits how often telemetry packets actually get sent to FTC Dashboard.
 *
 * Building a [TelemetryPacket] (including field-overlay [com.acmerobotics.dashboard.canvas.Canvas]
 * drawing) is cheap -- it just appends to a list. The network send (JSON serialization + I/O) is
 * not, and calling it every loop is one of the most common causes of an FTC opmode's loop time
 * blowing up, especially with RoadRunner field-overlay drawing (pose-history polylines etc.)
 * growing every cycle. [send] only forwards a packet once every [minIntervalMs], dropping the
 * rest -- there's no point sending faster than a human (or the dashboard UI) can actually perceive.
 */
class DashboardTelemetryThrottle(private val minIntervalMs: Long = 50) {
    private var lastSentAt = 0L

    fun send(packet: TelemetryPacket) {
        val now = System.currentTimeMillis()
        if (now - lastSentAt >= minIntervalMs) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            lastSentAt = now
        }
    }
}
