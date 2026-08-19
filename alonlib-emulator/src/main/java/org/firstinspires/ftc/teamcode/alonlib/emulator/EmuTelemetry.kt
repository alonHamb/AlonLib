package org.firstinspires.ftc.teamcode.alonlib.emulator

import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * A [Telemetry] that renders into a plain list of strings via [snapshot], for
 * [emulator.ui.SnapshotTelemetryPanel]'s telemetry supplier, instead of transmitting to a driver
 * station.
 */
class EmuTelemetry : Telemetry {

    private sealed interface Renderable {
        fun render(): String
    }

    private inner class EmuItem(
        private var caption: String?,
        private var producer: () -> String,
        /** Mirrors the real SDK: Func-based items survive [clear], plain-value ones don't. */
        private val survivesClear: Boolean
    ) : Telemetry.Item, Renderable {
        var retainedOverride: Boolean? = null

        override fun getCaption(): String? = caption
        override fun setCaption(caption: String): Telemetry.Item = apply { this.caption = caption }
        override fun setValue(format: String, vararg args: Any?): Telemetry.Item = apply { producer = { String.format(format, *args) } }
        override fun setValue(value: Any?): Telemetry.Item = apply { producer = { value.toString() } }
        override fun <T> setValue(valueProducer: Func<T>): Telemetry.Item = apply { producer = { valueProducer.value().toString() } }
        override fun <T> setValue(format: String, valueProducer: Func<T>): Telemetry.Item = apply { producer = { String.format(format, valueProducer.value()) } }
        override fun setRetained(retained: Boolean?): Telemetry.Item = apply { retainedOverride = retained }
        override fun isRetained(): Boolean = retainedOverride ?: survivesClear

        override fun addData(caption: String, format: String, vararg args: Any?): Telemetry.Item =
            this@EmuTelemetry.addData(caption, format, *args)

        override fun addData(caption: String, value: Any?): Telemetry.Item = this@EmuTelemetry.addData(caption, value)
        override fun <T> addData(caption: String, valueProducer: Func<T>): Telemetry.Item = this@EmuTelemetry.addData(caption, valueProducer)
        override fun <T> addData(caption: String, format: String, valueProducer: Func<T>): Telemetry.Item =
            this@EmuTelemetry.addData(caption, format, valueProducer)

        override fun render(): String {
            val c = caption
            return if (c.isNullOrEmpty()) producer() else "$c$captionValueSeparator${producer()}"
        }
    }

    private inner class EmuLine(private val lineCaption: String? = null) : Telemetry.Line, Renderable {
        val itemsOnLine = mutableListOf<EmuItem>()

        private fun track(item: EmuItem): Telemetry.Item {
            itemsOnLine += item
            return item
        }

        override fun addData(caption: String, format: String, vararg args: Any?): Telemetry.Item =
            track(EmuItem(caption, { String.format(format, *args) }, false))

        override fun addData(caption: String, value: Any?): Telemetry.Item =
            track(EmuItem(caption, { value.toString() }, false))

        override fun <T> addData(caption: String, valueProducer: Func<T>): Telemetry.Item =
            track(EmuItem(caption, { valueProducer.value().toString() }, true))

        override fun <T> addData(caption: String, format: String, valueProducer: Func<T>): Telemetry.Item =
            track(EmuItem(caption, { String.format(format, valueProducer.value()) }, true))

        override fun render(): String {
            val body = itemsOnLine.joinToString(itemSeparator) { it.render() }
            return if (lineCaption != null) "$lineCaption$body" else body
        }
    }

    private inner class EmuLog : Telemetry.Log {
        override fun getCapacity(): Int = logCapacity
        override fun setCapacity(capacity: Int) {
            logCapacity = capacity
        }

        override fun getDisplayOrder(): Telemetry.Log.DisplayOrder = displayOrder
        override fun setDisplayOrder(displayOrder: Telemetry.Log.DisplayOrder) {
            this@EmuTelemetry.displayOrder = displayOrder
        }

        override fun add(entry: String) = pushLog(entry)
        override fun add(format: String, vararg args: Any?) = pushLog(String.format(format, *args))
        override fun clear() = logLines.clear()
    }

    private val renderables = mutableListOf<Renderable>()
    private val actions = mutableListOf<Runnable>()
    private val logLines = ArrayDeque<String>()
    private var logCapacity = 8
    private var displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
    private var autoClearFlag = true
    private var msInterval = 0
    private var itemSeparator = " | "
    private var captionValueSeparator = " : "
    private val logImpl = EmuLog()

    private fun pushLog(line: String) {
        logLines.addLast(line)
        while (logLines.size > logCapacity) logLines.removeFirst()
    }

    /** The rendered telemetry, one string per top-level item/line, for the emulator UI. */
    fun snapshot(): List<String> {
        val out = mutableListOf<String>()
        out += renderables.map { it.render() }
        out += if (displayOrder == Telemetry.Log.DisplayOrder.NEWEST_FIRST) logLines.reversed() else logLines
        return out
    }

    override fun addData(caption: String, format: String, vararg args: Any?): Telemetry.Item {
        val item = EmuItem(caption, { String.format(format, *args) }, false)
        renderables += item
        return item
    }

    override fun addData(caption: String, value: Any?): Telemetry.Item {
        val item = EmuItem(caption, { value.toString() }, false)
        renderables += item
        return item
    }

    override fun <T> addData(caption: String, valueProducer: Func<T>): Telemetry.Item {
        val item = EmuItem(caption, { valueProducer.value().toString() }, true)
        renderables += item
        return item
    }

    override fun <T> addData(caption: String, format: String, valueProducer: Func<T>): Telemetry.Item {
        val item = EmuItem(caption, { String.format(format, valueProducer.value()) }, true)
        renderables += item
        return item
    }

    override fun removeItem(item: Telemetry.Item): Boolean = (item as? Renderable)?.let { renderables.remove(it) } ?: false
    override fun removeLine(line: Telemetry.Line): Boolean = (line as? Renderable)?.let { renderables.remove(it) } ?: false

    override fun clear() {
        renderables.removeAll { it !is EmuItem || !it.isRetained() }
    }

    override fun clearAll() {
        renderables.clear()
        actions.clear()
    }

    override fun addAction(action: Runnable): Any {
        actions += action
        return action
    }

    override fun removeAction(token: Any): Boolean = actions.remove(token)

    override fun speak(text: String) {}
    override fun speak(text: String, languageCode: String, countryCode: String) {}

    override fun update(): Boolean {
        actions.forEach { it.run() }
        if (autoClearFlag) clear()
        return true
    }

    override fun addLine(): Telemetry.Line {
        val line = EmuLine()
        renderables += line
        return line
    }

    override fun addLine(lineCaption: String): Telemetry.Line {
        val line = EmuLine(lineCaption)
        renderables += line
        return line
    }

    override fun isAutoClear(): Boolean = autoClearFlag
    override fun setAutoClear(autoClear: Boolean) {
        autoClearFlag = autoClear
    }

    override fun getMsTransmissionInterval(): Int = msInterval
    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        msInterval = msTransmissionInterval
    }

    override fun getItemSeparator(): String = itemSeparator
    override fun setItemSeparator(itemSeparator: String) {
        this.itemSeparator = itemSeparator
    }

    override fun getCaptionValueSeparator(): String = captionValueSeparator
    override fun setCaptionValueSeparator(captionValueSeparator: String) {
        this.captionValueSeparator = captionValueSeparator
    }

    override fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat) {}
    override fun log(): Telemetry.Log = logImpl
}
