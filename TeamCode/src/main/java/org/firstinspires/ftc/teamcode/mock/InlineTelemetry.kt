package org.firstinspires.ftc.teamcode.mock

import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry

const val TELEMETRY_IS_ENABLED = false
val DUMMY_LOG = object: Telemetry.Log {
    override fun getCapacity(): Int = 0

    override fun setCapacity(capacity: Int) {
    }

    override fun getDisplayOrder() = Telemetry.Log.DisplayOrder.OLDEST_FIRST

    override fun setDisplayOrder(displayOrder: Telemetry.Log.DisplayOrder?) {
    }

    override fun add(entry: String?) {
    }

    override fun add(format: String?, vararg args: Any?) {
    }

    override fun clear() {
    }
}


@Suppress("NOTHING_TO_INLINE", "OVERRIDE_BY_INLINE", "KotlinConstantConditions")
class InlineTelemetry(val around: Telemetry) : Telemetry {
    override inline fun addData(
        caption: String?,
        format: String?,
        vararg args: Any?
    ): Telemetry.Item? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addData(caption, format, args)
    }

    override inline fun addData(
        caption: String?,
        value: Any?
    ): Telemetry.Item? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addData(caption, value)
    }

    override inline fun <T : Any?> addData(
        caption: String?,
        valueProducer: Func<T?>?
    ): Telemetry.Item? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addData(caption, valueProducer)
    }

    override inline fun <T : Any?> addData(
        caption: String?,
        format: String?,
        valueProducer: Func<T?>?
    ): Telemetry.Item? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addData(caption, format, valueProducer)
    }

    override inline fun removeItem(item: Telemetry.Item?): Boolean {
        if (!TELEMETRY_IS_ENABLED) return false
        return around.removeItem(item)
    }

    override inline fun clear() {
        if (!TELEMETRY_IS_ENABLED) return
        around.clear()
    }

    override inline fun clearAll() {
        if (!TELEMETRY_IS_ENABLED) return
        around.clearAll()
    }

    override inline fun addAction(action: Runnable?): Any? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addAction(action)
    }

    override inline fun removeAction(token: Any?): Boolean {
        if (!TELEMETRY_IS_ENABLED) return false
        return around.removeAction(token)
    }

    override inline fun speak(text: String?) {
        if (!TELEMETRY_IS_ENABLED) return
        return around.speak(text)
    }

    override inline fun speak(
        text: String?,
        languageCode: String?,
        countryCode: String?
    ) {
        if (!TELEMETRY_IS_ENABLED) return
        return around.speak(text, languageCode, countryCode)
    }

    override inline fun update(): Boolean {
        if (!TELEMETRY_IS_ENABLED) return false
        return around.update()
    }

    override inline fun addLine(): Telemetry.Line? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addLine()
    }

    override inline fun addLine(lineCaption: String?): Telemetry.Line? {
        if (!TELEMETRY_IS_ENABLED) return null
        return around.addLine(lineCaption)
    }

    override inline fun removeLine(line: Telemetry.Line?): Boolean {
        if (!TELEMETRY_IS_ENABLED) return false
        return around.removeLine(line)
    }

    override inline fun isAutoClear(): Boolean {
        if (!TELEMETRY_IS_ENABLED) return true
        return around.isAutoClear
    }

    override inline fun setAutoClear(autoClear: Boolean) {
        if (!TELEMETRY_IS_ENABLED) return
        around.isAutoClear = autoClear
    }

    override inline fun getMsTransmissionInterval(): Int {
        if (!TELEMETRY_IS_ENABLED) return 0
        return around.msTransmissionInterval
    }

    override inline fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        if (!TELEMETRY_IS_ENABLED) return
        around.msTransmissionInterval = msTransmissionInterval
    }

    override inline fun getItemSeparator(): String {
        if (!TELEMETRY_IS_ENABLED) return " | "
        return around.itemSeparator
    }

    override inline fun setItemSeparator(itemSeparator: String) {
        if (!TELEMETRY_IS_ENABLED) return
        around.itemSeparator = itemSeparator
    }

    override inline fun getCaptionValueSeparator(): String {
        if (!TELEMETRY_IS_ENABLED) return " : "
        return around.captionValueSeparator
    }

    override inline fun setCaptionValueSeparator(captionValueSeparator: String?) {
        if (!TELEMETRY_IS_ENABLED) return
        around.captionValueSeparator = captionValueSeparator
    }

    override inline fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat?) {
        if (!TELEMETRY_IS_ENABLED) return
        around.setDisplayFormat(displayFormat)
    }

    override inline fun log(): Telemetry.Log {
        if (!TELEMETRY_IS_ENABLED) return DUMMY_LOG
        return around.log()
    }

}