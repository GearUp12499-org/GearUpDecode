package org.firstinspires.ftc.teamcode.utilities

import kotlin.time.Duration
import kotlin.time.TimeSource

object StaticStore {
    private val timeSource = TimeSource.Monotonic
    var lastOpMode: TimeSource.Monotonic.ValueTimeMark? = null

    fun mark() {
        lastOpMode = timeSource.markNow()
    }

    fun duration() = lastOpMode?.let {timeSource.markNow() - it} ?: Duration.INFINITE
}