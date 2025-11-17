package org.firstinspires.ftc.teamcode

import kotlin.time.Clock
import kotlin.time.Duration
import kotlin.time.ExperimentalTime
import kotlin.time.Instant

/**
 * inter-op-mode-properties
 */
@OptIn(ExperimentalTime::class)
object Lifetime {
    fun bump() {
        lastActive = Clock.System.now()
    }
    var lastActive: Instant = Instant.fromEpochSeconds(0, 0)

    fun timeSinceLastActive(): Duration {
        val now = Clock.System.now()
        val delta = now - lastActive
        return delta
    }
}