package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.Task

/**
 * Doesn't start until you tell it to with [requestStart].
 */
class SentinelTask : Task<SentinelTask>() {
    private var started = false

    fun requestStart() {
        started = true
    }

    override fun canStart() = started

    override fun onTick() = true
}