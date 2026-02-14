package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver

class PinpointTask(private val pinpoint: GoBildaPinpoint2Driver) : Task<PinpointTask>() {
    companion object {
        private val TAGS = setOf(BuiltInTags.DAEMON)
    }

    override fun getTags(): Set<String> {
        return TAGS
    }

    override fun onTick(): Boolean {
        pinpoint.update()
        return false
    }
}