package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.Task
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver

class PinpointUpdater(val it: GoBildaPinpoint2Driver) : Task<PinpointUpdater>() {
    override fun getTags() = DAEMON_TAGS

    override fun onTick(): Boolean {
        it.update()
        return false
    }
}