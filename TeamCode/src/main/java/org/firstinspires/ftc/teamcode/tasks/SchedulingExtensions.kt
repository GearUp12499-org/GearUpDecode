package org.firstinspires.ftc.teamcode.tasks

import android.util.Log
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.api.BuiltInTags

fun Scheduler.stopAllWith(lock: Lock) {
    tasks.values.filter {
        it.getState() != ITask.State.Finished && it.getState() != ITask.State.Cancelled
                && it.dependedLocks().contains(lock)
    }.forEach(ITask<*>::stop)
}

val DAEMON_TAGS = setOf(BuiltInTags.DAEMON)

fun <T : ITask<*>> T.debug(): T {
    Log.i("TaskSharkDebug", this.dependedTasks().toString())
    return this
}