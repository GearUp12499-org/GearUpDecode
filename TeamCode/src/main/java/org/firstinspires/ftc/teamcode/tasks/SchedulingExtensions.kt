package org.firstinspires.ftc.teamcode.tasks

import android.util.Log
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.api.BuiltInTags

fun Scheduler.stopAllWith(lock: Lock) = stopF { "nointerrupt" !in it.getTags() && it.dependedLocks().contains(lock) }

inline fun Scheduler.stopF(predicate: (it: ITask<*>) -> Boolean) {
    tasks.values.forEach {
        when (it.getState()) {
            ITask.State.Finished -> {}
            ITask.State.Cancelled -> {}
            else -> if (predicate(it)) it.stop()
        }
    }
}

val DAEMON_TAGS = setOf(BuiltInTags.DAEMON)

fun <T : ITask<*>> T.debug(): T {
    Log.i("TaskSharkDebug", this.dependedTasks().toString())
    return this
}