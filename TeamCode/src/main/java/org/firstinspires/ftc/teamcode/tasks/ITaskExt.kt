package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.ITask.State.*
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.prefabs.OneShot

@Suppress("NOTHING_TO_INLINE")
inline fun <T: ITask<*>> T.isAliveOrQueued() = when (getState()) {
    NotStarted, Starting, Ticking -> true
    Finishing, Finished, Cancelled -> false
}

fun Scheduler.stopUsing(lock: Lock) = tasks.values
    .filter { lock in it.dependedLocks() && it.isAliveOrQueued() }
    .forEach(ITask<*>::stop)

fun raceTasks(vararg tasks: ITask<*>): Collection<ITask<*>> {
    val stopAll = OneShot {
        tasks.forEach(ITask<*>::finish)
    }
    for (task in tasks) {
        task.then(stopAll)
    }
    return tasks.toList()
}

fun <T: ITask<*>> Collection<ITask<*>>.then(next: T): T {
    for (task in this) {
        task.then(next)
    }
    return next
}