package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.ITask.State.*
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Scheduler

@Suppress("NOTHING_TO_INLINE")
inline fun <T: ITask<*>> T.isAliveOrQueued() = when (getState()) {
    NotStarted, Starting, Ticking -> true
    Finishing, Finished, Cancelled -> false
}

fun Scheduler.stopUsing(lock: Lock) = tasks.values
    .filter { lock in it.dependedLocks() && it.isAliveOrQueued() }
    .forEach(ITask<*>::stop)