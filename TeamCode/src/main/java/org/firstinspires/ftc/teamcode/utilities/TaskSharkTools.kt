package org.firstinspires.ftc.teamcode.utilities

import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Scheduler

private fun describeTask(sb: StringBuilder, t: ITask<*>, sc: Scheduler, indent: Int) {
    sb.append(" ".repeat(indent))
    sb.append(t.describeVerbose())
    sb.append("\n")
    if (t.getState() == ITask.State.NotStarted) {
        if (t.dependedLocks().isNotEmpty()) {
            sb.append(" ".repeat(indent+2))
            sb.append("Locks:\n")
            for (lock in t.dependedLocks()) {
                sb.append(" ".repeat(indent+4))
                sb.append("${lock.getFriendlyName()}\n")
            }
        }
        if (t.dependedTasks().isNotEmpty()) {
            sb.append(" ".repeat(indent+2))
            sb.append("Depends on:\n")
            for (task in t.dependedTasks()) {
                describeTask(sb, task, sc, indent + 4)
            }
        }
    }
}


fun reportIt(sch: Scheduler): String {
    val notStarted: MutableList<ITask<*>> = mutableListOf()
    val starting: MutableList<ITask<*>> = mutableListOf()
    val ticking: MutableList<ITask<*>> = mutableListOf()
    val finishing: MutableList<ITask<*>> = mutableListOf()
    var finished = 0
    var cancelled = 0

    for ((_, task) in sch.tasks) {
        when (task.getState()) {
            ITask.State.NotStarted -> notStarted += task
            ITask.State.Starting -> starting += task
            ITask.State.Ticking -> ticking += task
            ITask.State.Finishing -> finishing += task
            ITask.State.Finished -> finished++
            ITask.State.Cancelled -> cancelled++
        }
    }

    return buildString {
        append("-- Scheduler report --\n")
        append("Overview: NS ${notStarted.size}  S ${starting.size}  T ${ticking.size}  F ${finishing.size}  Fn $finished  Fc $cancelled\n")
        append("${notStarted.size} waiting:\n")
        for (task in notStarted) {
            describeTask(this, task, sch, 2)
        }
        append("${starting.size} starting:\n")
        for (task in starting) {
            describeTask(this, task, sch, 2)
        }
        append("${ticking.size} ticking:\n")
        for (task in ticking) {
            describeTask(this, task, sch, 2)
        }
        append("${finishing.size} finishing:\n")
        for (task in finishing) {
            describeTask(this, task, sch, 2)
        }
    }
}