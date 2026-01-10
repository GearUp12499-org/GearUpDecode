package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages

class Deferred(private val ctor: () -> ITask<*>?) : Task<Deferred>() {
    companion object {
        init {
            systemPackages.add(Deferred::class.qualifiedName!!)
        }
    }

    private class Check : Task<Check>() {
        override fun onTick(): Boolean {
            return true
        }
    }
    private val check = Check()

    override fun onStart() {
        val task = ctor() ?: return finish()
        scheduler!!.let {
            // as-if then()
            it.add(task)
            it.resurvey(task)
        }
        task.then(check)
    }

    override fun onTick(): Boolean {
        val state = check.getState()
        if (state == ITask.State.Cancelled) stop()
        return state == ITask.State.Finished
    }
}