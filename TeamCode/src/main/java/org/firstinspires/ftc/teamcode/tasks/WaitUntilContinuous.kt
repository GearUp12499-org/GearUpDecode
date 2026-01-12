package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages

class WaitUntilContinuous(private val duration: Double, private val cond: Condition) : Task<WaitUntilContinuous>() {
    companion object {
        init {
            systemPackages.add(WaitUntilContinuous::class.qualifiedName!!)
        }
    }

    fun interface Condition {
        fun check(): Boolean
    }

    private var isMatching = false
    private var startedMatching = 0L

    override fun onTick(): Boolean {
        if (cond.check()) {
            if (!isMatching) {
                isMatching = true
                startedMatching = System.nanoTime()
            }
            if ((System.nanoTime() - startedMatching) / 1.0e9 > duration) {
                return true
            }
        } else {
            if (isMatching) isMatching = false
        }
        return false
    }
}