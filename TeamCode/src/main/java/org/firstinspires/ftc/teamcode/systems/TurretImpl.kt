package org.firstinspires.ftc.teamcode.systems

import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware

class TurretImpl(private val hw: CompBot2Hardware) : Task<TurretImpl>() {
    companion object {
        private val LOCK_ROOT = Lock.StrLock("turret_impl")

        init {
            systemPackages.add(TurretImpl::class.qualifiedName!!)
        }
    }

    val lock = LOCK_ROOT.derive()

    override fun onTick(): Boolean {
        return false
    }
}