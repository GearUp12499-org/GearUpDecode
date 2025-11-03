package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware

class Indexer(private val indexerMotor: DcMotor) : Task<Indexer>() {
    companion object {
        private val TAGS = setOf(BuiltInTags.DAEMON)
        private val LOCK_ROOT = Lock.StrLock("indexer_impl")
    }

    val lock = LOCK_ROOT.derive()

    init {
        require(CompBotHardware.Locks.INDEXER)
    }

    override fun getTags(): Set<String> = TAGS

    override fun onTick(): Boolean {
        TODO("Not yet implemented")
    }

    override fun onFinish(completedNormally: Boolean) {
        indexerMotor.power = 0.0
    }
}