package org.firstinspires.ftc.teamcode.perform

import android.util.Log
import org.firstinspires.ftc.teamcode.hwqueue.HardwareQuery
import org.firstinspires.ftc.teamcode.hwqueue.HardwareThreadSink
import org.firstinspires.ftc.teamcode.hwqueue.SyncHardwareQuery
import java.lang.System.nanoTime
import kotlin.concurrent.withLock

class ProfilingHardwareThread : HardwareThreadSink() {
    private val logicIdles = PerformanceHelpers.intContainer()
    private val hwIdles = PerformanceHelpers.intContainer()
    private var ptr = 0

    override fun waitForAll() {
        val start = nanoTime()
        super.waitForAll()
        val end = nanoTime()
        val lastEmptied = lastEmptied
        val idleAcc = idleAccCommit
        val hwIdle = lastEmptied - end + idleAcc
        val nanos = end - start
        logicIdles[ptr] = nanos.toInt()
        hwIdles[ptr] = if (hwIdle.toULong() > 0xffffffffUL) -1 else hwIdle.toInt()
        ptr = (ptr + 1) % logicIdles.size
    }

    @Volatile private var idleAccCommit = 0L
    @Volatile private var idleAcc = 0L
    @Volatile private var lastEmptied = -1L

    override fun iteration(): Boolean {
        if (stopFlag) return true
        signalling.withLock {
            stateUpdated.await()
        }
        if (stopFlag) return true
        var didAnythingHappen = false
        while (true) {
            val item1: SyncHardwareQuery<*>? = synchronous.poll()
            if (item1 != null) {
                if (!didAnythingHappen) idleAcc += nanoTime() - lastEmptied
                if (item1.isRevoked) continue
                didAnythingHappen = true
                item1.run()
                continue
            }
            val item2: HardwareQuery<*>? = asynchronous.poll()
            if (item2 != null) {
                if (!didAnythingHappen) idleAcc += nanoTime() - lastEmptied
                if (item2.isRevoked) continue
                didAnythingHappen = true
                item2.run()
                continue
            }
            break
        }
        idleAccCommit = idleAcc
        lastEmptied = nanoTime()
        signalling.withLock {
            emptied.signalAll()
        }
        return false
    }

    fun write() {
        val f = PerformanceHelpers.findSuitableFile()
        var n = 0
        f.writer(Charsets.UTF_8).use {
            it.write("logic,hw\n");
            for ((logic, hw) in logicIdles zip hwIdles) {
                if (logic == -1) break
                it.write("$logic,$hw\n")
                n++
            }
        }
        Log.i("Profiler", "Wrote $n records to $f")
    }
}