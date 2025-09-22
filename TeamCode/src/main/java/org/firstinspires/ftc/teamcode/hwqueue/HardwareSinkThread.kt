package org.firstinspires.ftc.teamcode.hwqueue

import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.Future
import java.util.concurrent.locks.Condition
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock
import kotlin.math.sign

class HardwareSinkThread : Thread() {
    private val theQueue = ConcurrentLinkedQueue<HardwareQuery<*>>()
    @Volatile
    private var stopFlag = false
    private val signalling = ReentrantLock()
    private val stateUpdated: Condition = signalling.newCondition()
    private val emptied: Condition = signalling.newCondition()

    fun <T> supply(query: HardwareQuery<T>): Future<T> {
        theQueue.add(query)
        signalling.withLock {
            stateUpdated.signal()
        }
        return query.get()
    }

    fun quit() {
        signalling.withLock {
            stopFlag = true
            stateUpdated.signalAll()
            emptied.signalAll()
        }
    }

    init {
        isDaemon = true
    }

    private fun iteration(): Boolean {
        if (stopFlag) return true
        signalling.withLock {
            stateUpdated.await()
        }
        if (stopFlag) return true
        var didAnythingHappen = false
        while (true) {
            val item: HardwareQuery<*>? = theQueue.poll()
            if (item == null) break
            didAnythingHappen = true

            item.run()
        }
        if (didAnythingHappen) signalling.withLock {
            emptied.signalAll()
        }
        return false
    }

    override fun run() {
        while (true) {
            if (iteration()) return
        }
    }

    fun waitForAll() {
        while (!theQueue.isEmpty()) signalling.withLock {
            emptied.await()
        }
    }
}