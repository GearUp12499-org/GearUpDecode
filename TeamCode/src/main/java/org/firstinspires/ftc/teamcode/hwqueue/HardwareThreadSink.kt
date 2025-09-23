package org.firstinspires.ftc.teamcode.hwqueue

import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.Future
import java.util.concurrent.locks.Condition
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

class HardwareThreadSink {
    private val asynchronous = ConcurrentLinkedQueue<HardwareQuery<*>>()
    private val synchronous = ConcurrentLinkedQueue<SyncHardwareQuery<*>>()
    @Volatile
    private var stopFlag = false
    private val signalling = ReentrantLock()
    private val stateUpdated: Condition = signalling.newCondition()
    private val emptied: Condition = signalling.newCondition()

    fun <T> supply(query: HardwareQuery<T>): Future<T> {
        asynchronous.add(query)
        signalling.withLock {
            stateUpdated.signal()
        }
        return query.get()
    }

    fun <T> supplySync(query: SyncHardwareQuery<T>): T {
        synchronous.add(query)
        signalling.withLock {
            stateUpdated.signal()
        }
        return query.wait()
    }

    fun quit() {
        signalling.withLock {
            stopFlag = true
            stateUpdated.signalAll()
            emptied.signalAll()
        }
    }

    private fun iteration(): Boolean {
        if (stopFlag) return true
        signalling.withLock {
            stateUpdated.await()
        }
        if (stopFlag) return true
        var didAnythingHappen = false
        while (true) {
            val item1: SyncHardwareQuery<*>? = synchronous.poll()
            if (item1 != null) {
                if (item1.isRevoked) continue
                didAnythingHappen = true
                item1.run()
                continue
            }
            val item2: HardwareQuery<*>? = asynchronous.poll()
            if (item2 != null) {
                if (item2.isRevoked) continue
                didAnythingHappen = true
                item2.run()
                continue
            }
            break
        }
        if (didAnythingHappen) signalling.withLock {
            emptied.signalAll()
        }
        return false
    }

    fun run() {
        while (true) {
            if (iteration()) return
        }
    }

    fun waitForAll() {
        while (!asynchronous.isEmpty()) signalling.withLock {
            emptied.await()
        }
    }
}