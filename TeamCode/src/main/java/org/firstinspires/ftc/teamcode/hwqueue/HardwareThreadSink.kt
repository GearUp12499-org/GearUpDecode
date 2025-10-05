package org.firstinspires.ftc.teamcode.hwqueue

import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.Future
import java.util.concurrent.locks.Condition
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

open class HardwareThreadSink {
    protected val asynchronous = ConcurrentLinkedQueue<HardwareQuery<*>>()
    protected val synchronous = ConcurrentLinkedQueue<SyncHardwareQuery<*>>()
    @Volatile
    protected var stopFlag = false
    protected val signalling = ReentrantLock()
    protected val stateUpdated: Condition = signalling.newCondition()
    protected val emptied: Condition = signalling.newCondition()

    open fun <T> supply(query: HardwareQuery<T>): Future<T> {
        asynchronous.add(query)
        signalling.withLock {
            stateUpdated.signal()
        }
        return query.get()
    }

    open fun <T> supplySync(query: SyncHardwareQuery<T>): T {
        synchronous.add(query)
        signalling.withLock {
            stateUpdated.signal()
        }
        return query.wait()
    }

    open fun quit() {
        signalling.withLock {
            stopFlag = true
            stateUpdated.signalAll()
            emptied.signalAll()
        }
    }

    protected open fun iteration(): Boolean {
        if (stopFlag) return true
        signalling.withLock {
            stateUpdated.await()
        }
        if (stopFlag) return true
        while (true) {
            val item1: SyncHardwareQuery<*>? = synchronous.poll()
            if (item1 != null) {
                if (item1.isRevoked) continue
                item1.run()
                continue
            }
            val item2: HardwareQuery<*>? = asynchronous.poll()
            if (item2 != null) {
                if (item2.isRevoked) continue
                item2.run()
                continue
            }
            break
        }
        signalling.withLock {
            emptied.signalAll()
        }
        return false
    }

    open fun run() {
        while (true) {
            if (iteration()) return
        }
    }

    open fun waitForAll() {
        while (!asynchronous.isEmpty()) signalling.withLock {
            emptied.await()
        }
    }
}