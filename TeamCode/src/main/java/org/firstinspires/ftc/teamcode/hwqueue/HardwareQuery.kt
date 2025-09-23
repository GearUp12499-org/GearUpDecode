package org.firstinspires.ftc.teamcode.hwqueue

import java.util.concurrent.CompletableFuture
import java.util.concurrent.Future
import java.util.concurrent.locks.Condition
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

/**
 * Lockable object for processing hardware queries.
 */
abstract class HardwareQuery<R> {
    private val fut = CompletableFuture<R>()
    @Volatile @JvmField var isRevoked = false

    fun revoke() { isRevoked = true }

    abstract fun runInner(): R

    fun run() {
        require(!fut.isDone) { "This query is already completed" }
        fut.complete(runInner())
    }

    fun get(): Future<R> = fut
}

abstract class SyncHardwareQuery<R> : HardwareQuery<R>() {
    fun wait(): R {
        return get().get()
    }
}

inline fun <T> query(crossinline block: () -> T) = object : HardwareQuery<T>() {
    override fun runInner(): T {
        return block()
    }
}

inline fun <T> querySync(sink: HardwareThreadSink, crossinline block: () -> T): T {
    return sink.supplySync(object : SyncHardwareQuery<T>() {
        override fun runInner(): T {
            return block()
        }
    })
}