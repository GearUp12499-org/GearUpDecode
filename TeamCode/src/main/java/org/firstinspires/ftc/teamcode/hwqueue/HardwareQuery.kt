package org.firstinspires.ftc.teamcode.hwqueue

import java.util.concurrent.CompletableFuture
import java.util.concurrent.Future

/**
 * Lockable object for processing hardware queries.
 */
abstract class HardwareQuery<R> {
    private val fut = CompletableFuture<R>()


    abstract fun runInner(): R

    fun run() {
        require(!fut.isDone) { "This query is already completed" }
        fut.complete(runInner())
    }

    fun get(): Future<R> = fut
}