package org.firstinspires.ftc.teamcode.tasks

import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.systemPackages

@DslMarker
internal annotation class TaskComposerDsl

class ComposedTask(
    private val canStartX: Task<ComposedTask>.() -> Boolean,
    private val onStartX: Task<ComposedTask>.() -> Unit,
    private val onTickX: Task<ComposedTask>.() -> Boolean,
    private val onFinishX: Task<ComposedTask>.(completedNormally: Boolean) -> Unit,
    private val tags: Set<String>,
    reqLock: Set<Lock>,
) : Task<ComposedTask>() {
    companion object {
        init {
            systemPackages.add(ComposedTask::class.qualifiedName!!)
        }
    }

    init {
        reqLock.forEach(::require)
    }

    override fun getTags(): Set<String> {
        return tags
    }

    override fun canStart(): Boolean {
        return canStartX()
    }

    override fun onStart() {
        onStartX()
    }

    override fun onFinish(completedNormally: Boolean) {
        onFinishX(completedNormally)
    }

    override fun onTick(): Boolean {
        return onTickX()
    }
}

@TaskComposerDsl
class ComposeBuilder {
    private var canStart: Task<ComposedTask>.() -> Boolean = { true }
    private var onStart: Task<ComposedTask>.() -> Unit = {}
    private var onTick: Task<ComposedTask>.() -> Boolean = { true }
    private var onFinish: Task<ComposedTask>.(completedNormally: Boolean) -> Unit = {}
    private val tags: MutableSet<String> = mutableSetOf()
    private val locks: MutableSet<Lock> = mutableSetOf()

    fun canStart(it: Task<ComposedTask>.() -> Boolean) {
        canStart = it
    }

    fun onStart(it: Task<ComposedTask>.() -> Unit) {
        onStart = it
    }

    fun onTick(it: Task<ComposedTask>.() -> Boolean) {
        onTick = it
    }

    fun onFinish(it: Task<ComposedTask>.(completedNormally: Boolean) -> Unit) {
        onFinish = it
    }

    fun daemon() = tag(BuiltInTags.DAEMON)
    fun tag(tag: String) {
        tags.add(tag)
    }

    fun require(lock: Lock) {
        locks.add(lock)
    }

    fun build() = ComposedTask(
        canStart,
        onStart,
        onTick,
        onFinish,
        tags,
        locks
    )
}

inline fun compose(conf: ComposeBuilder.() -> Unit) = ComposeBuilder().also(conf).build()