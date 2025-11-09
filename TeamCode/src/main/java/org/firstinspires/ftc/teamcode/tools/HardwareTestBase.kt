package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.TaskStopException
import java.text.SimpleDateFormat
import java.util.Calendar
import java.util.Locale
import kotlin.time.Clock
import kotlin.time.Instant

abstract class HardwareTestBase : LinearOpMode() {
    companion object {
        const val PASS = "✓"
        const val FAIL = "✗"
        const val WAIT = "…"
        const val INFO = "ℹ"
    }

    abstract class TestableAction(val name: String) : Task<TestableAction>() {
        enum class Result {
            PASS,
            FAIL,
            NO_RESULT,
            INFORMATIONAL
        }

        var startedAt: Double = 0.0
        var result: Result = Result.NO_RESULT; private set
        var reason: String = "No reason specified"; protected set

        override fun onStart() {
            startedAt = System.currentTimeMillis() / 1000.0
        }

        fun informational(info: String) {
            if (result != Result.NO_RESULT) throw IllegalStateException("Test $name is already $result")
            result = Result.INFORMATIONAL
            this.reason = info
        }

        @JvmOverloads
        fun pass(reason: String = "passed") {
            if (result != Result.NO_RESULT) throw IllegalStateException("Test $name is already $result")
            result = Result.PASS
            this.reason = reason
            stop()
        }

        fun fail(reason: String) {
            if (result != Result.NO_RESULT) throw IllegalStateException("Test $name is already $result")
            result = Result.FAIL
            this.reason = reason
            stop()
        }

        abstract fun testIt()

        final override fun onTick(): Boolean {
            try {
                testIt()
                return false
            } catch (e: TaskStopException) {
                throw e
            } catch (e: Exception) {
                e.printStackTrace()
                fail(e.toString())
                // should be unreachable
                throw TaskStopException()
            }
        }

        fun report() = buildString {
            append("[")
            append(when (result) {
                Result.NO_RESULT -> WAIT
                Result.PASS -> PASS
                Result.FAIL -> FAIL
                Result.INFORMATIONAL -> INFO
            })
            append("] ")
            append(name)
            append(": ")
            if (result == Result.NO_RESULT) {
                if (state == ITask.State.NotStarted) {
                    append("waiting...")
                } else {
                    val duration = System.currentTimeMillis() / 1000.0 - startedAt
                    append("%.2fs... %s".format(duration, reason))
                }
            } else {
                append(reason)
            }
        }
    }

    fun <T: Any> watchForChanges(name: String, producer: () -> T?): TestableAction {
        return object : TestableAction(name) {
            var originValue: T? = null

            override fun onStart() {
                super.onStart()
                originValue = producer()
            }

            override fun testIt() {
                if (producer() != originValue) pass()
            }
        }
    }

    fun fallingEdge(producer: () -> Boolean): Task<*> {
        return object: Task.Anonymous() {
            var stage = false
            override fun onTick(): Boolean {
                val p = producer()
                return when {
                    !stage && p -> { stage = true; false }
                    stage && !p -> true
                    else -> false
                }
            }

        }
    }

    protected val tests: MutableList<TestableAction> = mutableListOf()

    fun <T: TestableAction> put(test: T): T {
        tests.add(test)
        return test
    }

    fun failAllIncomplete() {
        tests
            .filter { it.result == TestableAction.Result.NO_RESULT }
            .forEach { it.fail("Did not complete successfully in time") }
    }

    fun buildReport() = buildString {
        // in progress
        val inProgressTests = tests.filter { it.result == TestableAction.Result.NO_RESULT }
        append("%d in progress...\n".format(inProgressTests.size))
        inProgressTests.forEach {
            append(it.report())
            append("\n")
        }
        // then failed
        val failedTests = tests.filter { it.result == TestableAction.Result.FAIL }
        append("%d FAILED:\n".format(failedTests.size))
        failedTests.forEach {
            append(it.report())
            append("\n")
        }
        // then informational
        val infoTests = tests.filter { it.result == TestableAction.Result.INFORMATIONAL }
        append("%d informational:\n".format(infoTests.size))
        infoTests.forEach {
            append(it.report())
            append("\n")
        }
        // then passed
        val passedTests = tests.filter { it.result == TestableAction.Result.PASS }
        append("%d passed:\n".format(passedTests.size))
        passedTests.forEach {
            append(it.report())
            append("\n")
        }
        append("\n")
        append("%d total\n".format(tests.size))
        val cal = Calendar.getInstance().time
        val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US)
        append("Report generated ${sdf.format(cal)}.")
    }
}