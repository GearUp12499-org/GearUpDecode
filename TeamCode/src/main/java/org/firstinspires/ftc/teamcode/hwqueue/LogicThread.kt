package org.firstinspires.ftc.teamcode.hwqueue


class LogicThread(val hw: HardwareThreadSink, val action: Action) : Thread() {
    companion object {
        val threadHw: HardwareThreadSink get() = (currentThread() as? LogicThread)?.hw ?: throw IllegalStateException("No hardware context available!")
        val hw get() = lazy { threadHw }
    }

    fun interface Action {
        fun act()
    }

    override fun run() {
        try {
            action.act()
        } finally {
            hw.quit()
        }
    }
}