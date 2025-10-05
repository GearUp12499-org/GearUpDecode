package org.firstinspires.ftc.teamcode.hwqueue

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class TwoLayerOpMode : LinearOpMode() {
    @Volatile lateinit var hw: HardwareThreadSink

    override fun runOpMode() {
        hw = HardwareThreadSink()
        LogicThread(hw, ::run).start()
        hw.run()
    }

    inline fun <T> hardware(crossinline block: () -> T): T {
        return querySync(hw) {
            block()
        }
    }

    abstract fun run()
}