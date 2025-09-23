package org.firstinspires.ftc.teamcode.hwqueue

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class TwoLayerOpMode : LinearOpMode() {
    @Volatile lateinit var hw: HardwareThreadSink

    final override fun runOpMode() {
        hw = HardwareThreadSink()
        LogicThread(hw, ::run).start()
        hw.run()
    }

    abstract fun run()
}