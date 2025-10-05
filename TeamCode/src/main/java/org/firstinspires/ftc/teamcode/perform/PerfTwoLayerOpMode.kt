package org.firstinspires.ftc.teamcode.perform

import org.firstinspires.ftc.teamcode.hwqueue.LogicThread
import org.firstinspires.ftc.teamcode.hwqueue.TwoLayerOpMode

abstract class PerfTwoLayerOpMode : TwoLayerOpMode() {
    lateinit var phw: ProfilingHardwareThread
    override fun runOpMode() {
        phw = ProfilingHardwareThread()
        hw = phw
        LogicThread(hw, ::run).start()
        hw.run()
    }
}