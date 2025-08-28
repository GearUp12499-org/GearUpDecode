package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait

@TeleOp
class TaskSharkExample : LinearOpMode() {
    override fun runOpMode() {
        val scheduler = FastScheduler()

        scheduler
            .add(OneShot {
                Log.i("TaskSharkExample", "hello, world!")
            })
            .then(Wait.s(1))
            .then(OneShot {
                Log.i("TaskSharkExample", "this should be delayed!")
            })


        waitForStart()
        while (opModeIsActive()) scheduler.tick()
    }
}