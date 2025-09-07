package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark_android.TaskSharkAndroid

@TeleOp
class GroupExample : LinearOpMode() {
    override fun runOpMode() {
        TaskSharkAndroid.setup()
        val scheduler = FastScheduler()

        scheduler.add(Group {
            it.add(Wait.s(0.5))
                .then(OneShot { Log.i("demo", "Task 1") })
            it.add(Wait.s(1.0))
                .then(OneShot { Log.i("demo", "Task 2") })
            it.add(Wait.s(3.0))
                .then(OneShot { Log.i("demo", "Task 3") })
        }).then(OneShot {
            Log.i("demo", "Task 4")
        })

        waitForStart()

        while (opModeIsActive()) scheduler.tick()
    }
}