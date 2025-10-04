package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.IndexerHardware;
import org.firstinspires.ftc.teamcode.tasks.Indexer0Task;
import org.firstinspires.ftc.teamcode.tasks.IndexerNextTask;

import io.github.gearup12499.taskshark.FastScheduler;
import io.github.gearup12499.taskshark.ITask;

@TeleOp
public class indexerTeleOp extends LinearOpMode {

    IndexerHardware hardware;

    FastScheduler scheduler;


    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new IndexerHardware(hardwareMap);
        scheduler = new FastScheduler();

        scheduler.add(new Indexer0Task(hardware.indexer, hardware.indexer1));

        IndexerNextTask task = null;

        waitForStart();


        while (opModeIsActive()) {

            boolean indexer = hardware.indexer1.getState();
            double indexermotor = hardware.indexer.getCurrentPosition();

            if(gamepad1.a && (task==null || task.getState() == ITask.State.Finished)){
                task=scheduler.add(new IndexerNextTask(hardware.indexer));
            }

            scheduler.tick();

            telemetry.addData("indexermotor", indexermotor);
            telemetry.addData("indexer", indexer);
            telemetry.update();
        }

    }


}
