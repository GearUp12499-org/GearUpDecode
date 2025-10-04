package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.IndexerHardware;
import org.firstinspires.ftc.teamcode.tasks.Indexer0Task;

import io.github.gearup12499.taskshark.FastScheduler;

@Autonomous
public class IndexerAuto extends LinearOpMode {
    IndexerHardware hardware;
    FastScheduler scheduler;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new IndexerHardware(hardwareMap);
        scheduler = new FastScheduler();

        scheduler.add(new Indexer0Task(hardware.indexer, hardware.indexer1));

        waitForStart();

        while (opModeIsActive()){
            scheduler.tick();
        }
    }
}
