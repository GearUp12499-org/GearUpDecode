package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.IndexerHardware;

import io.github.gearup12499.taskshark.Task;

public class Indexer0Task extends Task<Indexer0Task> {
    private final DcMotor turn;
    private final DigitalChannel sensor;

    public Indexer0Task(DcMotor turn, DigitalChannel sensor){
        this.turn = turn;
        this.sensor = sensor;

        require(IndexerHardware.INDEXER_LOCK);
    }
    @Override
    public void onFinish(boolean completedNormally) {
        turn.setPower(0);
        turn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void onStart() {
        turn.setPower(0.3);
    }

    @Override
    public boolean onTick() {
        return !sensor.getState();
    }
}
