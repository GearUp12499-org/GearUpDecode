package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.IndexerHardware;

import io.github.gearup12499.taskshark.Task;

public class IndexerNextTask extends Task<IndexerNextTask> {

    private final DcMotor turn;

    public IndexerNextTask(DcMotor turn){
        this.turn = turn;

        require(IndexerHardware.INDEXER_LOCK);
    }
    public static final int ticksPerRotation = 980;
    public static final int ticksPerStep = ticksPerRotation/6;

    @Override
    public void onStart() {
        turn.setTargetPosition(turn.getCurrentPosition()-ticksPerStep);
        turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turn.setPower(0.5);
    }

    @Override
    public void onFinish(boolean completedNormally) {
        turn.setPower(0);
    }

    @Override
    public boolean onTick() {

        int maxError = 10;


        if(Math.abs(turn.getCurrentPosition()-turn.getTargetPosition()) <  maxError){
            return true;
        }
        return false;
    }
}
