package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hwqueue.GoBildaPinpointAsync;
import org.firstinspires.ftc.teamcode.perform.PerfTwoLayerOpMode;

@TeleOp
public class ThreadedPinpointProfiler extends PerfTwoLayerOpMode {
    @Override
    public void run() {
        Log.i("TPP", "Hello world!");
        GoBildaPinpointAsync asyncPinpoint = new GoBildaPinpointAsync(hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint"));
        asyncPinpoint.getUnderlying().resetPosAndIMU();
        asyncPinpoint.update();
        waitForStart();

        while (hardware(this::opModeIsActive)) {
            Log.i("TPP", "tick");
            asyncPinpoint.update();
            hw.waitForAll();
        }

        phw.write();
    }
}
