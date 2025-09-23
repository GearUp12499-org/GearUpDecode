package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.hwqueue.GoBildaPinpointAsync;
import org.firstinspires.ftc.teamcode.hwqueue.TwoLayerOpMode;

public class ThreadedOpMode extends TwoLayerOpMode {
    @Override
    public void run() {
        GoBildaPinpointAsync pinpoint = new GoBildaPinpointAsync(hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint"));

        while (opModeIsActive()) {

            hw.waitForAll();
        }

        hw.quit();
    }
}
