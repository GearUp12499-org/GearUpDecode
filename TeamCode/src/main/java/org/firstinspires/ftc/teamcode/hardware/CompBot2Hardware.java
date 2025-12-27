package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CompBot2Hardware extends HardwareMapper {
    @HardwareName("shoot1")
    @Reversed
    public DcMotorEx shoot1;

    @HardwareName("shoot2")
    public DcMotorEx shoot2;

    public CompBot2Hardware(HardwareMap map) {
        super(map);

    }
}
