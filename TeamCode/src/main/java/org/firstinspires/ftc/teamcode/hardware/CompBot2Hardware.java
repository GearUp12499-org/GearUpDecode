package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CompBot2Hardware extends HardwareMapper {
    @HardwareName("shoot1")
    @Reversed
    public DcMotorEx shoot1;

    @HardwareName("shoot2")
    public DcMotorEx shoot2;

    @HardwareName("axon")
    public Servo axonServo;

    @HardwareName("axonF")
    public AnalogInput axonEncoder;

    public CompBot2Hardware(HardwareMap map) {
        super(map);

    }
}
