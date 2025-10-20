package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DumbledoreHardware extends HardwareMapper {

    public double[] redFarStart = {-63,-16,0};

    public double[] shootPos = {12,-12,-Math.PI/4};

    public double[] blueBase = {-38,-33,0};
    @HardwareName("PinPoint")
    public GoBildaPinpoint2Driver PinPoint;

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backLeft")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("claw")
    public Servo claw;

    @HardwareName("pivot")
    public Servo pivot;


    public DumbledoreHardware(HardwareMap map) {
        super(map);
    }
}
