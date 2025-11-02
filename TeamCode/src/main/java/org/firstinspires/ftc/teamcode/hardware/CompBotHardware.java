package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CompBotHardware extends HardwareMapper {

    public double[] redFarStart = {-63, -16, 0};

    public double[] shootPos = {12, -12, -Math.PI / 4};

    public double[] blueBase = {-38, -33, 0};

    public double[] gateWaypoint = {0, -48, 0};

    public double[] gatePos = {0, -55, 0};

    @HardwareName("pinpoint")
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

    @HardwareName("indexer")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor indexer;

    @EncoderFor("indexer")
    @AutoClearEncoder
    public Encoder indexerEncoder;

    @HardwareName("intake")
    public DcMotor intake;


    public CompBotHardware(HardwareMap map) {
        super(map);
    }
}
