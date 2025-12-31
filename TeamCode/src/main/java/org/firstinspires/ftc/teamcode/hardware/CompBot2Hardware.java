package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver;

public class CompBot2Hardware extends HardwareMapper {
    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx frontRight;

    @HardwareName("backRight")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx backRight;

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx frontLeft;

    @HardwareName("backLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotorEx backLeft;

    @HardwareName("turret")
    public DcMotorEx turret;

    @HardwareName("intake")
    @Reversed
    public DcMotorEx intake;

    @HardwareName("shoot1")
    @Reversed
    public DcMotorEx shoot1;

    @HardwareName("shoot2")
    public DcMotorEx shoot2;

    @HardwareName("ballStop")
    public ServoImplEx ballStop;

    @HardwareName("ballStopEncoder")
    public AnalogInput ballStopEncoder;

    @HardwareName("hood")
    public ServoImplEx hood;

    @HardwareName("hoodEncoder")
    public AnalogInput hoodEncoder;

    @HardwareName("slider")
    public ServoImplEx slider;

    @HardwareName("sliderEncoder")
    public AnalogInput sliderEncoder;

    @HardwareName("dropDown")
    public ServoImplEx dropDown;

    @HardwareName("dropDownEncoder")
    public AnalogInput dropDownEncoder;

    @HardwareName("pinpoint")
    public GoBildaPinpoint2Driver pinpoint;

    @HardwareName("distanceRight")
    public Rev2mDistanceSensor distanceRight;

    @HardwareName("distanceLeft")
    public Rev2mDistanceSensor distanceLeft;

    @HardwareName("colorTopRight")
    public RevColorSensorV3 colorTopRight;

    @HardwareName("colorBottomRight")
    public RevColorSensorV3 colorBottomRight;

    @HardwareName("colorTopLeft")
    public RevColorSensorV3 colorTopLeft;

    @HardwareName("colorBottomLeft")
    public RevColorSensorV3 colorBottomLeft;

    @HardwareName("prism")
    public GoBildaPrismDriver prism;

    @HardwareName("frontRamp")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel frontRamp;

    @HardwareName("middleRamp")
    @DigitalMode(DigitalChannel.Mode.INPUT)
    public DigitalChannel middleRamp;

    public CompBot2Hardware(HardwareMap map) {
        super(map);
    }
}
