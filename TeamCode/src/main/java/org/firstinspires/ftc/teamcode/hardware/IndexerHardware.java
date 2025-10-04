package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import io.github.gearup12499.taskshark.Lock;

public class IndexerHardware extends HardwareMapper {
    public static final Lock INDEXER_LOCK = new Lock.StrLock("indexer");

    @HardwareName("indexer")
    @AutoClearEncoder
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor indexer;

    @HardwareName("indexer1")
    public DigitalChannel indexer1;
    public IndexerHardware(HardwareMap map) {
        super(map);
        indexer1.setMode(DigitalChannel.Mode.INPUT);
    }
}
