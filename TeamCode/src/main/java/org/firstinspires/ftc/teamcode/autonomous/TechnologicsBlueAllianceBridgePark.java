package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="TechnologicsBlueAllianceBridgePark", group="Robot")
@Disabled
public class TechnologicsBlueAllianceBridgePark extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public TechnologicsBlueAllianceBridgePark(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, true,AutonomousController.SEQUENCE_TECHNOLOGIC_BRIDGE_PARK);
    }

    @Override
    public void init() {
        mRealOpmode.init();
    }

    @Override
    public void loop() {
        mRealOpmode.loop();
    }


}


