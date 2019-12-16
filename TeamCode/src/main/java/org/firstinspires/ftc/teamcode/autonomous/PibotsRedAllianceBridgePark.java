package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="PibotsRedAllianceBridgePark", group="Robot")
//@Disabled
public class PibotsRedAllianceBridgePark extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public PibotsRedAllianceBridgePark(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, false,AutonomousController.SEQUENCE_PIBOTS_BRIDGE_PARK);
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


