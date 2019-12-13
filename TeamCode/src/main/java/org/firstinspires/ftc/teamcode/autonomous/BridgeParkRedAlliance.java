package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="RedAllianceBridgePark", group="Robot")
//@Disabled
public class BridgeParkRedAlliance extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public BridgeParkRedAlliance(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, false,AutonomousController.SEQUENCE_BRIDGE_PARK);
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


