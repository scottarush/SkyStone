package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="BlueAllianceBridgePark", group="Robot")
//@Disabled
public class BridgeParkBlueAlliance extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public BridgeParkBlueAlliance(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, true,AutonomousController.SEQUENCE_BRIDGE_PARK);
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


