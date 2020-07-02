package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="BlueAllianceGetStone", group="Robot")
//@Disabled
public class AutoGetStoneBlueAlliance extends OpMode{

    private SkystoneAutonomousOpMode mRealOpmode = null;
    public AutoGetStoneBlueAlliance(){
        mRealOpmode = new SkystoneAutonomousOpMode(this, true,AutonomousController.SEQUENCE_GET_STONE);
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


