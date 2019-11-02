package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="AutonomousMode", group="Robot")
@Disabled
public class SkystoneAutonomousOpMode extends OpMode {

    private FieldGraph mFieldGraph;

    private FieldTile mCurrentBlockLocation = null;

    public SkystoneAutonomousOpMode(){

        // Create the field graph
    //    mFieldGraph = new FieldGraph(6, 6);

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

 /*   *//**
     *
     * @param
     *//*
    public void setRobotLocation(int xloc,yloc){
        FieldTile block = mFieldGraph.getTile(blockNum);
        if (block != null){
            mCurrentBlockLocation = block;
        }
    }

*/

    private void initRoutes() {
        // Adds the viable routes to the field graph.

    }
}
