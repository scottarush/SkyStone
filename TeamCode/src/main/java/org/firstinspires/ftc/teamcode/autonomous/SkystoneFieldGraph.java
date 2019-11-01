package org.firstinspires.ftc.teamcode.autonomous;

public class SkystoneFieldGraph extends FieldGraph {

    /** size of each block in inches. **/
    public static final double BLOCK_WIDTH = 24.0d;
    public static final double BLOCK_HEIGHT = 24.0d;

    public static final int NUM_COLS = 6;
    public static final int NUM_ROWS = 6;

    public class Coord{
        public double x;
        public double y;
    }

    public SkystoneFieldGraph(){
        super(NUM_ROWS, NUM_COLS);
    }

    /**
     * Sets the robot location on the field in inches.
     */
    public void setRobotLocation(double x, double y){

    }

    /**
     * sets the robot location to the block.  Coord system is
     * center of the field.
     */
    public FieldBlock translateLocationToBlock(double x, double y){

    }
}
