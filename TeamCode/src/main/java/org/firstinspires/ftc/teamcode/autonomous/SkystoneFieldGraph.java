package org.firstinspires.ftc.teamcode.autonomous;

/**
 * SkystoneFile specific graph.  Layout is as follows:
 *
 */

public class SkystoneFieldGraph extends FieldGraph {

    /** size of each block in inches. **/
    public static final double TILE_WIDTH = 24.0d;
    public static final double TILE_HEIGHT = 24.0d;

    public static final int NUM_COLS = 6;
    public static final int NUM_ROWS = 6;

    private GraphPosition[][] mTileLocations;

    private GraphPosition mCurrentPosition = null;

    private Route mWhiteRouteBlue = new Route("BLUE_WHITE", this);

    public SkystoneFieldGraph(){
        super(NUM_ROWS, NUM_COLS,TILE_WIDTH,TILE_HEIGHT);

        // Add all the routes
        addBlueWhiteRoute(mWhiteRouteBlue);
    }

    /**
     * Sets the robot location on the field in absolute skystone coordinates
     * @return true if location was valid, false otherwise
     */
    public boolean setRobotLocation(double x, double y){
        GraphPosition position  = getGraphPositionAt(x,y);
        if (position == null) {
            // Bad position.  Null out current position and return false
            mCurrentPosition = null;
            return false;
        }
        // Valid position, save it
        mCurrentPosition = position;
        return true;
    }


    /**
     *  Blue White route.
     */
    private void addBlueWhiteRoute(Route r){
        r.setStartTile(7);
        r.addRouteTransition(1);
        r.addRouteTransition(2);
        r.addRouteTransition(1);
        r.addRouteTransition(7);
        r.addRouteTransition(13);
        r.addRouteTransition(19);
        r.addRouteTransition(25);
        r.addRouteTransition(31);
        r.addRouteTransition(32);
        r.addRouteTransition(31);
        r.addRouteTransition(25);
        r.addRouteTransition(19);
        r.setEndTile(19);
    }
}
