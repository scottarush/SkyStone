package org.firstinspires.ftc.teamcode.archive.autonomous;

import java.util.ArrayList;

/**
 * This is a graph of tiles that represent the field.
 * Block numbers are ordered from the bottom left, starting at 0
 * and scanning left to right and then up back to left
 */
public class FieldGraph {

    private FieldTile[][] mFieldTiles;
    private int mRows = 0;
    private int mCols = 0;

    private double mTileWidth = 0d;
    private double mTileHeight = 0d;

    private double mFieldWidth = 0d;
    private double mFieldHeight = 0d;

    private GraphPosition mRobotPosition = null;

    private Route mSelectedRoute = null;

    /**
     * Encapsulates a position the graph.
     */
    public class GraphPosition {
        public GraphPosition(FieldTile tile, double xoffset,double yoffset){
            mTile = tile;
            mXOffset = Math.round(xoffset * 100)/100d;
            mYOffset = Math.round(yoffset * 100)/100d;
        }
        /**
         * Tile
         */
        public FieldTile mTile= null;
        /**
         * offset in X  from center of block rounded to nearest 10th of units
         */
        public double mXOffset = 0;
        /**
         * offset in Y direction from center of block rounded to nearest 10th of units
         */
        public double mYOffset= 0;

    }

    public FieldGraph(int rows,int columns,double tileWidth,double tileHeight){
        mRows = rows;
        mCols = columns;
        mTileWidth = tileWidth;
        mTileHeight = tileHeight;
        mFieldWidth = mCols*tileWidth;
        mFieldHeight = mRows * tileHeight;
        // Create the graph with default connections

        mFieldTiles = new FieldTile[mRows][mCols];
        int tileNumCounter = 1;
        double y = 0;
        for(int row=0;row < mRows;row++){
            double x = 0;
            for(int col=0;col < mCols;col++){
                // Create the tile and put in the array
                FieldTile tile = new FieldTile(tileNumCounter++,mTileWidth,mTileHeight,x,y);
                mFieldTiles[row][col] = tile;
                x += mTileWidth;
            }
            y += mTileHeight;
        }
        // Now loop through newly created blocks to set the neighbors
        for(int row=0;row < mRows;row++){
            for(int col=0;col < mCols;col++){
                FieldTile tile = mFieldTiles[row][col];
                for(int i=0;i <=3;i++){
                    switch(i) {
                        case FieldTile.EDGE_TOP:
                            if (row < mRows-1) {
                                FieldTile top = mFieldTiles[row + 1][col];
                                tile.setNeighbor(FieldTile.EDGE_TOP, top);
                            }
                            break;
                        case FieldTile.EDGE_LEFT:
                            if (col > 0) {
                                FieldTile leftBlock = mFieldTiles[row][col - 1];
                                tile.setNeighbor(FieldTile.EDGE_LEFT, leftBlock);
                            }
                            break;
                        case FieldTile.EDGE_RIGHT:
                            if (col < mCols - 1) {
                                FieldTile rightBlock = mFieldTiles[row][col + 1];
                                tile.setNeighbor(FieldTile.EDGE_RIGHT, rightBlock);
                            }
                        case FieldTile.EDGE_BOTTOM:
                            if (row > 0) {
                                FieldTile bottomBlock = mFieldTiles[row-1][col];
                                tile.setNeighbor(FieldTile.EDGE_BOTTOM, bottomBlock);
                            }
                    }
                }
            }
        }
    }

    /**
     *
     * @param tileNum
     * @return the FieldTile or null if the block is invalid
     */
    public FieldTile getTile(int tileNum) {
        for (int row = 0; row < mRows; row++) {
            for (int col = 0; col < mCols; col++) {
                FieldTile tile = mFieldTiles[row][col];
                if (tile.getTileLocation().mTileNum == tileNum) {
                    return tile;
                }
            }
        }
        // Invalid block number
        return null;
    }

    /**
     * @return the currently selected route or null if one has not been selected yet.
     */
    public Route getmSelectedRoute(){
        return mSelectedRoute;
    }

    /**
     * @return returns the current robot position or null if position has not been set
     */
    public GraphPosition getRobotPosition() {
        return mRobotPosition;
    }

    /**
     * Translates an absolute field position to a tile-relative position.
     * @return GraphPosition or null if coords invalid
     */
    protected GraphPosition getGraphPosition(double x, double y){
        if ((x < 0d)|| (x > mFieldWidth)){
            return null;
        }
        if ((y < 0d)|| (y > mFieldHeight)){
            return null;
        }
        // Now find the tile that this coord is in.
        for (int row = 0; row < mRows; row++) {
            for (int col = 0; col < mCols; col++) {
                FieldTile tile = mFieldTiles[row][col];
                FieldTile.TileLocation location = tile.getTileLocation();
                if ((x > location.x) && (x < (location.x + mTileWidth))){
                    // right column
                    if ((y > location.y) && (y < (location.y + mTileWidth))){
                        // This is it
                        double xoffset = (x-location.x-mTileWidth/2d);
                        double yoffset = (y-location.y-mTileHeight/2d);
                        GraphPosition pos = new GraphPosition(getTile(location.mTileNum), xoffset, yoffset);
                        return pos;
                    }

                }
            }
        }
        return null;  // Shouldn't happen for validated coords.
    }

    /**
     * Sets the location of the robot in the field graph at the provided location in absolute
     * coordinates.  If the position is not on the currently selected route, then the route will be updated to
     * the provided route.
     * @param x xposition in field coordinates
     * @param y yposition in field coorindates
     * @param startingRoute preferred starting route or null if no preference
     * @return false if either the location or a non-null if route is not a Starting route on this tile.
     */
    protected boolean resetRobotPosition(double x, double y, Route startingRoute){
        if ((x < 0d) || (x > mFieldWidth))
            return false;
        if ((y < 0d) || (y > mFieldHeight))
            return false;
        mRobotPosition = getGraphPosition(x,y);
        mSelectedRoute = null;

        ArrayList<Route> routes = mRobotPosition.mTile.getStartingRoutes();
        if (routes.size() > 0){
            if (startingRoute != null) {
                if (routes.contains(startingRoute)) {
                    mSelectedRoute = startingRoute;
                    return true;
                }
                else{
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * updates the robot position
     * @param x x coordinate of robot center in field coordinates
     * @param y y coordinate or robot center in field coordinates
     * @return true if the robot position matches the expected tile on the route, false
     * if they don't match or no route has been selected.
     */
    public boolean updateRobotPosition(double x, double y, Route route) {
        if ((x < 0d) || (x > mFieldWidth))
            return false;
        if ((y < 0d) || (y > mFieldHeight))
            return false;
        mRobotPosition = getGraphPosition(x, y);
        if (mSelectedRoute == null)
            return false;
        return mSelectedRoute.getNextRouteTransition().startTile != mRobotPosition.mTile;
     }

    /**
     * @return the next maneuver on the currently selected route or null if either a route has not been
     * set, the robot is no longer on the route, or we have reached the end of the selected route.
     *
     * Notes:
     * 1. The caller is responsible for incrementing the route index on the Route within the
     * maneuver when the maneuver is complete and/or insuring that the robot is on the correct tile.
     *
     * 2.  The robot is assumed to be on the mSelectedRoute.  If it has moved off of the route then
     * the maneuver provided will be a direct vector to the next tile.  This may or may not be
     * a possible movement, but no checks are made for possible obstructions.
     *
     **/
    public Maneuver getNextManeuver(){
        if (mRobotPosition == null)
            return null;
        if (mSelectedRoute == null){
            return null;
        }
        // Now get the tile at the current route index
        if (mSelectedRoute.isLastTransition()){
            return null;
        }
        RouteTransition transition = mSelectedRoute.getNextRouteTransition();
        // GCompute the offset to move the current robot location tile location to the end one.
        double ydelta = (transition.endTile.getTileLocation().y)-(mRobotPosition.mYOffset+mRobotPosition.mTile.getTileLocation().y);
        double xdelta = (transition.endTile.getTileLocation().x)-(mRobotPosition.mXOffset+mRobotPosition.mTile.getTileLocation().x);

        MovementManeuver maneuver = new MovementManeuver(xdelta,ydelta,transition);
        return maneuver;
    }


}
