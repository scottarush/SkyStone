package org.firstinspires.ftc.teamcode.autonomous;

import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * This is a data structure class used by the FieldGraph to represent the
 * each block.
 */
public class FieldTile {

    public static final int EDGE_TOP = 0;
    public static final int EDGE_RIGHT = 1;
    public static final int EDGE_BOTTOM = 2;
    public static final int EDGE_LEFT = 3;


    private FieldTile mNeighbors[] = new FieldTile[4];

    /**
     * List of routes that start on this tile. Routes in this
     * list will also be in the outgoing routes
     */
    private ArrayList<Route>mStartingRoutes = new ArrayList<>();
    /**
     * List of routes that end on this tile.  Routes in this
     * list will also be in the incoming routes.
     */
    private ArrayList<Route>mEndingRoutes = new ArrayList<>();

    /**
     * Array of Lists of incoming routes on each edges
     */
    private ArrayList<Route> mIncomingRoutesArray[] = null;
    /**
     * Array of Lists of outgoing routes on each edges.
     */
    private ArrayList<Route> mOutgoingRoutesArray[] = null;

    private TileLocation mTileCoord = null;

    public FieldTile(int tileNum,double xbase, double ybase){
        mTileCoord = new TileLocation(tileNum,xbase,ybase);
        for(int i=0;i < 3;i++){
            mIncomingRoutesArray[i] = new ArrayList<>();
            mOutgoingRoutesArray[i] = new ArrayList<>();
        }
     }

    public static class TileLocation {
        TileLocation(int tileNum, double x, double y){
            this.x = x;
            this.y = y;
            this.blocknum = tileNum;

        }
        /** left edge of tile  **/
        public double x = 0;
        /** lower edge of tile  **/
        public double y = 0;
        public int blocknum = 0;
    }


    public TileLocation getTileLocation(){
        return mTileCoord;
    }
    /**
     * Sets a neighboor
     */
    public boolean setNeighbor(int edge, FieldTile block){
        if (!isEdgeValid(edge)){
            return false;
        }
        mNeighbors[edge] = block;
        return true;
    }


    public static boolean isEdgeValid(int edge){
        if ((edge < 0) || (edge > 3)) {
            return false;
        }
        return true;
    }

    public boolean hasNeighbor(int edge){
        if (!isEdgeValid(edge))
            return false;
        return mNeighbors[edge] != null;
    }

    /**
     * Called to set this tile as the start of a route.
     * @param r
     */
    public void setStartOfRoute(Route r){
        mStartingRoutes.add(r);
    }
    /**
     * Called to set this tile as the end of a route.
     * @param r
     * @return true on success, false if this route is not already in incoming connections
     */
    public boolean setEndOfRoute(Route r){
        boolean found = false;
        for(int i=0;i <= 3;i++) {
            if (mIncomingRoutesArray[i].contains(r)){
                found = true;
                break;
            }
        }
        if (!found)
            return false;
        mEndingRoutes.add(r);
        return true;
    }

    /**
     * Called to add this route as an outgoing connection on this tile.
     * @param r
     * @param destTile
     * @return false if the destTile is not a neighbor of this tile.
     */
    public boolean addRouteTransition(Route r, FieldTile destTile){
        // Find the edge where the route goes.
        int edge = 0;
        for(int i=0;i <= 3;i++){
            if (mNeighbors[i] == destTile){
                // This is it.  add it to the correct edge array
                mOutgoingRoutesArray[i].add(r);
                // And add this tile to the mIncomingRoutesArray on the correct edge
                int destEdge = 0;
                switch(i){
                    case EDGE_LEFT:
                        destEdge = EDGE_RIGHT;
                        break;
                    case EDGE_RIGHT:
                        destEdge = EDGE_LEFT;
                        break;
                    case EDGE_BOTTOM:
                        destEdge = EDGE_TOP;
                        break;
                    case EDGE_TOP:
                        destEdge = EDGE_BOTTOM;
                        break;
                }
                destTile.mIncomingRoutesArray[destEdge].add(r);
                return true;
            }
        }
        // Must not have been a neighbor
        return false;
    }

}
