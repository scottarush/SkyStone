package org.firstinspires.ftc.teamcode.autonomous;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * This the class for a route.
 */
public class Route {
    private String mName;
    private FieldGraph mGraph;

    FieldTile mStartTile = null;
    FieldTile mEndTile = null;
    FieldTile mCurrentBuildTile = null;


    public Route(String name,FieldGraph graph){
        mName = name;
        mGraph = graph;
    }

    public String getName(){
        return mName;
    }

    /**
     * Sets the starting tile of the route.
     * @return false if tileNum invalid, false otherwise
     */
    public boolean setStartTile(int tileNum){
        FieldTile tile = mGraph.getTile(tileNum);
        if (tile == null)
            return false;
        mStartTile = tile;
        tile.setStartOfRoute(this);
        mCurrentBuildTile = mStartTile;
        return true;
    }

    public boolean addRouteTransition(int destTileNum){
        FieldTile tile = mGraph.getTile(destTileNum);
        if (tile == null)
            return false;
        boolean retcode = mCurrentBuildTile.addRouteTransition(this,tile);
        if (retcode){
            mCurrentBuildTile = tile;
        }
        return retcode;
    }

    /**
     * Sets the ending tile of the route.  This route must have
     * been previously added to the tile or will return an error
     * @return false if tileNum invalid, false otherwise
     */
    public boolean setEndTile(int tileNum){
        FieldTile tile = mGraph.getTile(tileNum);
        if (tile == null)
            return false;
        boolean retcod = tile.setEndOfRoute(this);
        if (retcod){
            mEndTile = tile;
        }
        return retcod;
    }


}
