package org.firstinspires.ftc.teamcode.autonomous;

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

    /**
     * Encapsulates a position the graph.
     */
    public class GraphPosition {
        public GraphPosition(FieldTile tile, double xoffset,double yoffset){
            mTile = tile;
            mXOffset = Math.round(xoffset * 10)/100d;
            mYOffset = Math.round(yoffset * 10)/10d;
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
        // Create the graph with default connections
        mFieldTiles = new FieldTile[mRows][mCols];
        int blockNumCount = 1;
        double y = 0;
        for(int row=0;row < mRows;row++){
            double x = 0;
            for(int col=0;col < mCols;col++){
                // Create the tile and put in the array
                FieldTile tile = new FieldTile(blockNumCount++,x,y);
                mFieldTiles[row][col] = tile;
                x += mTileWidth;
            }
            y += mTileHeight;
        }
        // Now loop through newly created blocks o set the neighbors
        for(int row=0;row < mRows;row++){
            for(int col=0;col < mCols;col++){
                // Create the block
                FieldTile block = mFieldTiles[row][col];
                for(int i=0;i < 4;i++){
                    switch(i) {
                        case FieldTile.EDGE_TOP:
                            if (row > 0) {
                                FieldTile top = mFieldTiles[row - 1][col];
                                block.setNeighbor(FieldTile.EDGE_TOP, top);
                            }
                            break;
                        case FieldTile.EDGE_LEFT:
                            if (col > 0) {
                                FieldTile leftBlock = mFieldTiles[row][col - 1];
                                block.setNeighbor(FieldTile.EDGE_LEFT, leftBlock);
                            }
                            break;
                        case FieldTile.EDGE_RIGHT:
                            if (col < mCols - 1) {
                                FieldTile rightBlock = mFieldTiles[row][col + 1];
                                block.setNeighbor(FieldTile.EDGE_RIGHT, rightBlock);
                            }
                        case FieldTile.EDGE_BOTTOM:
                            if (row < mRows - 1) {
                                FieldTile bottomBlock = mFieldTiles[row+1][col];
                                block.setNeighbor(FieldTile.EDGE_RIGHT, bottomBlock);
                            }
                    }
                }
            }
        }
    }

    /**
     * Adds a route segment
     * @param tileNumber
     * @param edge FieldTile perimeter constant of the OUT_CONNECTION to add
     * @return true if add, false if invalid block or edge was a perimeter (and can't add connection)
     */
    public boolean addRouteSegment(int tileNumber,int edge){
        FieldTile block = getTile(tileNumber);
        if (block == null)
            return false;
        if (!FieldTile.isEdgeValid(edge)){
            return false;
        }
         return true;
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
                if (tile.getTileLocation().blocknum == tileNum) {
                    return tile;
                }
            }
        }
        // Invalid block number
        return null;
    }

    /**
     * translates the location to a tile
     * @return GraphPosition or null if coords invalid
     */
    public GraphPosition getGraphPositionAt(double x, double y){
        double ymax = (mRows * mTileHeight)/2.0d;
        double xmax = (mCols * mTileWidth)/2.0d;
        if ((Math.abs(x) > xmax) || (Math.abs(y) > ymax)){
            return null;
        }
        // Translate x and y coordinates from center to bottom left corner by adding the xmax
        // and ymax values
        double xt = x + xmax;
        double yt = y + ymax;

        // Now find the tile that this coord is in.
        for (int row = 0; row < mRows; row++) {
            for (int col = 0; col < mCols; col++) {
                FieldTile tile = mFieldTiles[row][col];
                FieldTile.TileLocation location = tile.getTileLocation();
                if ((xt > location.x) && (xt < (location.x + mTileWidth))){
                    // right column
                    if ((yt > location.y) && (yt < (location.y + mTileWidth))){
                        // This is it
                        GraphPosition pos = new GraphPosition(getTile(location.blocknum), (xt-location.x),(yt-location.y));
                        return pos;
                    }

                }
            }
        }
        return null;  // Shouldn't happen for validated coords.
    }

}
