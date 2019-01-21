package frc.Utils;

public class GamePieceManipulator {

    public static enum GamePiece {
        HATCH_PANEL, CARGO, NONE
    }

    // placeholder, will need to do something
    public static enum DeployState {

    }

    // placeholder function since Forklift will need to understand which piece
    // the manipulator has
    public GamePiece hasWhichGamePiece() {
        return GamePiece.NONE;
    }

    // placeholder, will need to be changed
    public boolean isDeployed() {
        return true;
    }
}
