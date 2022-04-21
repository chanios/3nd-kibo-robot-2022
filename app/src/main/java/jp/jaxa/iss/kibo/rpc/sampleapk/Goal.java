package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Goal {
    Point position;
    Quaternion orientation;
    String type;

    public Goal(String type) {
        this.type = type;
    }

    public Goal(Point position, Quaternion orientation, String type) {
        this.position = position;
        this.orientation = orientation;
        this.type = type;
    }
}
