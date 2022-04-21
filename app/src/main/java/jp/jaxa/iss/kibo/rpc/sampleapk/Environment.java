package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.HashMap;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;


public class Environment {

    private HashMap<Integer, Box> koz;

    private HashMap<Integer, Box> kiz;

    private HashMap<Integer, Goal> goals;
    // The Environment constructor
    public Environment() {
        this.koz = new HashMap<Integer, Box>();
        this.kiz = new HashMap<Integer, Box>();
        this.goals = new HashMap<Integer, Goal>();

        this.koz.put(0,new Box(9.8673,-9.18813,3.81957,10.7673,-8.28813,4.81957));
        this.koz.put(1,new Box(9.8585,-9.4500,4.82063,12.0085,-8.5000,4.87063));
        this.koz.put(2,new Box(11.1067,-9.44819,4.87385,12.0067,-8.89819,5.8738));

        this.kiz.put(0,new Box(10.3,-10.2,4.32,11.55,-6.4,5.57));
        this.kiz.put(1,new Box(9.5,-10.5,4.02,10.5,-9.6,4.8));

        // Point1
        this.addGoal(new Goal(
                new Point(10.71000f, -7.70000f, 4.48000f),
                new Quaternion(0f, 0.707f, 0f, 0.707f),
                "point"
        ));


        // Look For Nearby ar tag and lazer em
        this.addGoal(new Goal(
                "lazer_target"
        ));


        // Point2
        this.addGoal(new Goal(
                new Point(11.27460f, -9.92284f, 5.29881f),
                new Quaternion(0f, 0f, -0.707f, 0.707f),
                "point"
        ));


        // Look For Nearby ar tag and lazer em
        this.addGoal(new Goal(
                "lazer_target"
        ));

        // Point2
        this.addGoal(new Goal(
                new Point(11.27460f, -7.89178f, 4.96538f),
                new Quaternion(0f, 0f, -0.707f, 0.707f),
                "point"
        ));

    }
    public Integer getTotalGoal() {
        return this.goals.size();
    }
    public void addGoal(Goal g) {
        this.goals.put(this.goals.size(),g);
    }
    public Goal getGoal(Integer id) {
        return this.goals.get(id);
    }

    public boolean CanGo(Point p) {
        for (HashMap.Entry<Integer, Box> set : koz.entrySet()) {
            if(isPointInsideAABB(p, set.getValue())) return false;
        }

        for (HashMap.Entry<Integer, Box> set : kiz.entrySet()) {
            if(isPointInsideAABB(p, set.getValue())) return true;
        }

        return false;
    }
    public boolean isPointInsideAABB(Point point,Box box) {
        return (point.getX() >= box.x_min && point.getX() <= box.x_max) &&
                (point.getY() >= box.y_min && point.getY() <= box.y_max) &&
                (point.getZ() >= box.z_min && point.getZ() <= box.z_max);
    }
    public boolean intersect(Box a, Box b) {
        return (a.x_min <= b.x_max && a.x_max >= b.x_min) &&
                (a.y_min <= b.y_max && a.y_max >= b.y_min) &&
                (a.z_min <= b.z_max && a.z_max >= b.z_min);
    }
}
