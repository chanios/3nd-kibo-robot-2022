package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.HashMap;
import java.util.LinkedList;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;


public class Environment {

    private HashMap<Integer, Box> koz;

    private HashMap<Integer, Box> kiz;

    public LinkedList<Goal> goals;
    // The Environment constructor
    public Environment() {
        this.koz = new HashMap<Integer, Box>();
        this.kiz = new HashMap<Integer, Box>();
        this.goals = new LinkedList<>();

        this.koz.put(0,new Box(9.8673,-9.18813,3.81957,10.7673,-8.28813,4.81957));
        this.koz.put(1,new Box(9.8585,-9.4500,4.82063,12.0085,-8.5000,4.87063));
        this.koz.put(2,new Box(11.1067,-9.44819,4.87385,12.0067,-8.89819,5.8738));

        this.kiz.put(0,new Box(10.3,-10.2,4.32,11.55,-6.4,5.57));
        this.kiz.put(1,new Box(9.5,-10.5,4.02,10.5,-9.6,4.8));

        // Point1
        this.goals.add(new Goal(
                new Point(10.71000f, -7.70000f, 4.48000f),
                new Quaternion(0f, 0.707f, 0f, 0.707f),
                "point"
        ));


        // Look For Nearby ar tag and laser em
        this.goals.add(new Goal(
                "laser"
        ));


        // Point2
        this.goals.add(new Goal(
                new Point(11.27460f, -9.92284f, 5.29881f),
                new Quaternion(0f, 0f, -0.707f, 0.707f),
                "point"
        ));


        // Look For Nearby ar tag and laser em
        this.goals.add(new Goal(
                "laser"
        ));

        // Point2
        this.goals.add(new Goal(
                new Point(11.27460f, -7.89178f, 4.96538f),
                new Quaternion(0f, 0f, -0.707f, 0.707f),
                "point"
        ));

        System.out.println(goals);

        return;
    }
    public boolean CanGo(Point p) {
        double size = 0.35;
        Box box = new Box(
                p.getX() - size,
                p.getY() - size,
                p.getZ() - size,
                p.getX() + size,
                p.getY() + size,
                p.getZ() + size
        );
        for (HashMap.Entry<Integer, Box> set : koz.entrySet()) {
            if(intersect(box, set.getValue())) return false;
        }

        for (HashMap.Entry<Integer, Box> set : kiz.entrySet()) {
            if(isPointInsideAABB(p, set.getValue())) return true;
        }

        return false;
    }
    public double distance(Box box, Point point) {
        double dx = Math.max(box.x_min - point.getX(), point.getX() - box.x_max);
        double dy = Math.max(box.y_min - point.getY(), point.getY() - box.y_max);
        double dz = Math.max(box.z_min - point.getZ(), point.getZ() - box.z_max);
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
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
