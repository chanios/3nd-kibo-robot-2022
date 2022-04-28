package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.SparseArray;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;

import gov.nasa.arc.astrobee.types.Point;

public class Pathfinding {

    public Environment e;

    double RESOLUTION = 0.1;

    HashMap<Integer, Node> NODES = new HashMap<>();

    public Pathfinding(Environment e) {
        this.e = e;
    }

    public LinkedList<Point> nextPoint(Point a) {
        LinkedList<Point> path = new LinkedList();

        double x = a.getX();
        double y = a.getY();
        double z = a.getZ();


        path.push(new Point(x + RESOLUTION, y ,z));
        path.push(new Point(x - RESOLUTION, y ,z));
        path.push(new Point(x, y + RESOLUTION ,z));
        path.push(new Point(x, y - RESOLUTION ,z));
        path.push(new Point(x, y ,z + RESOLUTION));
        path.push(new Point(x, y ,z - RESOLUTION));

        return path;
    }

    public Node getNode(Point p) {
        int key = hashCode(p);

        Node node = this.NODES.get(key);

        if(node == null) {
            node = new Node(p, key);
        }
        return node;
    }

    public boolean RayCast(Point a, Point b) {
        System.out.println("Raycasting from " + a + " to " + b);
        // Compute vector from A to B by subtracting A from B
        double X1 = a.getX();
        double Y1 = a.getY();
        double Z1 = a.getZ();
        double X2 = b.getX();
        double Y2 = b.getY();
        double Z2 = b.getZ();
        double dX = X2-X1;
        double dY = Y2-Y1;
        double dZ = Z2-Z1;

        // Start at A and interpolate along this vector
        int steps = 20;
        int step = 0;
        while (step++ < steps) {
            double factor = (float)step / steps; // runs from 0 to 1 inclusive
            double x = X1 + dX * factor;
            double y = Y1 + dY * factor;
            double z = Z1 + dZ * factor;

            Point checkpoint = new Point(x,y,z);
            System.out.println("Checking " + checkpoint);
            if(!e.CanGo(checkpoint)) {
                System.out.println(true);
                return true;
            }
            System.out.println(false);
        }
        return false;
    }
    public Path OptimzePath(Path path) {
        Point[] paths = path.toArray(new Point[path.size()]);

        for (int i = 0; i < paths.length; i++)
        {
            Point next = paths[i+1];
            Point current = paths[i];
            if(next != null) {
                if(!RayCast(current, next)) {

                }
            }
        }

        Point current = null;
        while(iterator.hasNext()){
            Point point = iterator.next();
            if(current != null && iterator.hasNext()) {
                if(!RayCast(current, point)) {
                    Point point2 = iterator.next();
                    if(!RayCast(current, point2)) {

                    }
                    iterator.remove();
                    continue;
                }
            }
            current = point;
        }
        return path;
    }
    public Path backTrace(Path path,Node node, Point goalPoint) {
        path.addFirst(goalPoint);
        path.addFirst(node.point);
        while (node.parent != null) {
            node = node.parent;
            path.addFirst(node.point);
        }
        OptimzePath(path);
        path.removeFirst();
        path.setComplete(true);
        return path;
    }
    public Path FindPath(Point a, Point b) {
        Path path = new Path();
        SparseArray<Node> OPEN_NODES = new SparseArray<>();
        Node StartNode = getNode(a);

        OPEN_NODES.put(StartNode.key,StartNode);

        while(OPEN_NODES.size() != 0) {
            double lowestcost = Double.POSITIVE_INFINITY;
            Node CurrentNode = null;
            for(int i = 0; i < OPEN_NODES.size(); i++) {
                Node node = OPEN_NODES.valueAt(i);
                if(node.F < lowestcost) {
                    lowestcost = node.F;
                    CurrentNode = node;
                }
            }

            if(CurrentNode == null) { // no more path
                break;
            };

            OPEN_NODES.remove(CurrentNode.key);
            CurrentNode.setClosed(true);

            if(euclideanDistance(CurrentNode.point, b) < 0.3) {
                return backTrace(path, CurrentNode, b);
            }

            for(Point point: nextPoint(CurrentNode.point))
            {
                Node node = getNode(point);
                if(node.closed || !e.CanGo(point)) {
                    continue;
                }

                double G = CurrentNode.G + euclideanDistance(point,CurrentNode.point);

                if(OPEN_NODES.get(node.key) == null || G < node.G) {
                    node.updateNode(G,euclideanDistance(point,b));
                    node.setParent(CurrentNode);
                    if(OPEN_NODES.get(node.key) == null) {
                        OPEN_NODES.put(node.key, node);
                        node.opened = true;
                    }
                }
            }
        }
        return path;
    }

    public static Double euclideanDistance(Point a, Point b) {
        double x = a.getX() - b.getX();
        double y = a.getY() - b.getY();
        double z = a.getZ() - b.getZ();
        return Math.sqrt(x * x + y * y + z * z);
    }

    static final long hashDoubleBits(long hash, double d) {
        hash *= 31L;
        // Treat 0.0d and -0.0d the same (all zero bits)
        if (d == 0.0d)
            return hash;

        return hash + Double.doubleToLongBits(d);
    }
    static public int hashCode(Point p) {
        long bits = 1L;
        bits = hashDoubleBits(bits, p.getX());
        bits = hashDoubleBits(bits, p.getY());
        bits = hashDoubleBits(bits, p.getZ());

        return (int)(bits ^ (bits >> 32));
    }

}
class Node {
    Point point;
    double G;
    double H;
    double F;
    boolean closed;
    boolean opened;
    Node parent;
    Integer key;

    public Node(Point p, Integer key) {
        this.point = p;
        this.key = key;
    }
    public void setClosed(boolean s) {
        this.closed = s;
    }
    public void setOpened(boolean s) {
        this.opened = s;
    }
    public void updateNode(double G, double H) {
        this.G = G;
        this.H = H;
        this.F = G + H;
    }
    public void setParent(Node node) {
        this.parent = node;
    }
}
class Path extends LinkedList<Point> {

    private boolean isComplete = false;

    /**
     * Return true if the path concludes at the CellSpace's goal Cell.
     *
     * @return
     */
    public boolean isComplete() {
        return isComplete;
    }

    protected void setComplete(boolean isComplete) {
        this.isComplete = isComplete;
    }
}
