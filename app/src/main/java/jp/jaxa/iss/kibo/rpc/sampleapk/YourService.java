package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Board;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

import java.util.ArrayList;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    Environment e;

    Integer laser_count = 0;

    Integer point_count = 0;

    Dictionary dict =  Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

    final Integer RETRY_MAX = 5;

    @Override
    protected void runPlan1(){
        this.e = new Environment();

        this.prePlanning();

        api.startMission();

        int i = 0;
        while (e.goals.size() > i) {
            Goal current = e.goals.get(i);
            DoGoal(current);
            i++;
        }

        api.reportMissionCompletion();
    }
    private void prePlanning() {
        // pre calculate path
        Point lastPosition = this.getMyPosition();

        int t = 0;
        while (this.e.goals.size() > t) {
            Goal goal = this.e.goals.get(t);
            if(goal.type.equals("point")) {
                goal.setPath(getPath(lastPosition, goal.position));
                lastPosition = goal.position;
            }
            t++;
        }
        return;
    }
    private Point getMyPosition() {
        Kinematics k = api.getRobotKinematics();

        return k.getPosition();
    }
    private Quaternion getOrientation(Point a, Point b) {
        double[] crossed = computeCrossProduct(a,b);

        double w = Math.sqrt(Math.pow(3,2) * Math.pow(3,2)) + DotProductPoint(a, b);

        return new Quaternion((float)crossed[0], (float)crossed[1], (float)crossed[2], (float)w);
    }

    private double DotProductPoint(Point a, Point b) {
        int sum = 0;
        sum += a.getX() * b.getX();
        sum += a.getY() * b.getY();
        sum += a.getZ() * b.getZ();
        return sum;
    }

    public static double[] computeCrossProduct(Point a, Point b) {
        double crossProduct[] = new double[3];

        final double x1 = a.getX();
        final double y1 = a.getY();
        final double z1 = a.getZ();
        final double x2 = b.getX();
        final double y2 = b.getY();
        final double z2 = b.getZ();

        crossProduct[0] = y1 * z2 - z1 * y2;
        crossProduct[1] = z1 * x2 - x1 * z2;
        crossProduct[2] = x1 * y2 - y1 * x2;

        return crossProduct;
    }
    private void DoGoal(Goal goal) {
        if(goal.type.equals("point")) {
            moveToWrapper(goal.position, goal.orientation, goal.path);
            point_count++;
            if(point_count == 1) {
                api.reportPoint1Arrival();
            }
        } else if(goal.type.equals("laser")) {
            api.laserControl(true);
            laser_count++;
            if(laser_count == 1) {
                calibrateWithAruco(new TargetBoard().target1());
                api.takeTarget1Snapshot();
            } else if(laser_count == 2) {
                calibrateWithAruco(new TargetBoard().target2());
                api.takeTarget2Snapshot();
            }
            api.laserControl(false);
        }
        return;
    }
    private void calibrateWithAruco(TargetBoard targetBoard) {
        // get a camera image
        Mat img = api.getMatNavCam();

        Mat ids = new Mat();

        double[][] co = api.getNavCamIntrinsics();

        java.util.List<org.opencv.core.Mat> corners = new ArrayList<>();

        Mat cameraMatrix = new MatOfDouble(co[0]);

        Mat distCoeffs = new MatOfDouble(co[1]);

        Mat rvecs = new Mat();

        Mat tvecs = new Mat();

        Board board = Board.create(targetBoard.getObjPoints(), dict, targetBoard.getBoardIDs());

        Aruco.detectMarkers(img, this.dict, corners, ids);

        Aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvecs, tvecs);

        Aruco.drawAxis(img, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1f);

        api.saveMatImage(img, "test_img");

        return;
    }

    private Path getPath(Point a, Point b) {
        System.out.println("Gettings Path from " + a + " to " + b);

        Pathfinding pathfinding = new Pathfinding(e);

        Path path = pathfinding.FindPath(a,b);

        System.out.println("Found Path " + path.size() + " length");

        return path;
    }

    private void moveToWrapper(Point point, Quaternion quaternion, Path path){

        if(path == null) {
            Result result = api.moveTo(point, quaternion, true);
            Integer loopCounter = 0;
            while(!result.hasSucceeded() && loopCounter < RETRY_MAX){
                result = api.moveTo(point, quaternion, true);
                loopCounter++;
            }
        } else {
            for(Point p: path)
            {
                Result result = api.moveTo(p, quaternion, true);
                Integer loopCounter = 0;
                while(!result.hasSucceeded() && loopCounter < RETRY_MAX){
                    result = api.moveTo(p, quaternion, true);
                    loopCounter++;
                }
            }
        }

        return;
    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        api.relativeMoveTo(point, quaternion, true);
    }

}

