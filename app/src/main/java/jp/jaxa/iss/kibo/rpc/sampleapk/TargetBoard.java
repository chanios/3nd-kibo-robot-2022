package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

public class TargetBoard {

    private MatOfInt boardIDs;
    private List<Mat> objPoints;

    public List<Mat> getObjPoints() {
        return objPoints;
    }

    public MatOfInt getBoardIDs() {
        return boardIDs;
    }

    public TargetBoard target1() {
        int[] id = new int[] { 1, 2, 3, 4 };
        boardIDs = new MatOfInt();
        boardIDs.fromArray(id);

        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.075f, 0.0625f, 0.0f));
        c0.add(new Point3(0.125f, 0.0625f, 0.0f));
        c0.add(new Point3(0.125f, 0.0125f, 0.0f));
        c0.add(new Point3(0.075f, 0.0125f, 0.0f));

        c1.add(new Point3(-0.125f, 0.0625f, 0.0f));
        c1.add(new Point3(-0.075f, 0.0625f, 0.0f));
        c1.add(new Point3(-0.075f, 0.0125f, 0.0f));
        c1.add(new Point3(-0.125f, 0.0125f, 0.0f));

        c2.add(new Point3(-0.125f, -0.0125f, 0.0f));
        c2.add(new Point3(-0.075f, -0.0125f, 0.0f));
        c2.add(new Point3(-0.075f, -0.0625f, 0.0f));
        c2.add(new Point3(-0.125f, -0.0625f, 0.0f));

        c3.add(new Point3(0.075f, -0.0125f, 0.0f));
        c3.add(new Point3(0.125f, -0.0125f, 0.0f));
        c3.add(new Point3(0.125f, -0.0625f, 0.0f));
        c3.add(new Point3(0.075f, -0.0625f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);
        objPoints = new ArrayList<Mat>();
        objPoints.add(c_id0);
        objPoints.add(c_id1);
        objPoints.add(c_id2);
        objPoints.add(c_id3);

        return this;
    }

    public TargetBoard target2() {

        int[] id = new int[] { 11, 12, 13, 14 };
        boardIDs = new MatOfInt();
        boardIDs.fromArray(id);

        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.0875f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0165f, 0.0f));
        c0.add(new Point3(0.0875f, 0.0165f, 0.0f));

        c1.add(new Point3(-0.1375f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0165f, 0.0f));
        c1.add(new Point3(-0.1375f, 0.0165f, 0.0f));

        c2.add(new Point3(-0.1375f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0665f, 0.0f));
        c2.add(new Point3(-0.1375f, -0.0665f, 0.0f));

        c3.add(new Point3(0.0875f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0665f, 0.0f));
        c3.add(new Point3(0.0875f, -0.0665f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);
        objPoints = new ArrayList<Mat>();
        objPoints.add(c_id0);
        objPoints.add(c_id1);
        objPoints.add(c_id2);
        objPoints.add(c_id3);

        return this;
    }
}
