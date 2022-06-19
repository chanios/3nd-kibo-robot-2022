package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
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

    double multiplyMatricesCell(double[][] firstMatrix, double[][] secondMatrix, int row, int col) {
        double cell = 0;
        for (int i = 0; i < secondMatrix.length; i++) {
            cell += firstMatrix[row][i] * secondMatrix[i][col];
        }
        return cell;
    }

    double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }
        return result;
    }
    public List<Point> getROI(Mat tvec, ArrayList<MatOfPoint3f> offset, Mat camMatrix, MatOfDouble distortion) {
        List<Point> ROI_points = new ArrayList<Point>();


        double tarx = (double) tvec.get(0, 0)[0];
        double tary = (double) tvec.get(1, 0)[0];
        double tarz = (double) tvec.get(2, 0)[0];

        System.out.println("tarx: "+ tarx);
        System.out.println("tary: "+ tary);
        System.out.println("tarz: "+ tarz);
        Point3 tar_pos = new Point3(tarx, tary, tarz);

        MatOfPoint3f _target3D = new MatOfPoint3f();
        _target3D.fromArray(tar_pos);
        MatOfPoint2f _targetImagePlane = new MatOfPoint2f();
        Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
        Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);

        double[] _r = new double[] { 0.0f, 0.0f, 0.0f };
        double[] _t = new double[] { 0.0f, 0.0f, 0.0f };
        _rvec.put(0, 0, _r);
        _tvec.put(0, 0, _t);

        // find center of marker in 2D image
        Calib3d.projectPoints(_target3D, _rvec, _tvec, camMatrix, distortion, _targetImagePlane);

        int cpx = (int) _targetImagePlane.get(0, 0)[0];
        int cpy = (int) _targetImagePlane.get(0, 0)[1];
        Point center = new Point(cpx, cpy);

        for (int i = 0; i < 4; i++) {

            Calib3d.projectPoints(offset.get(i), _rvec, _tvec, camMatrix, distortion, _targetImagePlane);

            // without distortion parameter
//			Calib3d.projectPoints(offset.get(i), _rvec,_tvec,camMatrix, new MatOfDouble(), _targetImagePlane);

            int _cpx = (int) _targetImagePlane.get(0, 0)[0];
            int _cpy = (int) _targetImagePlane.get(0, 0)[1];
            Point _center = new Point(_cpx, _cpy);
            System.out.print(offset.get(i).get(0, 0)[0] + " " + offset.get(i).get(0, 0)[1] + " "
                    + offset.get(i).get(0, 0)[2] + " ");
            System.out.println(_center);
            ROI_points.add(_center);
        }
        return ROI_points;
    }
    public Mat getWarppedPaper(Mat img, List<Point> src_pts) {

//		img.copyTo(processedImg);
        List<Integer> list_x = new ArrayList<Integer>();
        List<Integer> list_y = new ArrayList<Integer>();

        for (int i = 0; i < 4; i++) {
            list_x.add((int) src_pts.get(i).x);
            list_y.add((int) src_pts.get(i).y);
        }
        Collections.sort(list_x);
        Collections.sort(list_y);

        double max_w = list_x.get(3) - list_x.get(0);
        double max_h = list_y.get(3) - list_y.get(0);
//	    1-------0
//	    |		|
//	    |  x,y  |
//	    |		|
//	    2-------3
        MatOfPoint2f dst_pts = new MatOfPoint2f(new Point(max_w - 1, 0), new Point(0, 0), new Point(0, max_h - 1),
                new Point(max_w - 1, max_h - 1));
        MatOfPoint2f _pts = new MatOfPoint2f();
        _pts.fromList(src_pts);

        Mat perspective_tf = Imgproc.getPerspectiveTransform(_pts, dst_pts);
        Mat warped_img = new Mat();
        Imgproc.warpPerspective(img, warped_img, perspective_tf, new Size(max_w, max_h), Imgproc.INTER_LINEAR);
        return warped_img;
    }

    public ArrayList<MatOfPoint3f> find_ROI3D(Mat rvec, Mat tvec) {

        List<Point3> global_corner = new ArrayList<>();

        // define paper_corner offset from its center

        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot);

        double[][] offset_corner = { new double[] { 0.0875f, -0.0875f, -0.0875f, 0.0875f },
                new double[] { 0.0665f, 0.0665f, -0.0665f, -0.0665f }, new double[] { 0f, 0f, 0f, 0f }, };

        double[][] rotationMatrix = { new double[] { rot.get(0, 0)[0], rot.get(0, 1)[0], rot.get(0, 2)[0] },
                new double[] { rot.get(1, 0)[0], rot.get(1, 1)[0], rot.get(1, 2)[0] },
                new double[] { rot.get(2, 0)[0], rot.get(2, 1)[0], rot.get(2, 2)[0] } };

        double[][] global_offset = multiplyMatrices(rotationMatrix, offset_corner);

        Point3 tar_c0 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][0],
                (double) tvec.get(1, 0)[0] + global_offset[1][0], (double) tvec.get(2, 0)[0] + global_offset[2][0]);

        Point3 tar_c1 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][1],
                (double) tvec.get(1, 0)[0] + global_offset[1][1], (double) tvec.get(2, 0)[0] + global_offset[2][1]);

        Point3 tar_c2 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][2],
                (double) tvec.get(1, 0)[0] + global_offset[1][2], (double) tvec.get(2, 0)[0] + global_offset[2][2]);

        Point3 tar_c3 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][3],
                (double) tvec.get(1, 0)[0] + global_offset[1][3], (double) tvec.get(2, 0)[0] + global_offset[2][3]);

        MatOfPoint3f offset_c0 = new MatOfPoint3f();
        offset_c0.fromArray(tar_c0);

        MatOfPoint3f offset_c1 = new MatOfPoint3f();
        offset_c1.fromArray(tar_c1);

        MatOfPoint3f offset_c2 = new MatOfPoint3f();
        offset_c2.fromArray(tar_c2);

        MatOfPoint3f offset_c3 = new MatOfPoint3f();
        offset_c3.fromArray(tar_c3);

        ArrayList<MatOfPoint3f> offset_c = new ArrayList<MatOfPoint3f>();
        offset_c.add(offset_c0);
        offset_c.add(offset_c1);
        offset_c.add(offset_c2);
        offset_c.add(offset_c3);
        return offset_c;

    }
}
