package jp.jaxa.iss.kibo.rpc.sampleapk;

public class Box {
    double x_min, y_min, z_min, x_max, y_max, z_max;
    public Box(double x_min, double y_min, double z_min, double x_max, double y_max, double z_max) {
        this.x_min = x_min;
        this.y_min = y_min;
        this.z_min = z_min;
        this.x_max = x_max;
        this.y_max = y_max;
        this.z_max = z_max;
    }
}
