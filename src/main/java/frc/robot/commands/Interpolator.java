package frc.robot.commands;

import java.util.*;

public class Interpolator {

    public enum DataType { HOOD, HOOD_CLOSE, SHOT_TIME }

    private static boolean failed = false;
    public static void failed() {
        failed = true;
    }
    public static boolean hasFailed() {
        return failed;
    }

    //private static final HashMap<Double, Double> hoodValues = new HashMap<>();
    private static final HashMap<Double, Double> hoodCloseValues = new HashMap<>();
    private static final HashMap<Double, Double> shotTimeValues = new HashMap<>();
    public Interpolator() {
        /*hoodValues.put(1.635, 0.51);
        hoodValues.put(2.13,0.525);
        hoodValues.put(2.49,0.535);
        hoodValues.put(3.04,0.545);
        hoodValues.put(3.56,0.56);
        hoodValues.put(4.04,0.575);*/



        hoodCloseValues.put(1.22, 0.49);
        hoodCloseValues.put(1.74,0.51);
        hoodCloseValues.put(2.16,0.525);
        hoodCloseValues.put(2.51,0.535);
        hoodCloseValues.put(2.75,0.545);
        
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        shotTimeValues.put(0.0, 0.0);
        

    }

    // Load both files using File objects



    public static double interpolate(double x, DataType type) {
        if (type==DataType.HOOD) {return (0.0184378*(x*x)) - (0.109559*x) + 0.692893;}
        if (type==DataType.HOOD_CLOSE) {return (.0353235*x)+.447668;};
        HashMap<Double, Double> values = ((type==DataType.HOOD_CLOSE ? hoodCloseValues : shotTimeValues));

        if (values.containsKey(x)) return values.get(x);
        Double lower = null;
        Double upper = null;
        Double a = -1.0;
        for (Double key : values.keySet().stream().sorted().toList()) {
            if (a.equals(-1.0)) {a=key;}
            if (key > x) {
                upper = key;
                lower = a;
                break;
            }
        }

        if (lower != null && upper != null) {
            return linear(x, lower, values.get(lower), upper, values.get(upper));
        }

        // extrapolate using first/last two points
        List<Map.Entry<Double, Double>> pts = new ArrayList<>(values.entrySet());
        if (lower == null) return linear(x, pts.get(0).getKey(), pts.get(0).getValue(),
                                            pts.get(1).getKey(), pts.get(1).getValue());

        int n = pts.size();
        return linear(x, pts.get(n - 2).getKey(), pts.get(n - 2).getValue(),
                            pts.get(n - 1).getKey(), pts.get(n - 1).getValue());
    }

    private static double linear(double x, double x1, double y1, double x2, double y2) {
        double t = (x - x1) / (x2 - x1);
        return (1 - t) * y1 + t * y2;
    }
}