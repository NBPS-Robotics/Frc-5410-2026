package frc.robot.commands;

import java.io.*;
import java.nio.file.*;
import java.util.*;

public class Interpolator {

    public enum DataType { HOOD, SHOT_TIME }

    private static boolean failed = false;
    public static void failed() {
        failed = true;
    }
    public static boolean hasFailed() {
        return failed;
    }

    private static final TreeMap<Double, Double> hoodValues = new TreeMap<>();
    private static final TreeMap<Double, Double> shotTimeValues = new TreeMap<>();

    // Load both files using File objects
    public static void loadFiles(File hoodFile, File shotTimeFile) throws IOException {
        loadIntoMap(hoodFile, hoodValues);
        loadIntoMap(shotTimeFile, shotTimeValues);
    }

    private static void loadIntoMap(File file, TreeMap<Double, Double> map) throws IOException {
        for (String line : Files.readAllLines(file.toPath())) {
            if (line.isBlank()) continue;
            String[] p = line.trim().split("\\s+");
            map.put(Double.parseDouble(p[0]), Double.parseDouble(p[1]));
        }
    }

    public static double interpolate(double x, DataType type) {
        TreeMap<Double, Double> values = (type == DataType.HOOD) ? hoodValues : shotTimeValues;

        if (values.containsKey(x)) return values.get(x);

        Double lower = values.floorKey(x);
        Double upper = values.ceilingKey(x);

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