package com.slsh.IDeer9427.lib.data;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class IDearLog {
  private static IDearLog instance;

  public static IDearLog getInstance() {
    if (instance == null) instance = new IDearLog();
    return instance;
  }

  public enum FieldType {
    CAN,
    NON_CAN
  }

  private final NetworkTable table;
  private final DoublePublisher numFieldsPub;

  private final Map<String, DoublePublisher> publishers = new HashMap<>();
  private final Map<String, Supplier<Double>> canSuppliers = new HashMap<>();
  private final Map<String, Supplier<Double>> nonCanSuppliers = new HashMap<>();
  private final Map<String, Double> cache = new HashMap<>();

  private final Map<String, Double> lastSent = new HashMap<>();
  private final Map<String, Long> lastTsMs = new HashMap<>();

  private final Object lock = new Object();
  private Notifier slowNotifier;
  private Notifier fastNotifier;

  private volatile double eps = 1e-3;
  private volatile long maxStaleMs = 200;

  private IDearLog() {
    table = NetworkTableInstance.getDefault().getTable("CustomLogger");
    numFieldsPub = table.getSubTable("Meta").getDoubleTopic("NumFields").publish();
    numFieldsPub.setDefault(0.0);
  }

  public void setChangeEpsilon(double epsilon) {
    this.eps = Math.max(0.0, epsilon);
  }

  public void setMaxStaleMs(long ms) {
    this.maxStaleMs = Math.max(0, ms);
  }

  public void addField(String name, Supplier<Double> supplier, FieldType type) {
    synchronized (lock) {
      if (publishers.containsKey(name)) return;
      DoublePublisher pub = table.getDoubleTopic(name).publish();
      pub.setDefault(Double.NaN);
      publishers.put(name, pub);
      cache.put(name, Double.NaN);
      if (type == FieldType.CAN) canSuppliers.put(name, supplier);
      else nonCanSuppliers.put(name, supplier);
      numFieldsPub.set(publishers.size());
    }
  }

  public void startUpdaters(int slowPeriodMs, int fastPeriodMs) {
    stopUpdaters();
    if (!canSuppliers.isEmpty()) {
      slowNotifier = new Notifier(new UpdaterRunnable(canSuppliers));
      slowNotifier.startPeriodic(slowPeriodMs / 1000.0);
    }
    if (!nonCanSuppliers.isEmpty()) {
      fastNotifier = new Notifier(new UpdaterRunnable(nonCanSuppliers));
      fastNotifier.startPeriodic(fastPeriodMs / 1000.0);
    }
  }

  public void stopUpdaters() {
    if (slowNotifier != null) {
      slowNotifier.stop();
      slowNotifier.close();
      slowNotifier = null;
    }
    if (fastNotifier != null) {
      fastNotifier.stop();
      fastNotifier.close();
      fastNotifier = null;
    }
  }

  public void logData() {
    Map<String, Double> snap;
    synchronized (lock) {
      snap = new HashMap<>(cache);
    }
    long now = System.currentTimeMillis();

    for (var e : snap.entrySet()) {
      String k = e.getKey();
      double v = e.getValue();
      Double prev = lastSent.get(k);
      long ts = lastTsMs.getOrDefault(k, 0L);

      boolean changed = (prev == null) || !bothFiniteAndClose(prev, v, eps);
      boolean stale = now - ts >= maxStaleMs;

      if (changed || stale) {
        DoublePublisher pub = publishers.get(k);
        if (pub != null) {
          pub.set(v);
          lastSent.put(k, v);
          lastTsMs.put(k, now);
        }
      }
    }
    numFieldsPub.set(publishers.size());
  }

  private static boolean bothFiniteAndClose(double a, double b, double tol) {
    if (!Double.isFinite(a) || !Double.isFinite(b)) return false;
    return Math.abs(a - b) <= tol;
  }

  private class UpdaterRunnable implements Runnable {
    private final Map<String, Supplier<Double>> suppliersRef;

    UpdaterRunnable(Map<String, Supplier<Double>> suppliers) {
      this.suppliersRef = suppliers;
    }

    @Override
    public void run() {
      Map<String, Supplier<Double>> snap;
      synchronized (lock) {
        snap = new HashMap<>(suppliersRef);
      }

      Map<String, Double> newVals = new HashMap<>(snap.size());
      for (var e : snap.entrySet()) {
        double v;
        try {
          v = e.getValue().get();
          if (!Double.isFinite(v)) v = Double.NaN;
        } catch (Exception ex) {
          v = Double.NaN;
        }
        newVals.put(e.getKey(), v);
      }
      synchronized (lock) {
        cache.putAll(newVals);
      }
    }
  }
}
