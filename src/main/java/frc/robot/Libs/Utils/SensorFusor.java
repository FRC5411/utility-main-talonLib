package frc.robot.Libs.Utils;

import java.nio.channels.UnsupportedAddressTypeException;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.KalmanFilter;

public class SensorFusor {
    public static HashMap<String, DoubleSupplier> positionSuppliers;
    public static HashMap<String, DoubleSupplier> velocitySuppliers;

    public String[] sensorList;

    public SensorFusor(String[] sensorList) {
        this.sensorList = sensorList;
    }

    public void update() {
        double[] measurements = new double[sensorList.length];
        for (int i = 0; i < sensorList.length; i++) {
            measurements[i] = positionSuppliers.get(sensorList[i]).getAsDouble();
        }
    }
}
