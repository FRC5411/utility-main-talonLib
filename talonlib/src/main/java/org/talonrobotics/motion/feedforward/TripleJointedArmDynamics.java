package talon.motion.feedforward;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
// Uses their code heavily, but adapted for a triple jointed arm

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 * 
 * Used the same formuals for triple jointed arm
 */
public class TripleJointedArmDynamics {
  private static final double g = 9.80665;

  public static class JointConfig {
    private final double mass;
    private final double cgRadius;
    private final double length;
    private final double moi;
    private final DCMotor motor;
    public JointConfig(double mass, double cgRadius, double length, double moi, DCMotor motor) {
        this.mass = mass;
        this.cgRadius = cgRadius;
        this.length = length;
        this.moi = moi;
        this.motor = motor;
    } 
    public double mass() {return mass;}
    public double cgRadius() {return cgRadius;}
    public double length() {return length;}
    public double moi() {return moi;}
    public DCMotor motor() {return motor;}
  }

  private final JointConfig joint_1;
  private final JointConfig joint_2;
  private final JointConfig joint_3;

  public TripleJointedArmDynamics(JointConfig joint_1, JointConfig joint_2, JointConfig joint_3) {
    this.joint_1 = joint_1;
    this.joint_2 = joint_2;
    this.joint_3 = joint_3;
  }

  public Vector<N3> calculate(Vector<N3> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0, 0.0), VecBuilder.fill(0.0, 0.0, 0.0));
  }

  public Vector<N3> calculate(Vector<N3> position, Vector<N3> velocity) {
    return calculate(position, velocity, VecBuilder.fill(0.0, 0.0, 0.0));
  }

  public Vector<N3> calculate(Vector<N3> position, Vector<N3> velocity, Vector<N3> acceleration) {
    var torque = M(position).times(acceleration).
                 plus(C(position, velocity).
                 times(velocity)).plus(Tg(position));

    return VecBuilder.fill(
        joint_1.motor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_2.motor().getVoltage(torque.get(1, 0), velocity.get(1, 0)),
        joint_3.motor().getVoltage(torque.get(2, 0), velocity.get(2, 0)));
  }

  private Matrix<N3, N3> M(Vector<N3> position) {
    var M = new Matrix<>(N3.instance, N3.instance);
    double I1 = joint_1.moi(); // Enter the value of I1
    double l1 = joint_1.length(); // Enter the value of l1
    double m2 = joint_2.mass(); // Enter the value of m2
    double m3 = joint_3.mass(); // Enter the value of m3
    double m1 = joint_1.mass(); // Enter the value of m1
    double r1 = joint_2.cgRadius(); // Enter the value of r1
    double l2 = joint_2.length(); // Enter the value of l2
    double r2 = joint_2.cgRadius(); // Enter the value of r2
    double r3 = joint_3.cgRadius(); // Enter the value of r3
    double I2 = joint_3.moi(); // Enter the value of I2
    double I3 = joint_3.moi(); // Enter the value of I3
    double alpha1 = position.get(0, 0); // Enter the value of alpha1(t)
    double alpha2 = position.get(1, 0); // Enter the value of alpha2(t)
    double alpha3 = position.get(2, 0); // Enter the value of alpha3(t)
    
    double result11 = I1 + Math.pow(l1, 2) * m2 + Math.pow(l1, 2) * m3 + m1 * Math.pow(r1, 2);
    double result12 = l1 * Math.cos(alpha1 - alpha2) * (l2 * m3 + m2 * r2);
    double result13 = l1 * m3 * r3 * Math.cos(alpha1 - alpha3);
    double result21 = l1 * Math.cos(alpha1 - alpha2) * (l2 * m3 + m2 * r2);
    double result22 = m3 * Math.pow(l2, 2) + m2 * Math.pow(r2, 2) + I2;
    double result23 = l2 * m3 * r3 * Math.cos(alpha2 - alpha3);
    double result31 = l1 * m3 * r3 * Math.cos(alpha1 - alpha3);
    double result32 = l2 * m3 * r3 * Math.cos(alpha2 - alpha3);
    double result33 = m3 * Math.pow(r3, 2) + I3;

    M.set(0, 0, result11);
    M.set(0, 1, result12);
    M.set(0, 2, result13);

    M.set(1, 0, result21);
    M.set(1, 1, result22);
    M.set(1, 2, result23);

    M.set(2, 0, result31);
    M.set(2, 1, result32);
    M.set(2, 2, result33);

    return M;
  }

  private Matrix<N3, N3> C(Vector<N3> position, Vector<N3> velocity) {
    var C = new Matrix<>(N3.instance, N3.instance);

    double l1 = joint_1.length(); // Enter the value of l1
    double l2 = joint_2.length(); // Enter the value of l2
    double m2 = joint_2.mass(); // Enter the value of m2
    double m3 = joint_3.mass(); // Enter the value of m3
    double r2 = joint_2.cgRadius(); // Enter the value of r2
    double r3 = joint_3.cgRadius(); // Enter the value of r3
    double alpha1 = position.get(0, 0); // Enter the value of alpha1(t)
    double alpha2 = position.get(1, 0); // Enter the value of alpha2(t)
    double alpha3 = position.get(2, 0); // Enter the value of alpha3(t)
    double alpha1dot = velocity.get(0, 0); // Enter the value of alpha1dot(t)
    double alpha2dot = velocity.get(1, 0); // Enter the value of alpha2dot(t)
    double alpha3dot = velocity.get(2, 0); // Enter the value of alpha3dot(t)

    double result11 = 0;
    double result12 = l1 * Math.sin(alpha1 - alpha2) * alpha2dot * (l2 * m3 + m2 * r2);
    double result13 = l1 * m3 * r3 * Math.sin(alpha1 - alpha3) * alpha3dot;

    double result21 = l1 * Math.sin(alpha2 - alpha1) * alpha1dot * (l2 * m3 + m2 * r2);
    double result22 = 0;
    double result23 = l2 * m3 * r3 * Math.sin(alpha2 - alpha3) * alpha3dot;

    double result31 = (l1 * Math.sin(alpha2 - alpha1) * alpha2dot * (l2 * m3 + m2 * r2)) / 2
            + (l1 * m3 * r3 * Math.sin(alpha3 - alpha1) * alpha1dot) / 2
            + (l1 * m3 * r3 * Math.sin(alpha3 - alpha1) * alpha3dot) / 2;
    double result32 = l2 * m3 * r3 * Math.sin(alpha3 - alpha2) * alpha2dot;
    double result33 = 0;

    C.set(0, 0, result11);
    C.set(0, 1, result12);
    C.set(0, 2, result13);

    C.set(1, 0, result21);
    C.set(1, 1, result22);
    C.set(1, 2, result23);

    C.set(2, 0, result31);
    C.set(2, 1, result32);
    C.set(2, 2, result33);

    return C;
  }

  private Matrix<N3, N1> Tg(Vector<N3> position) {
    var Tg = new Matrix<>(N3.instance, N1.instance);

    double l1 = joint_1.length(); // Enter the value of l1
    double m2 = joint_2.mass(); // Enter the value of m2
    double m3 = joint_3.mass(); // Enter the value of m3
    double m1 = joint_1.mass(); // Enter the value of m1
    double r1 = joint_1.cgRadius(); // Enter the value of r1
    double l2 = joint_2.length(); // Enter the value of l2
    double r2 = joint_2.cgRadius(); // Enter the value of r2
    double r3 = joint_3.cgRadius(); // Enter the value of r3
    double alpha1 = position.get(0, 0); // Enter the value of alpha1(t)
    double alpha2 = position.get(1, 0); // Enter the value of alpha2(t)
    double alpha3 = position.get(2, 0); // Enter the value of alpha3(t)

    double result11 = g * Math.cos(alpha1) * (l1 * m2 + l1 * m3 + m1 * r1);
    double result12 = g * Math.cos(alpha2) * (l2 * m3 + m2 * r2);
    double result13 = g * m3 * r3 * Math.cos(alpha3);

    Tg.set(0, 0, result11);
    Tg.set(1, 0, result12);
    Tg.set(2, 0, result13);

    return Tg;
  }
}