package talon.feedforward;
// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 */
public class DoubleJointedArmDynamics {
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

  public DoubleJointedArmDynamics(JointConfig joint_1, JointConfig joint_2) {
    this.joint_1 = joint_1;
    this.joint_2 = joint_2;
  }

  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity) {
        return calculate(position, velocity, VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var torque = M(position).times(acceleration).
                 plus(C(position, velocity).
                 times(velocity)).plus(Tg(position));

    return VecBuilder.fill(
        joint_1.motor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_2.motor().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }

  private Matrix<N2, N2> M(Vector<N2> position) {
    var M = new Matrix<>(N2.instance, N2.instance);

    M.set(
        0,
        0,
        joint_1.mass() * Math.pow(joint_1.cgRadius(), 2.0)
            + joint_2.mass() * (Math.pow(joint_1.length(), 2.0) + Math.pow(joint_2.cgRadius(), 2.0))
            + joint_1.moi()
            + joint_2.moi()
            + 2
                * joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0)
            + joint_2.moi()
            + joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0)
            + joint_2.moi()
            + joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(1, 1, joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0) + joint_2.moi());

    return M;
  }

  private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
    var C = new Matrix<>(N2.instance, N2.instance);

    C.set(
        0,
        0,
        -joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));

    return C;
  }

  private Matrix<N2, N1> Tg(Vector<N2> position) {
    var Tg = new Matrix<>(N2.instance, N1.instance);

    Tg.set(
        0,
        0,
        (joint_1.mass() * joint_1.cgRadius() + joint_2.mass() * joint_1.length())
                * g
                * Math.cos(position.get(0, 0))
            + joint_2.mass()
                * joint_2.cgRadius()
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        joint_2.mass()
            * joint_2.cgRadius()
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    return Tg;
  }
}