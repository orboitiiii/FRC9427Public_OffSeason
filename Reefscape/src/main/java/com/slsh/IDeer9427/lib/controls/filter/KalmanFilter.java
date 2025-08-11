package com.slsh.IDeer9427.lib.controls.filter;

import com.slsh.IDeer9427.lib.controls.plant.LinearSystem;
import com.slsh.IDeer9427.lib.controls.util.StateSpaceUtil;
import edu.wpi.first.math.jni.DAREJNI;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionCommon_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.linsol.chol.LinearSolverChol_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

/**
 * Implements a Kalman Filter for state estimation in linear systems. Supports discrete-time
 * prediction and correction with noise covariance handling.
 */
public class KalmanFilter {
  public final LinearSystem m_system;
  private final SimpleMatrix m_Q; // Continuous process noise covariance
  private final SimpleMatrix m_R; // Continuous measurement noise covariance

  private final SimpleMatrix m_xhat;
  private SimpleMatrix m_P;
  private final SimpleMatrix m_initP;

  private final SimpleMatrix m_discA;
  private final SimpleMatrix m_discB;
  private final SimpleMatrix m_discQ;
  private final SimpleMatrix m_discR;
  private final SimpleMatrix m_M_temp;
  private final SimpleMatrix m_phi_temp;
  private final SimpleMatrix m_V_temp;
  private final SimpleMatrix m_phiQ_temp;
  private final SimpleMatrix m_integral_temp;
  private final SimpleMatrix m_expA_temp;
  private final SimpleMatrix m_expAT_temp;
  private double m_lastDt = -1.0;

  private final SimpleMatrix m_temp_predict_x1;
  private final SimpleMatrix m_temp_predict_x2;
  private final SimpleMatrix m_temp_predict_p1;

  private final SimpleMatrix m_C;
  private final SimpleMatrix m_D;
  private final SimpleMatrix m_temp_correct_p1;
  private final SimpleMatrix m_temp_correct_p2;
  private final SimpleMatrix m_S; // C P C^T + R
  private final SimpleMatrix m_S_inv;
  private final SimpleMatrix m_K;
  private final SimpleMatrix m_temp_correct_y;
  private final SimpleMatrix m_I;
  private final SimpleMatrix m_P_backup;
  private final SimpleMatrix m_temp_IKC;
  private final SimpleMatrix m_temp_joseph;
  private final SimpleMatrix m_temp_KR;

  /**
   * Constructs a Kalman Filter instance. Discretizes system matrices and computes steady-state
   * covariance using DARE.
   *
   * @param system The linear system model.
   * @param Q_cont Continuous process noise covariance.
   * @param R_cont Continuous measurement noise covariance.
   * @param nominalDtSeconds Nominal time step for initial discretization.
   */
  public KalmanFilter(
      LinearSystem system, SimpleMatrix Q_cont, SimpleMatrix R_cont, double nominalDtSeconds) {
    this.m_system = system;
    this.m_Q = Q_cont;
    this.m_R = R_cont;

    int states = system.getNumStates();
    int inputs = system.getNumInputs();
    int outputs = system.getNumOutputs();
    int M_size = states + inputs;
    int V_size = 2 * states;

    m_xhat = new SimpleMatrix(states, 1);
    m_initP = new SimpleMatrix(states, states);
    m_P = new SimpleMatrix(states, states);
    m_C = system.getC();
    m_D = system.getD();

    m_discA = new SimpleMatrix(states, states);
    m_discB = new SimpleMatrix(states, inputs);
    m_discQ = new SimpleMatrix(states, states);
    m_discR = new SimpleMatrix(outputs, outputs);
    m_M_temp = new SimpleMatrix(M_size, M_size);
    m_phi_temp = new SimpleMatrix(M_size, M_size);
    m_V_temp = new SimpleMatrix(V_size, V_size);
    m_phiQ_temp = new SimpleMatrix(V_size, V_size);
    m_integral_temp = new SimpleMatrix(states, states);
    m_expA_temp = new SimpleMatrix(states, states);
    m_expAT_temp = new SimpleMatrix(states, states);

    m_temp_predict_x1 = new SimpleMatrix(states, 1);
    m_temp_predict_x2 = new SimpleMatrix(states, 1);
    m_temp_predict_p1 = new SimpleMatrix(states, states);

    m_temp_correct_p1 = new SimpleMatrix(states, outputs);
    m_temp_correct_p2 = new SimpleMatrix(outputs, states);
    m_S = new SimpleMatrix(outputs, outputs);
    m_S_inv = new SimpleMatrix(outputs, outputs);
    m_K = new SimpleMatrix(states, outputs);
    m_temp_correct_y = new SimpleMatrix(outputs, 1);
    m_I = SimpleMatrix.identity(states);
    m_P_backup = new SimpleMatrix(states, states);
    m_temp_IKC = new SimpleMatrix(states, states);
    m_temp_joseph = new SimpleMatrix(states, states);
    m_temp_KR = new SimpleMatrix(states, outputs);

    // Compute initial discrete matrices using nominal dt
    StateSpaceUtil.discretizeAB(
        m_system.getA(), m_system.getB(), nominalDtSeconds, m_discA, m_discB, m_M_temp, m_phi_temp);
    StateSpaceUtil.discretizeQ(
        m_system.getA(),
        m_Q,
        nominalDtSeconds,
        m_discQ,
        m_V_temp,
        m_phiQ_temp,
        m_integral_temp,
        m_expA_temp,
        m_expAT_temp);
    StateSpaceUtil.discretizeR(m_R, nominalDtSeconds, m_discR);

    // Compute steady-state P using DARE for observer
    SimpleMatrix discAT = m_discA.transpose();
    SimpleMatrix CT = m_C.transpose();
    DAREJNI.dareABQR(
        discAT.getDDRM().getData(),
        CT.getDDRM().getData(),
        m_discQ.getDDRM().getData(),
        m_discR.getDDRM().getData(),
        states,
        outputs,
        m_initP.getDDRM().getData());

    reset();
  }

  /** Resets the filter state and covariance to initial values. */
  public void reset() {
    m_xhat.zero();
    System.arraycopy(
        m_initP.getDDRM().getData(), 0, m_P.getDDRM().getData(), 0, m_initP.getNumElements());
  }

  /**
   * Predicts the next state and covariance. x^- = A x + B u P^- = A P A^T + Q Discretizes matrices
   * if dt changes. Enforces symmetry on P to mitigate floating-point errors.
   *
   * @param u Input vector.
   * @param dtSeconds Time step.
   */
  public void predict(SimpleMatrix u, double dtSeconds) {
    if (dtSeconds != m_lastDt) {
      StateSpaceUtil.discretizeAB(
          m_system.getA(), m_system.getB(), dtSeconds, m_discA, m_discB, m_M_temp, m_phi_temp);
      StateSpaceUtil.discretizeQ(
          m_system.getA(),
          m_Q,
          dtSeconds,
          m_discQ,
          m_V_temp,
          m_phiQ_temp,
          m_integral_temp,
          m_expA_temp,
          m_expAT_temp);
      StateSpaceUtil.discretizeR(m_R, dtSeconds, m_discR);
      m_lastDt = dtSeconds;
    }

    DMatrixRMaj A = m_discA.getDDRM();
    DMatrixRMaj B = m_discB.getDDRM();
    DMatrixRMaj xhat = m_xhat.getDDRM();
    DMatrixRMaj u_ddrm = u.getDDRM();
    DMatrixRMaj P = m_P.getDDRM();
    DMatrixRMaj Q = m_discQ.getDDRM();

    // x^- = A x + B u
    CommonOps_DDRM.mult(A, xhat, m_temp_predict_x1.getDDRM());
    CommonOps_DDRM.mult(B, u_ddrm, m_temp_predict_x2.getDDRM());
    CommonOps_DDRM.add(m_temp_predict_x1.getDDRM(), m_temp_predict_x2.getDDRM(), xhat);

    // P^- = A P A^T + Q
    CommonOps_DDRM.mult(A, P, m_temp_predict_p1.getDDRM());
    System.arraycopy(Q.getData(), 0, P.getData(), 0, Q.getNumElements());
    CommonOps_DDRM.multAddTransB(m_temp_predict_p1.getDDRM(), A, P);

    // Enforce symmetry on P: P = (P + P^T)/2 to mitigate floating-point errors
    CommonOps_DDRM.transpose(P, m_temp_predict_p1.getDDRM());
    CommonOps_DDRM.add(P, m_temp_predict_p1.getDDRM(), P);
    CommonOps_DDRM.scale(0.5, P);
  }

  /**
   * Corrects the state estimate using measurement. K = P C^T (C P C^T + R)^{-1} x = x^- + K (y - C
   * x^- - D u) P = (I - K C) P^- (I - K C)^T + K R K^T (Joseph form for numerical stability)
   *
   * @param u Input vector.
   * @param y Measurement vector.
   */
  public void correct(SimpleMatrix u, SimpleMatrix y) {
    DMatrixRMaj C = m_C.getDDRM();
    DMatrixRMaj D = m_D.getDDRM();
    DMatrixRMaj P = m_P.getDDRM();
    DMatrixRMaj R = m_discR.getDDRM();
    DMatrixRMaj xhat = m_xhat.getDDRM();
    DMatrixRMaj u_ddrm = u.getDDRM();
    DMatrixRMaj y_ddrm = y.getDDRM();

    DMatrixRMaj K = m_K.getDDRM();
    DMatrixRMaj S = m_S.getDDRM();
    DMatrixRMaj S_inv = m_S_inv.getDDRM();
    DMatrixRMaj temp_p1 = m_temp_correct_p1.getDDRM();
    DMatrixRMaj temp_p2 = m_temp_correct_p2.getDDRM();
    DMatrixRMaj temp_y = m_temp_correct_y.getDDRM();

    // S = C P C^T + R
    CommonOps_DDRM.multTransB(P, C, temp_p1);
    CommonOps_DDRM.mult(C, P, temp_p2);
    System.arraycopy(R.getData(), 0, S.getData(), 0, R.getNumElements());
    CommonOps_DDRM.multAddTransB(temp_p2, C, S);

    // S_inv = S^{-1} using Cholesky
    CholeskyDecomposition_F64<DMatrixRMaj> chol = DecompositionFactory_DDRM.chol(S.numRows, true);
    LinearSolverChol_DDRM cholSolver =
        new LinearSolverChol_DDRM((CholeskyDecompositionCommon_DDRM) chol);

    // Set the matrix for decomposition and check success
    if (!cholSolver.setA(S)) {
      throw new RuntimeException("Cholesky decomposition failed; S may not be positive definite");
    }

    if (CommonOps_DDRM.det(S) <= 0) {
      throw new RuntimeException("S not positive definite");
    }

    cholSolver.invert(S_inv);

    // K = (P C^T) S_inv
    CommonOps_DDRM.mult(temp_p1, S_inv, K);

    // x = x^- + K (y - C x^- - D u)
    CommonOps_DDRM.mult(C, xhat, temp_y);
    CommonOps_DDRM.multAdd(D, u_ddrm, temp_y);
    CommonOps_DDRM.subtract(y_ddrm, temp_y, temp_y);
    CommonOps_DDRM.multAdd(K, temp_y, xhat);

    // Joseph form: P = (I - K C) P^- (I - K C)^T + K R K^T
    // Backup P^- (current P is P^-)
    System.arraycopy(P.getData(), 0, m_P_backup.getDDRM().getData(), 0, P.getNumElements());

    // temp = K C
    CommonOps_DDRM.mult(K, C, m_temp_joseph.getDDRM());

    // I - K C -> m_temp_IKC
    CommonOps_DDRM.subtract(m_I.getDDRM(), m_temp_joseph.getDDRM(), m_temp_IKC.getDDRM());

    // temp = (I - K C) P^-
    CommonOps_DDRM.mult(m_temp_IKC.getDDRM(), m_P_backup.getDDRM(), m_temp_joseph.getDDRM());

    // P = temp * (I - K C)^T
    CommonOps_DDRM.multTransB(m_temp_joseph.getDDRM(), m_temp_IKC.getDDRM(), P);

    // temp_KR = K R
    CommonOps_DDRM.mult(K, R, m_temp_KR.getDDRM());

    // P += (K R) K^T
    CommonOps_DDRM.multAddTransB(m_temp_KR.getDDRM(), K, P);

    // Enforce symmetry on updated P: P = (P + P^T)/2 to mitigate floating-point errors
    CommonOps_DDRM.transpose(P, m_temp_joseph.getDDRM());
    CommonOps_DDRM.add(P, m_temp_joseph.getDDRM(), P);
    CommonOps_DDRM.scale(0.5, P);
  }

  public SimpleMatrix getXhat() {
    return m_xhat;
  }

  public double getXhat(int row) {
    return m_xhat.get(row, 0);
  }

  public void setXhat(SimpleMatrix xhat) {
    System.arraycopy(
        xhat.getDDRM().getData(), 0, this.m_xhat.getDDRM().getData(), 0, xhat.getNumElements());
  }

  public void setXhat(int row, double val) {
    this.m_xhat.set(row, 0, val);
  }
}
