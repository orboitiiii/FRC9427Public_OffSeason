package com.slsh.IDeer9427.lib.controls.LinearController;

import com.slsh.IDeer9427.lib.controls.plant.LinearSystem;
import com.slsh.IDeer9427.lib.controls.util.StateSpaceUtil;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;

/**
 * Plant-inversion feedforward.
 *
 * <p>The feedforward law is u_ff = B⁺ (r_k+1 - A * r_k), where B⁺ is the pseudoinverse of B.
 *
 * <p>The constructor and `calculate` method of this class execute with almost no memory allocation.
 */
public class LinearPlantInversionFeedforward {
  // Discretized A matrix
  private final SimpleMatrix m_A;
  // Precomputed pseudoinverse of B (B⁺)
  private final SimpleMatrix m_B_pinv;

  // Reference state vector
  private SimpleMatrix m_r;
  // Feedforward output vector
  private final SimpleMatrix m_uff;

  // Pre-allocated temporary matrix for state term computation
  private final SimpleMatrix m_temp_state_term;

  /**
   * Constructs a feedforward controller.
   *
   * @param system The linear system to control
   * @param dtSeconds The discretization time step
   */
  public LinearPlantInversionFeedforward(LinearSystem system, double dtSeconds) {
    int states = system.getNumStates();
    int inputs = system.getNumInputs();

    // Pre-allocate all necessary matrices for initialization and the control loop
    m_A = new SimpleMatrix(states, states);
    SimpleMatrix B = new SimpleMatrix(states, inputs); // Temporary B for discretization
    SimpleMatrix M_temp = new SimpleMatrix(states + inputs, states + inputs);
    SimpleMatrix phi_temp = new SimpleMatrix(states + inputs, states + inputs);

    // Required for the control loop
    m_r = new SimpleMatrix(states, 1);
    m_uff = new SimpleMatrix(inputs, 1);
    m_temp_state_term = new SimpleMatrix(states, 1);

    // Discretize A and B
    StateSpaceUtil.discretizeAB(
        system.getA(),
        system.getB(),
        dtSeconds,
        m_A,
        B, // output to temporary B
        M_temp,
        phi_temp // temporary
        );

    // Check B condition number once using SVD (moved from calculate for efficiency)
    SingularValueDecomposition_F64<DMatrixRMaj> svd =
        DecompositionFactory_DDRM.svd(states, inputs, false, false, true);
    svd.decompose(B.getDDRM().copy());
    double[] sv = svd.getSingularValues();
    if (sv[sv.length - 1] < 1e-12) {
      throw new RuntimeException("B singular");
    }
    double cond = sv[0] / sv[sv.length - 1];
    if (cond > 1e10) {
      throw new RuntimeException("B ill-conditioned");
    }

    // Precompute B pseudoinverse (B⁺) using solver
    LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.pseudoInverse(true);
    if (!solver.setA(B.getDDRM())) {
      throw new RuntimeException("Unable to compute pseudoinverse of B");
    }
    m_B_pinv = new SimpleMatrix(inputs, states);
    // Compute B⁺ as a matrix: solve B X = I for X (pseudoinverse)
    DMatrixRMaj identity = CommonOps_DDRM.identity(states);
    DMatrixRMaj pinv_data = m_B_pinv.getDDRM();
    solver.solve(identity, pinv_data); // Now m_B_pinv = B^+

    reset();
  }

  /** Resets the reference and output. */
  public void reset() {
    this.m_r.zero();
    this.m_uff.zero();
  }

  public void reset(SimpleMatrix initialState) {
    if (initialState.getNumRows() != m_r.getNumRows() || initialState.getNumCols() != 1) {
      throw new IllegalArgumentException(
          "Initial state dimension mismatch; expected " + m_r.getNumRows() + "x1");
    }
    System.arraycopy(
        initialState.getDDRM().getData(),
        0,
        this.m_r.getDDRM().getData(),
        0,
        initialState.getNumElements());
    this.m_uff.zero();
  }

  /**
   * Calculates the feedforward output in a memory-allocation-free manner. u_ff = B⁺(r_k+1 − Ar_k)
   *
   * @param r The current reference state r_k
   * @param nextR The next reference state r_k+1
   */
  public void calculate(SimpleMatrix r, SimpleMatrix nextR) {
    // Get the underlying DMatrixRMaj
    DMatrixRMaj A_ddrm = m_A.getDDRM();
    DMatrixRMaj r_ddrm = r.getDDRM();
    DMatrixRMaj nextR_ddrm = nextR.getDDRM();
    DMatrixRMaj temp_ddrm = m_temp_state_term.getDDRM();
    DMatrixRMaj uff_ddrm = m_uff.getDDRM();
    DMatrixRMaj B_pinv_ddrm = m_B_pinv.getDDRM();

    // Calculate A * r_k, store in temp
    CommonOps_DDRM.mult(A_ddrm, r_ddrm, temp_ddrm);

    // Calculate r_k+1 - (A * r_k), store back in temp
    CommonOps_DDRM.subtract(nextR_ddrm, temp_ddrm, temp_ddrm);

    // Calculate u_ff = B⁺ * temp
    CommonOps_DDRM.mult(B_pinv_ddrm, temp_ddrm, uff_ddrm);

    // Update the current reference
    System.arraycopy(
        nextR.getDDRM().getData(), 0, this.m_r.getDDRM().getData(), 0, nextR.getNumElements());
  }

  public SimpleMatrix getUff() {
    return m_uff;
  }

  public SimpleMatrix getR() {
    return m_r;
  }
}
