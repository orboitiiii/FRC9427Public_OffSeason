package Control.LQR;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.Discretization;

/**
 * A plant defined using state-space notation.
 *
 * <p>A plant is a mathematical model of a system's dynamics.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 *
 * @param <States> Number of states.
 * @param <Inputs> Number of inputs.
 * @param <Outputs> Number of outputs.
 */
public class StateSpaceSystem<States extends Num, Inputs extends Num, Outputs extends Num> {
  /** Continuous system matrix. */
  private Matrix<States, States> m_A;

  /** Continuous input matrix. */
  private Matrix<States, Inputs> m_B;

  /** Output matrix. */
  private final Matrix<Outputs, States> m_C;

  /** Feedthrough matrix. */
  private final Matrix<Outputs, Inputs> m_D;

  /**
   * Construct a new LinearSystem from the four system matrices.
   *
   * @param A The system matrix A.
   * @param B The input matrix B.
   * @param C The output matrix C.
   * @param D The feedthrough matrix D.
   * @throws IllegalArgumentException if any matrix element isn't finite.
   */
  public StateSpaceSystem(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<Outputs, States> C,
      Matrix<Outputs, Inputs> D) {
    validateMatrix(A, "A");
    validateMatrix(B, "B");
    validateMatrix(C, "C");
    validateMatrix(D, "D");

    this.m_A = A;
    this.m_B = B;
    this.m_C = C;
    this.m_D = D;
  }

  /** Validates that all elements in the matrix are finite. */
  private void validateMatrix(Matrix<?, ?> matrix, String name) {
    for (int row = 0; row < matrix.getNumRows(); ++row) {
      for (int col = 0; col < matrix.getNumCols(); ++col) {
        if (!Double.isFinite(matrix.get(row, col))) {
          throw new IllegalArgumentException(
              "Elements of " + name + " aren't finite. Check the model implementation.");
        }
      }
    }
  }

  /**
   * Returns the system matrix A.
   *
   * @return the system matrix A.
   */
  public Matrix<States, States> getA() {
    return m_A;
  }

  /**
   * Sets a new system matrix A.
   *
   * @param A The updated system matrix A.
   */
  public void setA(Matrix<States, States> A) {
    validateMatrix(A, "A");
    this.m_A = A;
  }

  /**
   * Returns the input matrix B.
   *
   * @return the input matrix B.
   */
  public Matrix<States, Inputs> getB() {
    return m_B;
  }

  /**
   * Sets a new input matrix B.
   *
   * @param B The updated input matrix B.
   */
  public void setB(Matrix<States, Inputs> B) {
    validateMatrix(B, "B");
    this.m_B = B;
  }

  /**
   * Returns the output matrix C.
   *
   * @return Output matrix C.
   */
  public Matrix<Outputs, States> getC() {
    return m_C;
  }

  /**
   * Returns the feedthrough matrix D.
   *
   * @return the feedthrough matrix D.
   */
  public Matrix<Outputs, Inputs> getD() {
    return m_D;
  }

  /**
   * Computes the new x given the old x and the control input.
   *
   * <p>This is used by state observers directly to run updates based on state estimate.
   *
   * @param x The current state.
   * @param clampedU The control input.
   * @param dtSeconds Timestep for model update.
   * @return the updated x.
   */
  public Matrix<States, N1> calculateX(
      Matrix<States, N1> x, Matrix<Inputs, N1> clampedU, double dtSeconds) {
    var discABpair = Discretization.discretizeAB(m_A, m_B, dtSeconds);

    return discABpair.getFirst().times(x).plus(discABpair.getSecond().times(clampedU));
  }

  /**
   * Computes the new y given the control input.
   *
   * <p>This is used by state observers directly to run updates based on state estimate.
   *
   * @param x The current state.
   * @param clampedU The control input.
   * @return the updated output matrix Y.
   */
  public Matrix<Outputs, N1> calculateY(Matrix<States, N1> x, Matrix<Inputs, N1> clampedU) {
    return m_C.times(x).plus(m_D.times(clampedU));
  }

  @Override
  public String toString() {
    return String.format(
        "Linear System: A\n%s\n\nB:\n%s\n\nC:\n%s\n\nD:\n%s\n",
        m_A.toString(), m_B.toString(), m_C.toString(), m_D.toString());
  }
}
