package Control.LQR;

import Control.Filter.ExtendedKalmanFilter;
import Control.Filter.KalmanFilter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import java.util.function.Function;
import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;

/**
 * Combines a controller, feedforward, and observer for full state feedback control.
 *
 * <p>All "inputs" and "outputs" are defined from the plant's perspective. This means that U is an
 * input to the plant and Y is an output from the plant, which is opposite to the controller's view.
 *
 * <p>For further mathematical background, refer to:
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf
 *
 * @param <States> Number of states.
 * @param <Inputs> Number of inputs.
 * @param <Outputs> Number of outputs.
 */
public class StateSpaceControlLoop<States extends Num, Inputs extends Num, Outputs extends Num> {
  private final LQRController<States, Inputs, Outputs> m_controller;
  private final LinearPlantInversionFeedforward<States, Inputs, Outputs> m_feedforward;
  private final KalmanFilter<States, Inputs, Outputs> m_observer;
  private final ExtendedKalmanFilter<States, Inputs, Outputs> m_extendedObserver;
  private boolean useExtendedKalmanFilter = false;
  private Matrix<States, N1> m_nextR;
  private Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> m_clampFunction;

  /**
   * Constructs a state-space loop with the given plant, controller, and standard observer. By
   * default, the initial reference is all zeros. Users should call reset with the initial system
   * state before enabling the loop. This constructor assumes the inputs are voltages.
   *
   * @param plant State-space plant.
   * @param controller State-space controller.
   * @param observer Standard state-space observer.
   * @param maxVoltageVolts Maximum allowable voltage (e.g., 12V).
   * @param dtSeconds Nominal timestep in seconds.
   */
  public StateSpaceControlLoop(
      StateSpaceSystem<States, Inputs, Outputs> plant,
      LQRController<States, Inputs, Outputs> controller,
      KalmanFilter<States, Inputs, Outputs> observer,
      double maxVoltageVolts,
      double dtSeconds) {
    this(
        controller,
        new LinearPlantInversionFeedforward<>(plant, dtSeconds),
        observer,
        u -> StateSpaceUtil.desaturateInputVector(u, maxVoltageVolts));
  }

  /**
   * Constructs a state-space loop with the given plant, controller, and standard observer.
   *
   * @param plant State-space plant.
   * @param controller State-space controller.
   * @param observer Standard state-space observer.
   * @param clampFunction Function to clamp the input U.
   * @param dtSeconds Nominal timestep in seconds.
   */
  public StateSpaceControlLoop(
      StateSpaceSystem<States, Inputs, Outputs> plant,
      LQRController<States, Inputs, Outputs> controller,
      KalmanFilter<States, Inputs, Outputs> observer,
      Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction,
      double dtSeconds) {
    this(
        controller,
        new LinearPlantInversionFeedforward<>(plant, dtSeconds),
        observer,
        clampFunction);
  }

  /**
   * Constructs a state-space loop with the given controller, feedforward, and standard observer. By
   * default, the initial reference is all zeros. Users should call reset with the initial system
   * state before enabling the loop.
   *
   * @param controller State-space controller.
   * @param feedforward Plant inversion feedforward.
   * @param observer Standard state-space observer.
   * @param maxVoltageVolts Maximum allowable voltage (assumes inputs are voltages).
   */
  public StateSpaceControlLoop(
      LQRController<States, Inputs, Outputs> controller,
      LinearPlantInversionFeedforward<States, Inputs, Outputs> feedforward,
      KalmanFilter<States, Inputs, Outputs> observer,
      double maxVoltageVolts) {
    this(
        controller,
        feedforward,
        observer,
        u -> StateSpaceUtil.desaturateInputVector(u, maxVoltageVolts));
  }

  /**
   * Constructs a state-space loop with the given controller, feedforward, and standard observer. By
   * default, the initial reference is all zeros. Users should call reset with the initial system
   * state before enabling the loop.
   *
   * @param controller State-space controller.
   * @param feedforward Plant inversion feedforward.
   * @param observer Standard state-space observer.
   * @param clampFunction Function to clamp the input U.
   */
  public StateSpaceControlLoop(
      LQRController<States, Inputs, Outputs> controller,
      LinearPlantInversionFeedforward<States, Inputs, Outputs> feedforward,
      KalmanFilter<States, Inputs, Outputs> observer,
      Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction) {
    this.m_controller = controller;
    this.m_feedforward = feedforward;
    this.m_observer = observer;
    this.m_extendedObserver = null; // Not used in this constructor
    this.m_clampFunction = clampFunction;

    // Initialize the next reference vector with dimension equal to number of states.
    this.m_nextR = new Matrix<>(new SimpleMatrix(controller.getK().getNumCols(), 1));
    reset(m_nextR);
  }

  /**
   * Constructs a state-space loop with the given controller, feedforward, and extended observer.
   * This constructor supports nonlinear dynamics.
   *
   * @param controller State-space controller.
   * @param feedforward Plant inversion feedforward.
   * @param extendedObserver Extended Kalman Filter observer.
   * @param clampFunction Function to clamp the input U.
   */
  public StateSpaceControlLoop(
      LQRController<States, Inputs, Outputs> controller,
      LinearPlantInversionFeedforward<States, Inputs, Outputs> feedforward,
      ExtendedKalmanFilter<States, Inputs, Outputs> extendedObserver,
      Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction) {
    this.m_controller = controller;
    this.m_feedforward = feedforward;
    this.m_observer = null; // Not used in this constructor
    this.m_extendedObserver = extendedObserver;
    this.m_clampFunction = clampFunction;
    this.useExtendedKalmanFilter = true;

    // Initialize the next reference vector with dimension equal to number of states.
    this.m_nextR = new Matrix<>(new SimpleMatrix(controller.getK().getNumCols(), 1));
    reset(m_nextR);
  }

  /**
   * Returns the current state estimate (x̂) from the active observer.
   *
   * @return The state estimate x̂.
   */
  public Matrix<States, N1> getXHat() {
    if (useExtendedKalmanFilter) {
      return m_extendedObserver.getXhat();
    } else {
      return m_observer.getXhat();
    }
  }

  /**
   * Returns the total control input (u) calculated by the controller plus feedforward, after
   * clamping.
   *
   * @return The clamped control input u.
   */
  public Matrix<Inputs, N1> getU() {
    return clampInput(m_controller.getU().plus(m_feedforward.getUff()));
  }

  /**
   * Returns an element of the control input (u) at the specified row.
   *
   * @param row Row index of u.
   * @return The control input value at the specified row.
   */
  public double getU(int row) {
    return getU().get(row, 0);
  }

  /**
   * Sets the next reference vector (r).
   *
   * @param nextR Varargs representing the next reference vector.
   */
  public void setNextR(double... nextR) {
    if (nextR.length != m_nextR.getNumRows()) {
      throw new MatrixDimensionException(
          String.format(
              "The next reference does not have the correct number of entries! Expected %s, but got"
                  + " %s.",
              m_nextR.getNumRows(), nextR.length));
    }
    m_nextR = new Matrix<>(new SimpleMatrix(m_nextR.getNumRows(), 1, true, nextR));
  }

  /**
   * Returns the internally used LQR controller.
   *
   * @return The LQR controller.
   */
  public LQRController<States, Inputs, Outputs> getController() {
    return m_controller;
  }

  /**
   * Returns the internally used feedforward component.
   *
   * @return The feedforward component.
   */
  public LinearPlantInversionFeedforward<States, Inputs, Outputs> getFeedforward() {
    return m_feedforward;
  }

  /**
   * Returns the state error (r - x̂).
   *
   * @return The state error matrix.
   */
  public Matrix<States, N1> getError() {
    return m_controller.getR().minus(getXHat());
  }

  /**
   * Returns a specific element of the state error.
   *
   * @param index The index of the error element.
   * @return The error at the specified index.
   */
  public double getError(int index) {
    return m_controller.getR().minus(getXHat()).get(index, 0);
  }

  /**
   * Sets the function used to clamp the input u.
   *
   * @param clampFunction The clamping function.
   */
  public void setClampFunction(Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction) {
    this.m_clampFunction = clampFunction;
  }

  /**
   * Returns the function used to clamp the input u.
   *
   * @return The clamping function.
   */
  public Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> getClampFunction() {
    return m_clampFunction;
  }

  /**
   * Resets the reference vector and controller outputs. Also resets the feedforward and the initial
   * state estimate of the observer.
   *
   * @param initialState The initial state vector.
   */
  public final void reset(Matrix<States, N1> initialState) {
    m_nextR.fill(0.0);
    m_controller.reset();
    m_feedforward.reset(initialState);

    if (useExtendedKalmanFilter) {
      m_extendedObserver.setXhat(initialState);
    } else {
      m_observer.setXhat(initialState);
    }
  }

  /**
   * Corrects the state estimate x̂ using the measurement vector y.
   *
   * @param y Measurement vector.
   */
  public void correct(Matrix<Outputs, N1> y) {
    if (useExtendedKalmanFilter) {
      m_extendedObserver.correct(getU(), y);
    } else {
      m_observer.correct(getU(), y);
    }
  }

  /**
   * Predicts the next state using the active observer.
   *
   * @param dtSeconds Timestep for the prediction.
   */
  public void predict(double dtSeconds) {
    final Matrix<Inputs, N1> u =
        clampInput(
            m_controller.calculate(getXHat(), m_nextR).plus(m_feedforward.calculate(m_nextR)));
    if (useExtendedKalmanFilter) {
      m_extendedObserver.predict(u, dtSeconds);
    } else {
      m_observer.predict(u, dtSeconds);
    }
  }

  /**
   * Sets whether to use the extended Kalman Filter.
   *
   * @param useExtended True to use the extended Kalman Filter; false to use the standard Kalman
   *     Filter.
   */
  public void setUseExtendedKalmanFilter(boolean useExtended) {
    this.useExtendedKalmanFilter = useExtended;
  }

  /**
   * Applies the clamping function to the input u.
   *
   * @param unclampedU The unclamped control input.
   * @return The clamped control input.
   */
  private Matrix<Inputs, N1> clampInput(Matrix<Inputs, N1> unclampedU) {
    return m_clampFunction.apply(unclampedU);
  }
}
