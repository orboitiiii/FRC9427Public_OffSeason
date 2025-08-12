package Control.loops;

import edu.wpi.first.math.Matrix;

/**
 * Abstract base class for control loop implementations.
 *
 * @param <State> Type representing the system state vector (N x 1 matrix)
 * @param <Input> Type representing the control input vector (M x 1 matrix)
 */
public abstract class ModernControlLoops<State extends Matrix<?, ?>, Input extends Matrix<?, ?>> {

  /**
   * Resets the control loop with the specified initial state.
   *
   * @param systemState The initial system state vector structured as: [position1, position2, ...,
   *     positionN/2, velocity1, velocity2, ..., velocityN/2]^T where N is the dimension of the
   *     state vector
   */
  public abstract void reset(State systemState);

  /**
   * Configures the system model parameters.
   *
   * @param systemState Reference state vector for parameter identification: [position1, position2,
   *     ..., positionN/2, velocity1, velocity2, ..., velocityN/2]^T
   */
  public abstract void setModel(State systemState);

  /**
   * Computes the reference trajectory based on desired setpoint and current state.
   *
   * @param positionState Combined matrix containing both setpoint and current position:
   *     [desiredPos1, ..., desiredPosN/2, currentPos1, ..., currentPosN/2]^T
   * @return Computed reference vector for controller tracking
   */
  public abstract Input calculateReference(State positionState);

  /**
   * Computes the feedforward control input using current state.
   *
   * @param currentState Current system state vector
   * @return Feedforward control input vector
   */
  public abstract Input calculateFeedforward(State currentState);

  /**
   * Performs state prediction using system model.
   *
   * @param dt Time step for prediction (seconds)
   */
  public void predict(double dt) {
    // Default no-op implementation
    // Override in subclass with prediction logic
  }

  /**
   * Updates state estimate using sensor measurements.
   *
   * @param measurement Observed measurement vector
   */
  public void correct(Input measurement) {
    // Default no-op implementation
    // Override in subclass with measurement update logic
  }

  /**
   * Sets the next reference state for trajectory following.
   *
   * @param nextState Target state for the next control period
   */
  public void setNextReference(State nextState) {
    // Default no-op implementation
    // Override in subclass with reference transition logic
  }

  /**
   * Gets the computed control input at specified index.
   *
   * @param index Index of the control input channel
   * @return Control input value in [-1.0, 1.0] or physical units
   */
  public double getControlInput(int index) {
    // Default implementation returns neutral value
    // Override in subclass with actual input access
    return 0.0;
  }
}
