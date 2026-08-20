package swervelib.parser;

import java.util.function.Supplier;

import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;

/**
 * Cache for frequently requested data.
 */
public class Cache<T>
{

  /**
   * Cached value.
   */
  private T           value;
  /**
   * Supplier for cached value.
   */
  private Supplier<T> supplier;
  /**
   * Timestamp in microseconds.
   */
  private long        timestamp;
  /**
   * Validity period in microseconds.
   */
  private long        validityPeriod;

  /**
   * Cache for arbitrary values.
   *
   * @param val            Value to cache.
   * @param validityPeriod Validity period in milliseconds.
   */
  public Cache(Supplier<T> val, long validityPeriod)
  {
    supplier = val;
    value = supplier.get();
    timestamp = RobotController.getMonotonicTime();
    this.validityPeriod = validityPeriod * 1000L;
  }

  /**
   * Return whether the cache is stale.
   *
   * @return The stale state of the cache.
   */
  public boolean isStale()
  {
    return (RobotController.getMonotonicTime() - timestamp) > validityPeriod;
  }

  /**
   * Update the cache value and timestamp.
   *
   * @return {@link Cache} used.
   */
  public Cache<T> update()
  {
    this.value = supplier.get();
    this.timestamp = RobotController.getMonotonicTime();
    return this;
  }

  /**
   * Update the supplier to a new source. Updates the value and timestamp as well.
   *
   * @param supplier new supplier source.
   * @return {@link Cache} for chaining.
   */
  public Cache<T> updateSupplier(Supplier<T> supplier)
  {
    this.supplier = supplier;
    update();
    return this;
  }

  /**
   * Update the validity period for the cached value, also updates the value.
   *
   * @param validityPeriod The new validity period in milliseconds.
   * @return {@link Cache} for chaining.
   */
  public Cache<T> updateValidityPeriod(long validityPeriod)
  {
    this.validityPeriod = validityPeriod * 1000L;
    update();
    return this;
  }

  /**
   * Get the most up to date cached value.
   *
   * @return {@link T} updated to the latest cached version.
   */
  public T getValue()
  {
    if (isStale() || RobotBase.isSimulation())
    {
      update();
    }
    return value;
  }

}
