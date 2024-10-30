package uk.ac.rhul.cs2800;

/**
 * This creates a set of enums for all the types of input accepted and an invalid enum for ones that
 * are not accepted.
 * 
 * @author Robert
 *
 */
public enum Type {
  NUMBER(" numbers_ranging_from_0_to_9"), SYMBOL(" symbols_including_(_)_*_/_+_-"), STRING(
      " combination_of_symbols_and_numbers"), INVALID(" invalid_input");

  private String description;

  /**
   * Assigns a description to the enum.
   * 
   * @param x is the description taken from the enum Type.
   */
  private Type(String x) {
    description = x;
  }

  @Override
  public String toString() {
    return name() + description;
  }

}
