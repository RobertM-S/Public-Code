package uk.ac.rhul.cs2800;

/**
 * This creates a set of enums for all the symbols accepted and an invalid enum for ones that are
 * not accepted.
 * 
 * @author Robert
 *
 */
public enum Symbol {
  LEFT_BRACKET(" ("), RIGHT_BRACKET(" )"), TIMES(" *"), DIVIDE(" /"), PLUS(" +"), 
  MINUS(" -"), INVALID(" %");

  private String description;

  /**
   * Assigns a description to the enum.
   * 
   * @param x is the description taken from the enum Type.
   */
  
  Symbol(String x) {
    description = x;
  }

  @Override
  public String toString() {
    return description;
  }

}

