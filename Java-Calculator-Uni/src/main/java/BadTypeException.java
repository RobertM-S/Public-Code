package uk.ac.rhul.cs2800;

/**
 * BadTypeException is used to throw an exception when an invalid symbol or null is found. Extends
 * RuntimeException to reduce need to declare throws.
 * 
 * @author Robert
 *
 */
public class BadTypeException extends RuntimeException {

  /**
   * Added because BadTypeException was asking for a serial id.
   */
  private static final long serialVersionUID = 145874342098475L;

  /**
   * Method calls from the superclass to throw an exception.
   * 
   * @param message to be displayed with error
   */
  public BadTypeException(String message) {
    super(message);
  }

}
