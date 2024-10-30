package uk.ac.rhul.cs2800;

/**
 * Entry is a object class that will be instantiated as a object to placed into the stack.
 * 
 * @author Robert
 *
 */
public class Entry {
  
  // Here i have hard coded some values purely for testing purposes 

  float number = 27;
  Symbol other = Symbol.PLUS;
  String str = "+";
  Type type = Type.NUMBER;

  /**
   * Creates a unique identifier to allow for use with in data structures such as a hash map.
   *
   * @return return the unique identifier
   */
  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((str == null) ? 0 : str.hashCode());
    result = prime * result + ((type == null) ? 0 : type.hashCode());
    return result;
  }

  /**
   * The equals method checks that both the String and Types are equal of the current entry and
   * passed entry.
   * 
   * @param toCheck is the entry which will be evaluated against the current entry
   * @return boolean value depending on whether the entry's are equal to each other
   */
  public boolean equals(Entry toCheck) {
    if (this.str == toCheck.str && this.type == toCheck.type) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Entry is a generic constructor purely used for testing purposes.
   * 
   */
  public Entry() {}

  /**
   * Entry constructor accepting a Type as a parameter.
   * 
   * @param ype is passed so that the variable type can be changed
   */
  public Entry(Type ype) {
    type = ype;
  }

  /**
   * Entry constructor accepting a float as a parameter.
   * 
   * @param value is passed so that the variable number can be changed
   */
  public Entry(float value) {
    number = value;
  }

  /**
   * Entry constructor accepting a Symbol as a parameter.
   * 
   * @param which is passed so that the variable other can be changed
   */
  public Entry(Symbol which) {
    other = which;
  }

  /**
   * Entry constructor accepting a string as a parameter.
   * 
   * @param together is passed so that the variable str can be changed
   */
  public Entry(String together) {
    str = together;
  }

  /**
   * The method is a getter for the type attribute.
   * 
   * @return the current value of the Type type
   * @throws BadTypeException if type is INVALID
   */
  public Type getType() {
    if (type.equals(Type.INVALID)) {
      throw new BadTypeException("type is Invalid");
    }
    return type;
  }

  /**
   * The getString method returns the String str.
   * 
   * @return str from inside the entry class
   * @throws BadTypeException thrown if the string is empty or null
   */
  public String getString() {
    // strip is used to remove white spaces to check is str is empty
    if (str.strip() == "" || str.strip() == null) {
      throw new BadTypeException("String is equal to null");
    }
    return str;
  }

  /**
   * The getSymbol method returns the Symbol other. A bad type exception is called if the Symbol is
   * INVALID.
   * 
   * @return other from inside the entry class
   * @throws BadTypeException thrown if the Symbol is INVALID
   */
  public Symbol getSymbol() {
    if (other.equals(Symbol.INVALID)) {
      throw new BadTypeException("symbol is Invalid");
    }
    return other;
  }

  /**
   * The getValue method returns the Float number.
   * 
   * @return number from inside the entry class
   */
  public float getValue() {
    return number;
  }

}
