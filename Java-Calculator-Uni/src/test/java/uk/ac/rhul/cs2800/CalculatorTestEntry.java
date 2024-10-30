package uk.ac.rhul.cs2800;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


class CalculatorTestEntry {
  // I create a few instances of Entry for use with testing
  private Entry entry;
  private Entry testCheck;
  private Entry testCheckType;

  // Some hard coded values for use in testing
  float flt = 6;
  Symbol sym = Symbol.MINUS;
  String str = "-";
  Type type = Type.NUMBER;

  @BeforeEach
  // Sets up a clean entry before each test so that previous tests do not carry over
  public void setup() {
    entry = new Entry();
  }

  @Test
  // Test 8
  // Tests that an entry can be made using a float
  public void setupFloat() {
    entry = new Entry(flt);
  }

  @Test
  // Test 9
  // Tests that an entry can be made using a Symbol
  public void setupSymbol() {
    entry = new Entry(sym);
  }

  @Test
  // Test 10
  // Tests that an entry can be made using a String
  public void setupString() {
    entry = new Entry(str);
  }

  @Test
  // Test 11
  // Tests that getType correctly returns Type.NUMBER
  public void testGetType() {
    assertEquals(entry.getType(), Type.NUMBER, "type does not equal NUMBER");
  }

  @Test
  // Test 12
  // Tests that getString correctly returns +
  public void testGetString() {
    assertEquals(entry.getString(), "+", "str does not equal +");
  }

  @Test
  // Test 13
  // Tests that getSymbols correctly returns Symbol.PLUS
  public void testGetSymbol() {
    assertEquals(entry.getSymbol(), Symbol.PLUS, "Symbol does not equal PLUS");
  }

  @Test
  // Test 14
  // Tests that getValue correctly returns 27
  public void testGetValue() {
    assertEquals(entry.getValue(), 27, "number does not equal 27");
  }

  @Test
  // Test 15
  // Resets the values in entry to the ones hard coded for testing purposes and checks that it
  // returns 6
  public void testConstructGetValue() {
    entry = new Entry(flt);
    assertEquals(entry.getValue(), 6, "number does not equal 6");
  }

  @Test
  // Test 16
  // Resets the values in entry to the ones hard coded for testing purposes and checks that it
  // returns Symbol.MINUS
  public void testConstructGetSymbol() {
    entry = new Entry(sym);
    assertEquals(entry.getSymbol(), Symbol.MINUS, "other does not equal MINUS");
  }

  @Test
  // Test 17
  // Resets the values in entry to the ones hard coded for testing purposes and checks that it
  // returns -
  public void testConstructGetString() {
    entry = new Entry(str);
    assertEquals(entry.getString(), "-", "str does not equal -");
    // added test to ensure that BadTypeException is thrown if the string is empty
    entry = new Entry("     ");
    assertThrows(BadTypeException.class, () -> entry.getString(), " Empty string");
  }

  @Test
  // Test 18
  // Tests that the hashCode method correctly returns unique values
  void testHash() {
    entry = new Entry("3 + 3");
    testCheck = new Entry("3 3 +");

    assertEquals(entry.hashCode(), entry.hashCode(),
        "hashCode does not return the same thing for two inputs that are the same");

    assertNotSame(entry.hashCode(), testCheck.hashCode(),
        "hashCode returns the same value for two different inputs");

    entry = new Entry(Type.NUMBER);
    testCheckType = new Entry(Type.STRING);

    assertEquals(entry.hashCode(), entry.hashCode(),
        "hashCode does not create the same code for two of the same input");

    assertNotSame(entry.hashCode(), testCheckType.hashCode(),
        "hashCode returns the same value for two different inputs");
  }

  @Test
  // Test 19
  // Tests that equals method correctly evaluates two entry objects
  void testEquals() {
    assertTrue(entry.equals(entry), "equals does not return true for the same values");
    testCheck = new Entry("test");
    assertFalse(entry.equals(testCheck), "equals does returns true for different values");
  }



}
