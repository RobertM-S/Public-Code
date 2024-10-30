package uk.ac.rhul.cs2800;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.EmptyStackException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CalculatorTestStack {
  private Stack stack;
  private Entry entry;
  private Entry topCheck;


  @BeforeEach
  public void setup() {
    stack = new Stack();
    entry = new Entry(2.5f);
  }



  // This test is now obsolete but was originally used to make sure that when I created my list,
  // that it was empty so that my other tests would run unimpeded.

  // @Test
  // Test 1
  // void testEntry() { assertEquals(stack.entries(), 0, "test a newly created stack
  // to see that its list is empty"); }


  @Test
  // Test 2
  // The purpose of this test is to check that i can successfully push an entry to the stack
  // I then check that size has correctly been increased by 1 since i have pushed 1 thing.
  void testPush() {
    // stack.push(7);
    // I first created this test using integers and have now swapped to using Entries
    stack.push(entry);
    assertEquals(stack.getSize(), 1, "Test that push increases size by 1");
  }

  @Test
  // Test 3
  // Tests that pop correctly throws an error when you try to pop from an empty stack
  void testPop() {
    assertThrows(EmptyStackException.class, () -> stack.pop(),
        "You cannot pop from an empty stack");
  }

  @Test
  // Test 4
  // Tests the push and pop function working together
  void testPushPop() {
    stack.push(entry);
    entry = new Entry("-");
    stack.push(entry);
    assertEquals(stack.getSize(), 2,
        "test to see if the stack has size 2 when 2 things are pushed");
    assertTrue(entry.equals(stack.pop()), "tests that what is popped is equal to what is pushed");
    stack.pop();
    // assertEquals(stack.pop(), 13, "Tests that 13 is returned when popped");
    // assertEquals(stack.pop(), 2, "Tests that 2 is returned when popped");

    assertThrows(EmptyStackException.class, () -> stack.pop(),
        "You cannot pop from an empty stack");
  }

  @Test
  // Test 5
  // Tests that stack can be pushed to many times without issue
  void testManyPush() {
    for (float x = 1f; x < 50; x++) {
      entry = new Entry(x);
      stack.push(entry);
      assertEquals(stack.getSize(), x, "test to see if the stack size iterates with x");
      assertEquals(entry.number, x, "test to see if entry.number correctly iterates with x");
    }
  }


  @Test
  // Test 6
  // Tests that the top method works correctly without any adverse effect on the stack
  void top() {
    stack.push(entry);
    topCheck = new Entry("*");
    stack.push(topCheck);
    entry = new Entry(78.3f);
    stack.push(entry);
    // assertEquals(stack.top(), 9, "Tests that 9 is the top of the stack");
    // assertEquals(stack.pop(), 9, "Tests that 9 is returned when popped");
    // assertEquals(stack.top(), 7, "Tests that 7 is the top of the stack");
    assertTrue(entry.equals(stack.top()), "Tests that 78.3f is on top of the stack");
    assertTrue(entry.equals(stack.pop()), "tests that what is popped is equal to what is pushed");
    assertTrue(topCheck.equals(stack.top()), "Tests that * is on top of the stack");
    assertTrue(topCheck.equals(stack.top()),
        "Tests that Symbol.TIMES is still on top of the stack e.g. not been popped");
  }

  @Test
  // Test 7
  // Tests that size is iterated correctly 
  void testSize() {
    assertEquals(stack.getSize(), 0, "Test a newly created stack to see that it has size zero");
    stack.push(entry);
    stack.push(entry);
    stack.push(entry);
    assertEquals(stack.getSize(), 3, "Test a newly created stack to see that it has size three");
    
    
  }

}
