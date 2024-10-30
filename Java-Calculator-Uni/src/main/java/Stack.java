package uk.ac.rhul.cs2800;

import java.util.ArrayList;
import java.util.EmptyStackException;
import java.util.List;

/**
 * This is a stack class which will be used to hold the entries and perform basic stack functions.
 * 
 * @author Robert
 *
 */
public class Stack {
  private int size = 0;
  private List<Entry> entries = new ArrayList<Entry>();

  /**
   * This method adds an entry to the end of the stack and iterates the size variable.
   * 
   * @param i is the entry that will be pushed to the stack
   */
  public void push(Entry i) {
    // Entries[size] = i;
    entries.add(i);
    size++;
  }

  /**
   * This method removes an entry from the end of the stack and decreases the size variable. 
   * 
   * @return entry at point size in the list
   * @throws EmptyStackException if the list is empty
   */
  public Entry pop() {
    if (size == 0) {
      throw new EmptyStackException();
    }
    size--;
    return entries.get(size);
  }

  /**
   * This method retrieves the top entry from the stack. If the stack is empty then a
   * EmptyStackException is thrown.
   * 
   * @return entry at point size - 1 in the list
   * @throws EmptyStackException if the list is empty
   */
  public Entry top() {
    if (size == 0) {
      throw new EmptyStackException();
    }
    return entries.get(size - 1);
  }

  /**
   * This is a getter method for size.
   * 
   * @return size of the list
   */
  public int getSize() {
    return size;
  }

}
