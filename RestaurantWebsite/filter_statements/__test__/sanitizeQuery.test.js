const sanitize = require('../santizeQuery')

test("Check that user input is string", () => {
  expect(sanitize("aaaa")[0]).toBeTruthy();
});

test("Check that user input is number", () => {
  expect(sanitize("12.5")[0]).toBeTruthy();
});

test("Check that OR is removed", () => {
  expect(sanitize("OR")[1]).toBe('');
});

test("Check that OR is removed", () => {
  expect(sanitize("AND")[1]).toBe('');
});


test("Check that special characters is removed", () => {
  expect(sanitize("!£$%^&*()'{};:@~#,<>?/-a")[1]).toBe('a');
});

test("Check that inavlid queries treated as FALSE", () => {
  expect(sanitize("!£$%^&*()'{};:@~#,<>?/-")[0]).toBeFalsy();
});