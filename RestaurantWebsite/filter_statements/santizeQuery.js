let validator = require('validator');

/**
 * Santizes input to avoid sql injection. Returns null if input is unable to be sanitized.
 * @param input {String} the input statement
 * @returns {(boolean|sanitized_input)}
 */
function sanitize(input){
  input = validator.blacklist(input, "!£$%^&*()'{};:@~#,<>?/-");
  if (validator.isAlpha(input) || validator.isNumeric(input)){
    input = input.replace('OR', '');
    input = input.replace('AND', '');
    return [true, input];
  }
  return [false, null];
}
module.exports = sanitize;