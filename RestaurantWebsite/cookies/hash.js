const encrypt = require("cryptojs")

/**
 * A small hash function used to convert the username into a hash ready to be salted and used as the users cookie
 * @param {String} data The data to hash
 * @returns {Promise<string>} The output of the hashing function
 * @author Robert M-S Connor Kirkpatrick
 */
async function hash(data) {
    let encrypted = encrypt.Crypto.SHA256(data)
    return encrypted.toString()
}
module.exports = hash