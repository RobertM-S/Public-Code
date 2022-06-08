/**
 * A script that connects to a database and adds a new account which includes hashing the password and storing the hash
 * @author Robert MacKenzie-Shannon
 */

const dotenv = require("dotenv").config();

const mysql = require("mysql");
// User name and password should be changed to be the credentials required for the account
const username = "";
const password = "";
const encrypt = require ("cryptojs")

function main() {
    var connect = mysql.createConnection({
        host: process.env.DB_HOST,
        user: process.env.DB_USER,
        password: process.env.DB_PASSWORD,
        database: process.env.DB_NAME,
    });
    // password is encrypted using sha256
    let encrypted = encrypt.Crypto.SHA256(password)
    let query = "INSERT INTO `heroku_f256589a3f9af35`.`accounts` (`Username`, `Password`) VALUES ('" + username +"', '" + encrypted +"')"
    connect.query(query, function (result, err) {
        console.log(err)
        if (err) throw err
    })
};

main()
