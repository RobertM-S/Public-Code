/**
 * A script that creates a query of statements based on the list passed to it
 * @param sort a list of 1's and 0's referencing filters of allergies and ordering.
 * @return String sortQuery is conditions to be added to a sql query
 * @author Robert MacKenzie-Shannon
 */

function allergyStatements(sort) {
    let sortQuery = "";
    if (sort[0] === 1){
        sortQuery += "&& Nuts = '0' ";
    }
    if (sort[1] === 1){
        sortQuery += "&& Lactose = '0' ";
    }
    if (sort[2] === 1){
        sortQuery += "&& Gluten = '0' ";
    }
    if (sort[3] === 1){
        sortQuery += "&& Egg = '0' ";
    }
    if (sort[4] === 1){
        sortQuery += "ORDER BY price ASC"
    }
    return sortQuery
}
module.exports = allergyStatements