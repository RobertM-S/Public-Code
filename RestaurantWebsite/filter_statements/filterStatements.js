/**
 * Adds checks to the SQL query based on values within the FilterOptions array
 *
 * @param FilterOptions array of 1's and 0's containing the desired filters
 * @returns filterQuery SQL containing only the checks to be added to the main sql query
 *
 * @author Robert M-S
 */

function filterStatements(FilterOptions) {
    let filterQuery = "";
    
    if (FilterOptions[0] === 1){
        filterQuery += "&& Nuts = '0' ";
    }
    if (FilterOptions[1] === 1){
        filterQuery += "&& Lactose = '0' ";
    }
    if (FilterOptions[2] === 1){
        filterQuery += "&& Gluten = '0' ";
    }
    if (FilterOptions[3] === 1){
        filterQuery += "&& Egg = '0' ";
    }

    if (FilterOptions[4] === 1 && FilterOptions[5] === 1){
        filterQuery += "ORDER BY price ASC, calories ASC";
        return filterQuery
    }

    if (FilterOptions[4] === 1){
        filterQuery += "ORDER BY price ASC";
    }
    if (FilterOptions[5] === 1){
        filterQuery += "ORDER BY calories ASC";
    }
    return filterQuery
}
module.exports = filterStatements