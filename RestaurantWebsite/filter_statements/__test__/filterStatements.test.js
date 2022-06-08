const filterStatements = require('../filterStatements')

test("Checks that the array holding the filters changes everytime a tickbox is ticked", () => {
    expect(filterStatements([1,1,0,1,0,1])).toBe("&& Nuts = '0' && Lactose = '0' && Egg = '0' ORDER BY calories ASC")
})

test("Checks that the array holding the filters changes everytime a tickbox is ticked", () => {
    expect(filterStatements([1,0,1,0,1,1])).toBe("&& Nuts = '0' && Gluten = '0' ORDER BY price ASC, calories ASC")
})