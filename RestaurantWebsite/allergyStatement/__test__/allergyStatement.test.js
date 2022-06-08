const allergyStatements = require('../allergyStatements')

test("Checks that the array holding the allergies changes everytime a tickbox is ticked", () => {
    expect(allergyStatements([1,1,0,1])).toBe("&& Nuts = '0' && Lactose = '0' && Egg = '0' ")
})