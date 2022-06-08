/**
 * Script references all buttons on the main menu page and, using sockets, fires off events when the user presses a button
 * @author Robert MacKenzie-Shannon
 */

var socket = io();
sessionStorage.setItem("user_flag", JSON.stringify(false));
menuSetLayout(socket)
// default PastEvent set to all incase user click a filter before clicking a button
var pastEvent = 'All';
// binary array for storing the users desired filter options
let Filter_Options = [0,0,0,0,0,0]
// function that adds buttons for navigating to different pages
menuAddItems()
//retrieves buttons from different scripts so that button clicks may be handled in one place
let allButton = document.getElementById("ShowAll");
let startersButton = document.getElementById("ShowStarters");
let mainsButton = document.getElementById("ShowMains");
let dessertButton = document.getElementById("ShowDeserts");
let drinkButton = document.getElementById("ShowDrinks");
let menuOption = document.getElementById("menuList");
let menuItems = document.getElementById("items");
let query = document.getElementById("SearchQuery");
let userQuery = document.getElementById("userQuery");
let querySubmit = document.getElementById("Search");
let nutFilter = document.getElementById("nut_Filter");
let lactoseFilter = document.getElementById("lactose_Filter");
let glutenFilter = document.getElementById("gluten_Filter");
let eggFilter = document.getElementById("egg_Filter");
let priceFilter = document.getElementById("price_low");
let CaloriesFilter = document.getElementById("calories_low");

/**
 * Event listeners that are linked to a button which will fire off to a socket listener once an action has been met
 * Passes a variable containing the last event fired off and an array of the sorts associated with the action.
 * @author Robert MacKenzie-Shannon
 */

//Fires when the user enters into the search bar
userQuery.addEventListener("submit", function(e) {
  e.preventDefault()
  menu_button_clicked("Query", Filter_Options)
})
//Fires when the user checks the nut filter, uses an xor to change the value of the number referencing the nut filter in
//Filter_Options
nutFilter.addEventListener("change", function (e) {
  Filter_Options[0] = Filter_Options[0] ^ 1;
  e.preventDefault();
  menu_button_clicked(pastEvent, Filter_Options);
});
//Fires when the user checks the lactose filter, uses an xor to change the value of the number referencing the lactose filter in
//Filter_Options
lactoseFilter.addEventListener("change", function (e) {
  Filter_Options[1] = Filter_Options[1] ^ 1;
  e.preventDefault();
  menu_button_clicked(pastEvent, Filter_Options);
});
//Fires when the user checks the gluten filter, uses an xor to change the value of the number referencing the gluten filter in
//Filter_Options
glutenFilter.addEventListener("change", function (e) {
  Filter_Options[2] = Filter_Options[2] ^ 1;
  e.preventDefault();
  menu_button_clicked(pastEvent, Filter_Options);
});
//Fires when the user checks the egg filter, uses an xor to change the value of the number referencing the egg filter in
//Filter_Options
eggFilter.addEventListener("change", function (e) {
  Filter_Options[3] = Filter_Options[3] ^ 1;
  e.preventDefault();
  menu_button_clicked(pastEvent, Filter_Options);
});
//Fires when the user checks the price filter, uses an xor to change the value of the number referencing the price filter in
//Filter_Options
priceFilter.addEventListener("change", function (e) {
  Filter_Options[4] = Filter_Options[4] ^ 1;
  e.preventDefault();
  menu_button_clicked(pastEvent, Filter_Options);
});
//Fires when the user checks the calories filter, uses an xor to change the value of the number referencing the calories filter in
//Filter_Options
CaloriesFilter.addEventListener("change", function (e) {
  Filter_Options[5] = Filter_Options[5] ^ 1;
  e.preventDefault();
  menu_button_clicked(pastEvent, Filter_Options);
});
//Fires when the user clicks the all button, which shows all items currently in stock. Sets the past even to 'All' for use
//with filters
allButton.addEventListener("click", function (e) {
  e.preventDefault();
  menu_button_clicked("All", Filter_Options);
  pastEvent = 'All';
});
//Fires when the user clicks the starters button, which shows all items currently in stock. Sets the past even to 'Starters' for use
//with filters
startersButton.addEventListener("click", function (e) {
  e.preventDefault();
  menu_button_clicked("Starters", Filter_Options);
  pastEvent = 'Starters';
});
//Fires when the user clicks the mains button, which shows all items currently in stock. Sets the past even to 'Mains' for use
//with filters
mainsButton.addEventListener("click", function (e) {
  e.preventDefault();
  menu_button_clicked("Mains", Filter_Options)
  pastEvent = 'Mains';
});
//Fires when the user clicks the dessert button, which shows all items currently in stock. Sets the past even to 'Desserts' for use
//with filters
dessertButton.addEventListener("click", function (e) {
  e.preventDefault();
  menu_button_clicked("Desserts", Filter_Options)
  pastEvent = 'Desserts';
});
//Fires when the user clicks the Drinks button, which shows all items currently in stock. Sets the past even to 'Drinks' for use
//with filters
drinkButton.addEventListener("click", function (e) {
  e.preventDefault();
  menu_button_clicked("Drinks", Filter_Options)
  pastEvent = 'Drinks';
});
//code manages menu request via query
querySubmit.addEventListener("click", function (e) {
  e.preventDefault();
  menu_button_clicked("Query", Filter_Options)
});

/**
 * Socket event that receives the authentication information from the main server. If data is false, the users
 * cookie is invalid and thus is cleared. Clients and the manager can access this page, waiters will be sent
 * back to the portal. When authenticated the script will also display the username to the client
 * @event Username
 * @param data {Boolean|string} Either the clients username if successfully authenticated or boolean false
 * @author Connor Kirkpatrick
 */
socket.on("Username", (data) => {
  if(!data){
    clearCookie(socket);
  }
  else{
    if(data.indexOf("Manager") > -1){
      document.getElementById("username").innerText = data;
    }
    else{
      window.location.assign("/Staff_Portal")
    }
  }
})