// Created by Yamnel S.

var cleanCoords = [];

// This is hard coded to the coordinate location 
// on which the drone's lunching pad is at.
var DroneHome = {lat:26.464546315991463, lng:-81.76931453403085}; // School's Soccer field 

var map;
var markers = [];

function initMap() {
  var mapOptions = {
    zoom: 19,
    center: DroneHome,
    mapTypeId: 'satellite',
    tilt: 0
  }
  map = new google.maps.Map(document.getElementById('map'), mapOptions);

  // This event listener will call addMarker() when the map is clicked.
  map.addListener('click', function(event) {
    addMarker({position:event.latLng, info:event.latLng}); // add marker on click
	  
	  
	// Here I do some string formatting on the Coordinates.  
    var rawLatLng = String(event.latLng);

    var cleanLatLng = rawLatLng.replace(',', '');
    var cleanLatLng = cleanLatLng.replace(')', '');
    var cleanLatLng = cleanLatLng.replace('(', '');
    cleanCoords.push(cleanLatLng);
  });

  // Adds a marker at the center of the map where the drone is.
  // The original image used here, can be found under: img/3DRIcon.png
  addMarker({position:DroneHome, iconImg:'https://i.imgur.com/ZBMDKVf.png', info:'Drone'});
}

// Adds a marker to the map and push to the array.
function addMarker(markerInfo) {
  var marker = new google.maps.Marker({
    position: markerInfo.position,
    map: map
  });
  // check for custom image
  if(markerInfo.iconImg){
    //set icon image
    marker.setIcon(markerInfo.iconImg);
  }

  if(markerInfo.info){
    var infoPopUp = new google.maps.InfoWindow({
      content:'<h1>'+markerInfo.info+'</h1>'
    });
    marker.addListener('click', function(){
      infoPopUp.open(map, marker);
    })
  }

  markers.push(marker);
}

function pushToDB(){ // on btn click
  // checks the value of the 'ready' child in the DB
  firebase.database().ref().child('ready').once('value').then(function(snapshot){

    //  if there are coords stored and the DB is ready
    if(cleanCoords.length > 0 && snapshot.val() == 1){ // if there are pins and the ready flag is true
      firebase.database().ref().child('ready').set(0); // set ready to false
      firebase.database().ref().child('pilot').set(cleanCoords.shift()); // update coords.

    }else {
      if(snapshot.val() != 1){
        window.alert("Please wait for the drone to finish its current task!")
      }
      if(cleanCoords.length == 0){
        window.alert("Please add a new pin on the map.")
      }
    }
  });
}

// deletes all markers on the map and resets the DB flag
function rstBtn(){
  deleteMarkers()
  addMarker({position:DroneHome, iconImg:'https://i.imgur.com/ZBMDKVf.png', info:'Drone'});
  firebase.database().ref().child('ready').set(1);
  firebase.database().ref().child('pilot').set("");
}

// Sets the map on all markers in the array.
function setMapOnAll(map) {
  for (var i = 0; i < markers.length; i++) {
    markers[i].setMap(map);
  }
}

// Removes the markers from the map, but keeps them in the array.
function clearMarkers() {
  setMapOnAll(null);
}

// Shows any markers currently in the array.
function showMarkers() {
  setMapOnAll(map);
}


// Work In progress... As it is, it only remeves the very last pin, ONCE
function removeLastMarker(){

  var toBeRemoved2 = cleanCoords.indexOf(cleanCoords[cleanCoords.length-1]);
  if(toBeRemoved2 != -1){
    cleanCoords.splice(toBeRemoved2, 1);
  }

  markers[markers.length - 1].setMap(null);

  var toBeRemoved = markers.indexOf(markers[markers.length-1]);
  if(toBeRemoved != -1){
    markers.splice(toBeRemoved, 1);
  }
}

// Deletes all markers in the array by removing references to them.
function deleteMarkers() {
  clearMarkers();
  markers = [];
  cleanCoords = [];
}
