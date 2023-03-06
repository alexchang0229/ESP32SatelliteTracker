// Adapted from sgp4 library
// Predicts time of next pass and start azimuth for satellites
long Predict(unsigned long _timeNow) {
  passinfo overpass;                      //structure to store overpass info
  sat.initpredpoint( _timeNow , minPassElevation);     //finds the startpoint

  bool error;

  error = sat.nextpass(&overpass, 20);    //search for the next overpass, if there are more than 20 maximums below the horizon it returns false
  delay(0);

  if ( error == 1) { //no error, prints overpass information
    nextpassEpoch = (overpass.jdstart - 2440587.5) * 86400;
    nextpassEndEpoch = (overpass.jdstop - 2440587.5) * 86400;
    passDuration = nextpassEndEpoch - nextpassEpoch;
    //if(passDuration < 300){
    //   Predict(nextpassEndEpoch);
    //}
    AZstart = overpass.azstart;
    invjday(overpass.jdstart , timeZone , true , year, mon, day, hr, minute, sec); // Convert Julian date to print in serial.
    Serial.println("Next pass for: " + String(satnames[SAT]) + " In: " + String(nextpassEpoch - timeNow) + " Duration:" + String(passDuration));
    Serial.println("Start: az=" + String(overpass.azstart) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));
  } else {
    Serial.println("Prediction error");
    while (true);
  }
  delay(0);

  return nextpassEpoch;
}
