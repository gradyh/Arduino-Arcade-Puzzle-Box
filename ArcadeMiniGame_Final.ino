/*Arduino Arcade Mini Game with LED Panel Meters
Grady Hillhouse 2015 */

//Utility functions to help debugging running code.
//#define DEBUG //Uncomment for debugging

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x) 
#endif

//Declare Variables
float coeff[8];      //Value of coefficients in linear functions of panel meter readings
float potValue[3];    //Value of potentiometer (0-1)
int cOrder[3];      //Order of coefficients for randomizing
float fMax = 1023.0; //Maximum value of panel meter reading
float fMaxPercent = 0.8; //Maximum contribution of a single potentiometer to the overall panel meter reading
int goalLED[2]; //Which LED will light for each panel meter
int goal[2]; //Goal value for meter reading (0-1023)
float goalPotValue[3]; //Goal value for the three potentiometers
float goalPotBounds[2]; //Bounds for goal potentiometer values
int sOrder[4]; //Order for solving for second coefficients (there are four solution methods)
int panelMeterValue[2] = {0, 0}; //Value of panel meter
int j = 0; //Loop index

//Declare pins
const int pot[3] = {A0, A1, A2};  //Potentiometer pins
const int pMeter[2] = {10, 11};    //Panel meter pins
const int pLED[10] = {A4, A5, 2, 3, 4, 5, 6, 7, 8, 12};
const int pVib = 9; //vibration motor
const int rButton = 13; //reset button

//LED Calibration - Set these values after installation
int calibLED[10] = {136, 324, 472, 620, 852, 132, 330, 524, 696, 888};

void setup()
{
  //Initialize serial for debugging
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  DEBUG_PRINTLN("Debugging...");
  
  //Initialize pin modes
  for (int i = 0; i < 10; i++) {
    pinMode(pLED[i], OUTPUT);
  }
  
  for (int i = 0; i < 2; i++) {
    pinMode(pMeter[i], OUTPUT);
  }
    
  pinMode(pVib, OUTPUT);
    
  //Intro test sequence
  for (int i = 0; i < 10; i++) {
    digitalWrite(pLED[i], LOW);
  }
  
  for(int i = 0; i < 5; i++){
    digitalWrite(pLED[i], HIGH);
    digitalWrite(pLED[i + 5], HIGH);
    analogWrite(pMeter[0], calibLED[i] / 4);
    analogWrite(pMeter[1], calibLED[i + 5] / 4);
    delay(400);
    digitalWrite(pLED[i], LOW);
    digitalWrite(pLED[i + 5], LOW);
    }  
  
  for(int i = 0; i < 5; i++){
    digitalWrite(pLED[i], HIGH);
    digitalWrite(pLED[i + 5], HIGH);
  }
  
  analogWrite(pMeter[0],0);
  analogWrite(pMeter[1],0);
  delay(200);
  
  for(int i = 0; i < 256; i++){
    analogWrite(pMeter[0],i);
    analogWrite(pMeter[1],i);
    delay(3);
  }
  
  for(int i = 255; i > 0; i--){
    analogWrite(pMeter[0],i);
    analogWrite(pMeter[1],i);
    delay(3);
  }
  
  delay(750);
  
  for(int i = 0; i < 5; i++){
    digitalWrite(pLED[i], LOW);
    digitalWrite(pLED[i + 5], LOW);
  }
}


void loop() {
  
  int secondSolution = 0;
  
  //Find panel meter values and potentiometer relationships
  while(secondSolution == 0) {
    
    if (j > 0) {
      DEBUG_PRINTLN("Solution not found. Trying again...");
      DEBUG_PRINTLN();
      DEBUG_PRINTLN();
    }
    
    //Initialize Variables
    for(int i = 0; i < 2; i++){
      goal[i] = 0;
      goalPotBounds[i] = 0;
    }
    for (int i = 0; i < 8; i++){
      coeff[i] = 0;
    }
    for (int i = 0; i < 3; i++){
      cOrder[i] = 0;
      goalPotValue[i] = 0;
    }
    for (int i = 0; i < 10; i++) {
      digitalWrite(pLED[i], LOW);
    }
    
    //Choose goal for panel meter LEDs
    randomSeed(analogRead(A3)*millis());
    for (int i = 0; i < 2; i++){
      goalLED[i] = random(5);
      
      DEBUG_PRINT("Goal LED for Panel Meter ");
      DEBUG_PRINT(i + 1);
      DEBUG_PRINT(": ");
      DEBUG_PRINTLN(goalLED[i] + 1);
    }
    
    //Choose goal for panel meter reading (this can be calibrated after LEDs are installed)
    switch(goalLED[0]) {
      case 0:
        goal[0] = calibLED[0];
        break;
      case 1:
        goal[0] = calibLED[1];
        break;
      case 2:
        goal[0] = calibLED[2];
        break;
      case 3:
        goal[0] = calibLED[3];
        break;
      case 4:
        goal[0] = calibLED[4];
        break;
    }
    
    switch(goalLED[1]) {
      case 0:
        goal[1] = calibLED[5];
        break;
      case 1:
        goal[1] = calibLED[6];
        break;
      case 2:
        goal[1] = calibLED[7];
        break;
      case 3:
        goal[1] = calibLED[8];
        break;
      case 4:
        goal[1] = calibLED[9];
        break;
    }
    
    //Choose order to select meter one coefficients
    randomizeCOrder(cOrder);
    
    //Set first coefficient to a value within a specified range
    randomSeed(analogRead(A3)*millis());
    coeff[cOrder[0]] = (2.0 * fMax * fMaxPercent) * random(100) / 100.0 - (fMax * fMaxPercent);
    
    //Set second coefficient to a value within a specified range
    randomSeed(analogRead(A3)*millis());
    coeff[cOrder[1]] = (2.0 * (fMax - abs(coeff[cOrder[0]]))) * random(100) / 100.0 - (fMax - abs(coeff[cOrder[0]]));
    while(coeff[cOrder[1]] > (fMax * fMaxPercent)){
      coeff[cOrder[1]] = (2.0 * (fMax - abs(coeff[cOrder[0]]))) * random(100) / 100.0 - (fMax - abs(coeff[cOrder[0]]));
    }
    
    //Set third coefficient to a value which makes the range of f [0, fMax] (two choices, positive or negative)
    randomSeed(analogRead(A3)*millis());
    coeff[cOrder[2]] = (fMax - abs(coeff[cOrder[0]]) - abs(coeff[cOrder[1]])) - 2 * (fMax - abs(coeff[cOrder[0]]) - abs(coeff[cOrder[1]])) * random(1);
    
    //Set fourth coefficient to a value which makes the minimum at 0
    for (int i = 0; i < 3; i++){
      coeff[3] = coeff[3] + coeff[i] * v(coeff[i]);
    }
    
    //Check that when all +coeff pots are all the way up and all -coeff pots are all the way down, f = fmax
    if (pow(coeff[0] * u(coeff[0]) + coeff[1] * u(coeff[1]) + coeff[2] * u(coeff[2]) + coeff[3] - fMax, 2.0) > .01) error("Problem with inital coefficient selection.");
    
    //Check that when all +coeff pots are all the way down and all -coeff pots are all the way up, f = 0
    if (pow(-coeff[0] * v(coeff[0]) - coeff[1] * v(coeff[1]) - coeff[2] * v(coeff[2]) + coeff[3], 2.0) > .01) error("Problem with inital coefficient selection.");
    
    //Choose order to select potentiometer values
    randomizeCOrder(cOrder);
    
    //Find first bound for first potentiometer value
    if ((goal[0] - coeff[cOrder[1]] * u(coeff[cOrder[1]]) - coeff[cOrder[2]] * u(coeff[cOrder[2]]) - coeff[3]) / coeff[cOrder[0]] < 0.0){
      goalPotBounds[0] = 0.0;
    }
    else if ((goal[0] - coeff[cOrder[1]] * u(coeff[cOrder[1]]) - coeff[cOrder[2]] * u(coeff[cOrder[2]]) - coeff[3]) / coeff[cOrder[0]] > 1.0){
      goalPotBounds[0] = 1.0;
    }
    else {
      goalPotBounds[0] = (goal[0] - coeff[cOrder[1]] * u(coeff[cOrder[1]]) - coeff[cOrder[2]] * u(coeff[cOrder[2]]) - coeff[3]) / coeff[cOrder[0]];
    }
    
    //Find second bound for first potentiometer value
    if ((goal[0] + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]) - coeff[3]) / coeff[cOrder[0]] < 0.0){
      goalPotBounds[1] = 0.0;
    }
    else if ((goal[0] + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]) - coeff[3]) / coeff[cOrder[0]] > 1.0){
      goalPotBounds[1] = 1.0;
    }
    else {
      goalPotBounds[1] = (goal[0] + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]) - coeff[3]) / coeff[cOrder[0]];
    }
    
    //Choose a value for the first potentiometer within the bounds
    randomSeed(analogRead(A3)*millis());
    if (goalPotBounds[0] > goalPotBounds[1]) {
      goalPotValue[cOrder[0]] = (random(100) / 100.0) * (goalPotBounds[0] - goalPotBounds[1]) + goalPotBounds[1];
    }
    else {
      goalPotValue[cOrder[0]] = (random(100) / 100.0) * (goalPotBounds[1] - goalPotBounds[0]) + goalPotBounds[0];
    }
    
    //Find first bound for second potentiometer
    if ((goal[0] - coeff[cOrder[2]] * u(coeff[cOrder[2]]) - coeff[3] - coeff[cOrder[0]] * goalPotValue[cOrder[0]]) / coeff[cOrder[1]] < 0.0){
      goalPotBounds[0] = 0.0;
    }
    else if ((goal[0] - coeff[cOrder[2]] * u(coeff[cOrder[2]]) - coeff[3] - coeff[cOrder[0]] * goalPotValue[cOrder[0]]) / coeff[cOrder[1]] > 1.0){
      goalPotBounds[0] = 1.0;
    }
    else {
      goalPotBounds[0] = (goal[0] - coeff[cOrder[2]] * u(coeff[cOrder[2]]) - coeff[3] - coeff[cOrder[0]] * goalPotValue[cOrder[0]]) / coeff[cOrder[1]];
    }
    
    //Find second bound for second potentiometer
    if ((goal[0] + coeff[cOrder[2]] * v(coeff[cOrder[2]]) - coeff[3] - coeff[cOrder[0]] * goalPotValue[cOrder[0]]) / coeff[cOrder[1]] < 0.0){
      goalPotBounds[1] = 0.0;
    }
    else if ((goal[0] + coeff[cOrder[2]] * v(coeff[cOrder[2]]) - coeff[3] - coeff[cOrder[0]] * goalPotValue[cOrder[0]]) / coeff[cOrder[1]] > 1.0){
      goalPotBounds[1] = 1.0;
    }
    else {
      goalPotBounds[1] = (goal[0] + coeff[cOrder[2]] * v(coeff[cOrder[2]]) - coeff[3] - coeff[cOrder[0]] * goalPotValue[cOrder[0]]) / coeff[cOrder[1]];
    }
    
    //Choose a value for the second potentiometer within the bounds
    randomSeed(analogRead(A3)*millis());
    if (goalPotBounds[0] > goalPotBounds[1]) {
      goalPotValue[cOrder[1]] = (random(100) / 100.0) * (goalPotBounds[0] - goalPotBounds[1]) + goalPotBounds[1];
    }
    else {
      goalPotValue[cOrder[1]] = (random(100) / 100.0) * (goalPotBounds[1] - goalPotBounds[0]) + goalPotBounds[0];
    }
    
    //Choose value for third potentiometer which satisfies the equation f1(x,y,z) = Ax + By + Cz + D
    goalPotValue[cOrder[2]] = (goal[0] - coeff[cOrder[0]] * goalPotValue[cOrder[0]] - coeff[cOrder[1]] * goalPotValue[cOrder[1]] - coeff[3]) / coeff[cOrder[2]];
    
    //Check if any values for x, y, or z are outside of bounds
    for (int i = 0; i < 3; i++) {
      if (goalPotValue[i] < 0.0 || goalPotValue[i] > 1.0) error("Goal potentiometer value out of bounds");
    }
      
    //Check if values of x, y, and z are truly a solution for f(x,y,z) = goal[0]
    if (pow(goal[0] - coeff[0] * goalPotValue[0] - coeff[1] * goalPotValue[1] - coeff[2] * goalPotValue[2] - coeff[3],2) > 0.001) error("Goal potentiometer values are not a solution with given coefficients.");
    
    //Choose order to select meter two coefficients
    randomizeCOrder(cOrder);
    
    //Bump up coefficient numbers since I'm getting the coefficients for the second meter
    for (int i = 0; i < 3; i++) {
      cOrder[i] = cOrder[i] + 4;
    }
    
    //There are situations where my solution method cannot find a set of coefficients for the second panel meter which lead to a solution.
    //It seems fairly rare, and at first glance it has something to do with the three values of goalPotValue being too close together.
    //Rather than figure it out, I just make a guess at the first coefficient. If it turns out to be wrong, just iterate until a solution is found.
    randomSeed(analogRead(A3)*millis());
    coeff[cOrder[0]] = (random(100) / 100.0) * (2 * fMax * fMaxPercent) - (fMax * fMaxPercent);
    
    //Choose a solution order
    randomSeed(analogRead(A3)*millis());
    sOrder[0] = random(4);
    sOrder[1] = random(4);
    sOrder[2] = random(4);
    while (sOrder[1] == sOrder[0]) {
      sOrder[1] = random(4);
    }
    while (sOrder[2] == sOrder[0] || sOrder[2] == sOrder[1]) {
      sOrder[2] = random(4);
    }
    sOrder[3] = 6 - sOrder[0] - sOrder[1] - sOrder[2];
    
    //There are four potential solutions with the following assumptions: +B and +C, -B and +C, +B and -C, -B and -C
    //Each solution is attempted per the order randomly selected above, and then the assumptions are verified
    //If the assumptions are correct, the solution exists.
    j = 0; //Counter for loop
    while (j < 4 && secondSolution == 0) {
      
      if (sOrder[j] == 0) { //Assume B and C are positive
        coeff[cOrder[1]] = (goal[1] - coeff[cOrder[0]] * goalPotValue[cOrder[0] - 4] - goalPotValue[cOrder[2] - 4] * fMax + abs(coeff[cOrder[0]]) * goalPotValue[cOrder[2] - 4] - coeff[cOrder[0]] * v(coeff[cOrder[0]])) / (goalPotValue[cOrder[1] - 4] - goalPotValue[cOrder[2] - 4]);
        coeff[cOrder[2]] = fMax - abs(coeff[cOrder[0]]) - abs(coeff[cOrder[1]]);
        coeff[7] = coeff[cOrder[0]] * v(coeff[cOrder[0]]) + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]);
      }
      
      if (sOrder[j] == 1) { //Assume -B, +C
        coeff[cOrder[1]] = (goal[1] - coeff[cOrder[0]] * goalPotValue[cOrder[0] - 4] - goalPotValue[cOrder[2] - 4] * fMax + abs(coeff[cOrder[0]]) * goalPotValue[cOrder[2] - 4] - coeff[cOrder[0]] * v(coeff[cOrder[0]])) / (goalPotValue[cOrder[1] - 4] + goalPotValue[cOrder[2] - 4] - 1);
        coeff[cOrder[2]] = fMax - abs(coeff[cOrder[0]]) - abs(coeff[cOrder[1]]);
        coeff[7] = coeff[cOrder[0]] * v(coeff[cOrder[0]]) + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]);
      }
      
      if (sOrder[j] == 2) { //Assume +B, -C
        coeff[cOrder[1]] = (goal[1] - coeff[cOrder[0]] * goalPotValue[cOrder[0] - 4] + goalPotValue[cOrder[2] - 4] * fMax - abs(coeff[cOrder[0]]) * goalPotValue[cOrder[2] - 4] - coeff[cOrder[0]] * v(coeff[cOrder[0]]) + abs(coeff[cOrder[0]]) - fMax) / (goalPotValue[cOrder[1] - 4] + goalPotValue[cOrder[2] - 4] - 1);
        coeff[cOrder[2]] = -fMax + abs(coeff[cOrder[0]]) + abs(coeff[cOrder[1]]);
        coeff[7] = coeff[cOrder[0]] * v(coeff[cOrder[0]]) + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]);
      }
      
      if (sOrder[j] == 3) { //Assume -B, -C
        coeff[cOrder[1]] = (goal[1] - coeff[cOrder[0]] * goalPotValue[cOrder[0] - 4] + goalPotValue[cOrder[2] - 4] * fMax - abs(coeff[cOrder[0]]) * goalPotValue[cOrder[2] - 4] - coeff[cOrder[0]] * v(coeff[cOrder[0]]) + abs(coeff[cOrder[0]]) - fMax) / (goalPotValue[cOrder[1] - 4] - goalPotValue[cOrder[2] - 4]);
        coeff[cOrder[2]] = -fMax + abs(coeff[cOrder[0]]) + abs(coeff[cOrder[1]]);
        coeff[7] = coeff[cOrder[0]] * v(coeff[cOrder[0]]) + coeff[cOrder[1]] * v(coeff[cOrder[1]]) + coeff[cOrder[2]] * v(coeff[cOrder[2]]);
      }
      
      if (pow(goal[1] - coeff[4] * goalPotValue[0] - coeff[5] * goalPotValue[1] - coeff[6] * goalPotValue[2] - coeff[7], 2) < 0.05) {
        secondSolution = 1;
      }
      
      j = j + 1;
    }
  }
  
  //Light LEDs
  for (int i = 0; i < 2; i++){
      digitalWrite(pLED[goalLED[i] + (5 * i)], HIGH);
  }
  
  boolean solved[2] = {false, false};
  
  //Loop during solving process
  while(solved[0] == false || solved[1] == false) {
  
    //Get potentiometer values  
    getPotValues(potValue, 3);
    
    //Set panel meters
    for (int i = 0; i < 2; i++) {
      panelMeterValue[i] = 0;
    }
    
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        panelMeterValue[i] = panelMeterValue[i] + potValue[j] * coeff[j + (i * 4)];
      }
      panelMeterValue[i] = panelMeterValue[i] + coeff[3 + (i * 4)];
    }

    for (int i = 0; i < 2; i++) {
      analogWrite(pMeter[i], panelMeterValue[i] / 4);
      
      if (pow(panelMeterValue[i] - goal[i],2) < 5) {
        solved[i] = true;
      }
    }
    
    //Buzz if solved
    if (solved[0] == true && solved[1] == true) {
      digitalWrite(pVib, HIGH);
      delay(500);
      digitalWrite(pVib, LOW);
      delay(100);
      digitalWrite(pVib, HIGH);
      delay(500);
      digitalWrite(pVib, LOW);
    }
    
    //Reset button
    if (digitalRead(rButton) == HIGH) {
      while (digitalRead(rButton) == HIGH) {
        //Pause until button is let go
      }
      solved[0] = true;
      solved[1] = true;
    }
    
    delay(75);
    
  }
  
  delay(100);
}

void getPotValues(float theArray[], int n) //Get the potentiometer values
{
  for(int i = 0; i < n; i++){
    theArray[i] = analogRead(pot[i]) / 1023.0;
  }
}

void randomizeCOrder(int theArray[]) //Randomize coefficient order
{
  randomSeed(analogRead(A3)*millis());
  theArray[0] = random(3);
  theArray[1] = random(3);
  while(theArray[1] == theArray[0]){
    theArray[1] = random(3);
  }
  theArray[2] = 3 - theArray[0] - theArray[1];
}

/*
Unit step function
u = { 0; for n <= 0
      1; for n > 0
    }
*/
int u(double n)
{
  int u = 0;
  if (n > 0) u = 1;
  return u;
}

/*
Unit step function minus 1
v = { 0; for n > 0
     -1; for n <= 0
    }
*/
int v(double n) //Opposite unit step function
{
  int v = -1;
  if (n > 0) v = 0;
  return v;
}
