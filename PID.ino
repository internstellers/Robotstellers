/*

* PID3piLineFollower - demo code for the Pololu 3pi Robot

*

 * This code will follow a black line on a white background, using a

* PID-based algorithm.

*

* http://www.pololu.com/docs/0J21

* http://www.pololu.com

* http://forum.pololu.com

*

*/

 

// The following libraries will be needed by this demo

#include <Pololu3pi.h>

#include <PololuQTRSensors.h>

#include <OrangutanMotors.h>

#include <OrangutanAnalog.h>

#include <OrangutanLEDs.h>

#include <OrangutanLCD.h>

#include <OrangutanPushbuttons.h>

#include <OrangutanBuzzer.h>

 

Pololu3pi robot;

unsigned int sensors[5]; // an array to hold sensor values

unsigned int last_proportional = 0;

long integral = 0;

 

// This include file allows data to be stored in program space.  The

// ATmega168 has 16k of program space compared to 1k of RAM, so large

// pieces of static data should be stored in program space.

#include <avr/pgmspace.h>

 

// Introductory messages.  The "PROGMEM" identifier causes the data to

// go into program space.

const char welcome_line1[] PROGMEM = "Intern";

const char welcome_line2[] PROGMEM = "stellars";

const char demo_name_line1[] PROGMEM = "JPMC ";

const char demo_name_line2[] PROGMEM = "Winners";

 

// A couple of simple tunes, stored in program space.

const char contraPause[] PROGMEM = "!V15 T165 >>c16 >>g16 >>e16 >>>c4";

const char contra[] PROGMEM = "!V15 T165" \
">f16 >e-16 >c16 b-16 >c16 b-16 a-16 g16 a-16 g16 f16 e-16 f16" \
"<a-16 c16 e-16" \
"f2 >c8. b-16 >c8. >d16 >e-1 f2 >c8. b-16 >c8. >d16 a-1" \
"f2 >c8. b-16 >c8. >d16 >e-1 f2 >c8. b-16 >c8. >d16 a-1 >c8 >c8. >d8. >e-8 >e-8" \
"r16 g16 b-16 >c16 >d8 >d8 >d16 >e-8. >f8 >f8 >e-16 >f8. >c8 >c8. >d8. >e-8 >e-8 r16" \
"g16 b-16 >c16 >d8 >d8 >d16 >e-8. >f4 >b-4 >g16 >f16 >>d16 >>c16 >b-16 >a16 >g16 >f16" \
">g16 >f16 >d16 >c16 >d16 >c16 b-16 a16 c4 c16 e-16 g8 f8 e16 d8 <b-16 c1" \
"<a4 <b-16 c16 r8 d4 e-16 f16 r8 g4 a16 b-16 r8 >c16 >d16 r8 >c16 c16" \
"e-16 f16 g16 f16 g16 c16 c16 c16 e-16 f16 g8 b-16 a16 b-16 a8 f16" \
"a16 f16 a16 c16 c16 c16 e-16 f16 g8 b-16 a16 b-16 >c8 >d16 <<a4 <<b-16" \
"c8. d4 e-16 f8. g16 g16 g8 a16 b-16. >d4 >c4 b-16 b-16 b-16 b-16 r8" \
"b-16 b-16 b-16 b-8 b-16 r16 a8. g16 g16 g16 g16 r8 g16 g16 g16 g8 g16" \
"r16 a8. b-16 b-16 b-16 b-16 r8 b-16 b-16 b-16 b-8 b-16 r16 a8. g16 g16" \
"g16 g16 r16 r16 g8 a16 b-16 >c16 >d16 >c16 r16 a16 >d16 >d16 >d16 >d16" \
"r8 >d16 >d16 >d16 >d8 >d16 r16 >c8. b-16 b-16 b-16 b-16 r8 b-16 b-16 b-16" \
"b-8 b-16 r16 >c8. >d16 >d16 >d16 >d16 r8 >d16 >d16 >d16 >d8 >d16 r16 >e-8 >f16" \
">f16 >f16 >f16 r8 >f16 >f16 >f16 >f16 r8 r16 c16 e-16 f16 g16 f16 g16 c8 c16" \
"e-16 f16 g16 f16 g16 b-8 b-16 a16 b-16 g16 f16 g16 c8 c16 e-16 f16 r16" \
"g16 r16 a16 r16 b-8." \
"f2 >c8. b-16 >c8. >d16 >e-1 f2 >c8. b-16 >c8. >d16 a-1" \
"f2 >c8. b-16 >c8. >d16 >e-1 f2 >c8. b-16 >c8. >d16 a-1 >c8 >c8. >d8. >e-8 >e-8" \
"r16 g16 b-16 >c16 >d8 >d8 >d16 >e-8. >f8 >f8 >e-16 >f8. >c8 >c8. >d8. >e-8 >e-8 r16" \
"g16 b-16 >c16 >d8 >d8 >d16 >e-8. >f4 >b-4 >g16 >f16 >>d16 >>c16 >b-16 >a16 >g16 >f16" \
">g16 >f16 >d16 >c16 >d16 >c16 b-16 a16 c4 c16 e-16 g8 f8 e16 d8 <b-16 c1" \
"<a4 <b-16 c16 r8 d4 e-16 f16 r8 g4 a16 b-16 r8 >c16 >d16 r8 >c16 c16" \
"e-16 f16 g16 f16 g16 c16 c16 c16 e-16 f16 g8 b-16 a16 b-16 a8 f16" \
"a16 f16 a16 c16 c16 c16 e-16 f16 g8 b-16 a16 b-16 >c8 >d16 <<a4 <<b-16" \
"c8. d4 e-16 f8. g16 g16 g8 a16 b-16. >d4 >c4 b-16 b-16 b-16 b-16 r8" \
"b-16 b-16 b-16 b-8 b-16 r16 a8. g16 g16 g16 g16 r8 g16 g16 g16 g8 g16" \
"r16 a8. b-16 b-16 b-16 b-16 r8 b-16 b-16 b-16 b-8 b-16 r16 a8. g16 g16" \
"g16 g16 r16 r16 g8 a16 b-16 >c16 >d16 >c16 r16 a16 >d16 >d16 >d16 >d16" \
"r8 >d16 >d16 >d16 >d8 >d16 r16 >c8. b-16 b-16 b-16 b-16 r8 b-16 b-16 b-16" \
"b-8 b-16 r16 >c8. >d16 >d16 >d16 >d16 r8 >d16 >d16 >d16 >d8 >d16 r16 >e-8 >f16" \
">f16 >f16 >f16 r8 >f16 >f16 >f16 >f16 r8 r16 c16 e-16 f16 g16 f16 g16 c8 c16" \
"e-16 f16 g16 f16 g16 b-8 b-16 a16 b-16 g16 f16 g16 c8 c16 e-16 f16 r16" \
"g16 r16 a16 r16 b-8.";

 

// Data for generating the characters used in load_custom_characters

// and display_readings.  By reading levels[] starting at various

// offsets, we can generate all of the 7 extra characters needed for a

// bargraph.  This is also stored in program space.

const char levels[] PROGMEM = {

  0b00000,

  0b00000,

  0b00000,

  0b00000,

  0b00000,

  0b00000,

  0b00000,

  0b11111,

  0b11111,

  0b11111,

  0b11111,

  0b11111,

  0b11111,

  0b11111

};

 

// This function loads custom characters into the LCD.  Up to 8

// characters can be loaded; we use them for 7 levels of a bar graph.

void load_custom_characters()

{

  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar

  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars

  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...

  OrangutanLCD::loadCustomCharacter(levels + 3, 3);

  OrangutanLCD::loadCustomCharacter(levels + 4, 4);

 OrangutanLCD::loadCustomCharacter(levels + 5, 5);

  OrangutanLCD::loadCustomCharacter(levels + 6, 6);

  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect

}

 

// This function displays the sensor readings using a bar graph.

void display_readings(const unsigned int *calibrated_values)

{

  unsigned char i;

 

  for (i=0;i<5;i++) {

    // Initialize the array of characters that we will use for the

    // graph.  Using the space, an extra copy of the one-bar

    // character, and character 255 (a full black box), we get 10

    // characters in the array.

    const char display_characters[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255 };

 

    // The variable c will have values from 0 to 9, since

    // calibrated values are in the range of 0 to 1000, and

    // 1000/101 is 9 with integer math.

    char c = display_characters[calibrated_values[i] / 101];

 

    // Display the bar graph character.

    OrangutanLCD::print(c);

  }

}

 

// Initializes the 3pi, displays a welcome message, calibrates, and

// plays the initial music.  This function is automatically called

// by the Arduino framework at the start of program execution.

void setup()

{

  unsigned int counter; // used as a simple timer

 

  // This must be called at the beginning of 3pi code, to set up the

  // sensors.  We use a value of 2000 for the timeout, which

  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.

  robot.init(2000);

 

  load_custom_characters(); // load the custom characters

 

  // Play welcome music and display a message

  OrangutanLCD::printFromProgramSpace(welcome_line1);

  OrangutanLCD::gotoXY(0, 1);

  OrangutanLCD::printFromProgramSpace(welcome_line2);

  OrangutanBuzzer::playFromProgramSpace(contraPause);

  delay(1000);

 

  OrangutanLCD::clear();

  OrangutanLCD::printFromProgramSpace(demo_name_line1);

  OrangutanLCD::gotoXY(0, 1);

  OrangutanLCD::printFromProgramSpace(demo_name_line2);

  delay(1000);

 

  // Display battery voltage and wait for button press

  while (!OrangutanPushbuttons::isPressed(BUTTON_B))

  {

    int bat = OrangutanAnalog::readBatteryMillivolts();

 

    OrangutanLCD::clear();

    OrangutanLCD::print(bat);

    OrangutanLCD::print("mV");

    OrangutanLCD::gotoXY(0, 1);

    OrangutanLCD::print("Press B");

 

    delay(100);

  }

 

  // Always wait for the button to be released so that 3pi doesn't

  // start moving until your hand is away from it.

  OrangutanPushbuttons::waitForRelease(BUTTON_B);

  delay(1000);

 

  // Auto-calibration: turn right and left while calibrating the

  // sensors.

  for (counter=0; counter<80; counter++)

  {

    if (counter < 20 || counter >= 60)

      OrangutanMotors::setSpeeds(40, -40);

    else

      OrangutanMotors::setSpeeds(-40, 40);

 

    // This function records a set of sensor readings and keeps

    // track of the minimum and maximum values encountered.  The

    // IR_EMITTERS_ON argument means that the IR LEDs will be

    // turned on during the reading, which is usually what you

    // want.

    robot.calibrateLineSensors(IR_EMITTERS_ON);

 

    // Since our counter runs to 80, the total delay will be

    // 80*20 = 1600 ms.

    delay(20);

  }

  OrangutanMotors::setSpeeds(0, 0);

 

  // Display calibrated values as a bar graph.
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))

  {

    // Read the sensor values and get the position measurement.

    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

 

    // Display the position measurement, which will go from 0

    // (when the leftmost sensor is over the line) to 4000 (when

    // the rightmost sensor is over the line) on the 3pi, along

    // with a bar graph of the sensor readings.  This allows you

    // to make sure the robot is ready to go.

    OrangutanLCD::clear();

    OrangutanLCD::print(position);

    OrangutanLCD::gotoXY(0, 1);

    display_readings(sensors);

    if(OrangutanPushbuttons::isPressed(BUTTON_A))
    {
        OrangutanBuzzer::playFromProgramSpace(contra);
        while(!OrangutanPushbuttons::isPressed(BUTTON_C) && OrangutanBuzzer::isPlaying());
        OrangutanBuzzer::stopPlaying();
    }


    delay(100);

  }

  OrangutanPushbuttons::waitForRelease(BUTTON_B);

 

  OrangutanLCD::clear();

 

  OrangutanLCD::print("Internsteller!");                

 

  // Play music and wait for it to finish before we start driving.

  OrangutanBuzzer::playFromProgramSpace(contra);

  

}

 

// The main function.  This function is repeatedly called by

// the Arduino framework.

 

int integralCoefficient = 1;

void loop()

{

  // Get the position of the line.  Note that we *must* provide

  // the "sensors" argument to read_line() here, even though we

  // are not interested in the individual sensor readings.

  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

 

  // The "proportional" term should be 0 when we are on the line.

  int proportional = (int)position - 2000;

 

  // Compute the derivative (change) and integral (sum) of the

  // position.

  int derivative = proportional - last_proportional;

  integral += proportional;

 

  // Remember the last position.

  last_proportional = proportional;

 

  // Compute the difference between the two motor power settings,

  // m1 - m2.  If this is a positive number the robot will turn

  // to the right.  If it is a negative number, the robot will

  // turn to the left, and the magnitude of the number determines

  // the sharpness of the turn.  You can adjust the constants by which

  // the proportional, integral, and derivative terms are multiplied to

  // improve performance.

//  int power_difference = proportional/20 + integral/10000 + derivative*3/2;

 

    if((float(integral/10000)/((proportional))) > .5) {

        integral = 0;

        int randNum = random(1, 3);

        if(randNum % 2 == 0) {

          //integralCoefficient = (-1)*integralCoefficient;

        }

    }

    int power_difference = proportional + (integralCoefficient * integral)/10000 + derivative*3/5;

  //  if (counter > 90)

  //    integral =0;

  //    counter = 0;

   // counter++;

  // Compute the actual motor settings.  We never set either motor

  // to a negative value.

  const int maximum = 170;

  if (power_difference > maximum)

    power_difference = maximum;

  if (power_difference < -maximum)

    power_difference = -maximum;

 

  if (power_difference < 0)

    OrangutanMotors::setSpeeds(maximum + power_difference, maximum);

  else

    OrangutanMotors::setSpeeds(maximum, maximum - power_difference);

}