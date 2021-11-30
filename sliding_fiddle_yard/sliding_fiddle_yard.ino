/*
 * This code has been developed by PHILLIP EDWARDS a member of MERG with help from others. It is used to control
 * a sliding fiddle yard which has 16 tracks in and 15 tracks out each track has access to 6 other tracks
 * it moves the fiddle yard using a stepper motor
 * You will need to down load the AccelStepper.h library to use it uses a Nema 17 stepper motor and A4988 stepper drivers 
 */


#include <Wire.h>
#include <Auto485.h>
#include <CMRI.h>
#include <AccelStepper.h>

#define dirPin  3          //A4988 Direction Pin 
#define stepPin 4



//A4988 Step Pin
#define motorInterfaceType 1 // Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:

Auto485 bus(2);              // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(6, 0, 192, bus);   // Node setup addeess 6

int endStop = 6;                                   // Connect output to push button & end stop switch
int relayPin = 5;                                  // SFY relay to change track polarity
int constexpr SFY_unobstructed_detector = 7;       // All SLF IRD's connected to this pin
long initial_homing = -1;                          // Used to Home Stepper at startup
int value = 0;
int SFYclear = 8;                                  //Connect to Nano7 Pin 3 to report back to JMRI that SFY is not obstructed
int SFYmoving = 9;                                 //Connect to Nano7 pin 4 to report back to JMRI SFY is moving
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);  //defines stepper setup

void setup() {
  pinMode(relayPin, OUTPUT);                      // Set pin as output
  digitalWrite(relayPin, HIGH);                   // Set initial state OFF for low trigger relay
  pinMode(SFY_unobstructed_detector, INPUT);      // IRD's Connected to this pin
  pinMode (SFYclear, OUTPUT);                     // Set pin as output
  digitalWrite (SFYclear, LOW);                   // Set initial state to off
  pinMode(endStop, INPUT_PULLUP);                 // Endstop switch
  pinMode (SFYmoving, OUTPUT);                    // Set pin as output
  digitalWrite (SFYmoving, HIGH); 
  Serial.begin(115200);
  bus.begin(115200, SERIAL_8N2);                  // Open the RS485 bus at *****bps
 
// Home the SFY

  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(14000);

  while (digitalRead(endStop)) {
    stepper.moveTo(initial_homing);              // Read the state of endstop
    initial_homing--;                            // Decrease by 1 for next move if needed
    stepper.run();                               // Start moving the stepper

  }
  while (!digitalRead(endStop)) {                // Read the state of endstop
    stepper.moveTo(initial_homing);              // Make the Stepper move CW until the switch is deactivated
    stepper.run();
    initial_homing++;
  }
  stepper.setCurrentPosition(0);                 // End stop reached position zero
  //stepper.runToNewPosition(3000);              // Set this to move the SFY to a new starting position after homing complete
  int position = 0;                              // Sets the position of Steppers to zero
  Serial.println("Homing Completed");

}



int update_value_from_cmri(int original_value)   // Define case numbers to correspond with CMRI bit numbers in JMRI
{
  int static value_for_cmri_bit[] = { 11, 12, 13, 14, 15, 16, 17, //CMRI bit 6001-6007
                                      21, 22, 23, 24, 25, 26, 27, 28, //CMRI bit 6008-6015
                                      31, 32, 33, 34, 35, 36, 37, 38, 39, //CMRI bit 6016-6024
                                      41, 42, 43, 44, 45, 46, 47, 48, 49, 410, //CMRI bit 6025-6034
                                      51, 52, 53, 54, 55, 56, 57, 58, 59, 510, 511, 512, //CMRI bit 6035-6046
                                      61, 62, 63, 64, 65, 66, 67, 68, 69, 610, 611, 612, 613, //CMRI bit 6047-6059
                                      76, 77, 78, 79, 710, 711, 712, 713, 714, 715, 716, 717, 718, //CMRI bit 660-6072
                                      89, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819, 820, 821, //CMRI bit 6073-6085
                                      912, 913, 914, 915, 916, 917, 918, 919, 920, 921, 922, 923, 924, //CMRI bit 6086-6098
                                      1016, 1017, 1018, 1019, 1020, 1021, 1022, 1023, 1024, 1025, 1026, 1027, //CMRI bit 6099-6110
                                      1118, 1119, 1120, 1121, 1122, 1123, 1124, 1125, 1126, 1127, 1128, 1129, //CMRI bit 6111-6122
                                      1220, 1221, 1222, 1223, 1224, 1225, 1226, 1227, 1228, 1229, 1230, 1231, 1232, // CMRI bit 6123-6135
                                      1324, 1325, 1326, 1327, 1328, 1329, 1330, 1331, 1332, //CMRI bit 6136-6144
                                      1425, 1426, 1427, 1428, 1429, 1430, 1431, 1432, //CMRI bit 6145-6152
                                      1526, 1527, 1528, 1529, 1530, 1531, 1532, //CMRI bit 6153-6159
                                      1627, 1628, 1629, 1630, 1631, 1632, //CMRI bit 6160-6165
                                    };
  int constexpr number_of_cmri_bits =
    sizeof(value_for_cmri_bit) / sizeof(value_for_cmri_bit[0]);

    cmri.process();

  for (int cmri_bit = 0; cmri_bit < number_of_cmri_bits; cmri_bit++)
  {
    if (cmri.get_bit(cmri_bit) == 1)
    {
      return value_for_cmri_bit[cmri_bit];
    }
  }

  return original_value;
}

void move_stepper_to_position_for_value(int value) //Max49500 steps to reach SFY extent Min
{
  switch (value)
  { 
    case 11:
      {stepper.runToNewPosition(24470);    // When stepper is running it blocks all commands untill it has reached its new position.
        digitalWrite (relayPin, LOW);      // Turns relay off/on to change track polarity.
        break;
      }
    case 12:
      { stepper.runToNewPosition(28430);
        digitalWrite (relayPin, LOW);
        digitalWrite (SFYmoving, LOW);                 // Set pin 9 low
        break;
      }
    case 13:
      { stepper.runToNewPosition(32360);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 14:
      { stepper.runToNewPosition(36290);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 15:
      { stepper.runToNewPosition(40220);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 16:
      { stepper.runToNewPosition(44150);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 17:
      { stepper.runToNewPosition(48080);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 21:
      { stepper.runToNewPosition(20310);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 22:
      { stepper.runToNewPosition(24240);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 23:
      { stepper.runToNewPosition(28170);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 24:
      { stepper.runToNewPosition(32100);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 25:
      { stepper.runToNewPosition(36030);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 26:
      { stepper.runToNewPosition(39960);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 27:
      { stepper.runToNewPosition(43890);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 28:
      { stepper.runToNewPosition(47820);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 31:
      { stepper.runToNewPosition(16290);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 32:
      { stepper.runToNewPosition(20220);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 33:
      { stepper.runToNewPosition(24150);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 34:
      { stepper.runToNewPosition(28080);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 35:
      { stepper.runToNewPosition(32030);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 36:
      { stepper.runToNewPosition(35960);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 37:
      { stepper.runToNewPosition(39890);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 38:
      { stepper.runToNewPosition(43800);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 39:
      { stepper.runToNewPosition(47690);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 41:
      { stepper.runToNewPosition(10890);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 42:
      { stepper.runToNewPosition(14780);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 43:
      { stepper.runToNewPosition(18710);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 44:
      { stepper.runToNewPosition(22640);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 45:
      { stepper.runToNewPosition(26570);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 46:
      { stepper.runToNewPosition(30500);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 47:
      { stepper.runToNewPosition(34430);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 48:
      { stepper.runToNewPosition(38360);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 49:
      { stepper.runToNewPosition(42290);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 410:
      { stepper.runToNewPosition(46220);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 51:
      { stepper.runToNewPosition(5090);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 52:
      { stepper.runToNewPosition(9060);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 53:
      { stepper.runToNewPosition(12990);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 54:
      { stepper.runToNewPosition(16940);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 55:
      { stepper.runToNewPosition(20850);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 56:
      { stepper.runToNewPosition(24780);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 57:
      { stepper.runToNewPosition(28700);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 58:
      { stepper.runToNewPosition(32630);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 59:
      { stepper.runToNewPosition(36540);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 510:
      { stepper.runToNewPosition(40485);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 511:
      { stepper.runToNewPosition(44410);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 512:
      { stepper.runToNewPosition(48340);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 61:
      { stepper.runToNewPosition(930);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 62:
      { stepper.runToNewPosition(4860);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 63:
      { stepper.runToNewPosition(8790);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 64:
      { stepper.runToNewPosition(12720);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 65:
      { stepper.runToNewPosition(16650);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 66:
      { stepper.runToNewPosition(20580);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 67:
      { stepper.runToNewPosition(24510);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 68:
      { stepper.runToNewPosition(28440);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 69:
      { stepper.runToNewPosition(32370);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 610:
      { stepper.runToNewPosition(36320);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 611:
      { stepper.runToNewPosition(40230);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 612:
      { stepper.runToNewPosition(44160);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 613:
      { stepper.runToNewPosition(48090);
       digitalWrite (relayPin, LOW);
        break;
      }
    case 76:
      { stepper.runToNewPosition(1090);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 77:
      { stepper.runToNewPosition(5020);
      digitalWrite (relayPin, HIGH);
       break;
      }
    case 78:
      { stepper.runToNewPosition(8950);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 79:
      { stepper.runToNewPosition(12880);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 710:
      { stepper.runToNewPosition(16790);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 711:
      { stepper.runToNewPosition(20740);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 712:
      { stepper.runToNewPosition(24640);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 713:
      { stepper.runToNewPosition(28540);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 714:
      { stepper.runToNewPosition(32470);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 715:
      { stepper.runToNewPosition(36400);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 716:
      { stepper.runToNewPosition(40330);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 717:
      { stepper.runToNewPosition(44260);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 718:
      { stepper.runToNewPosition(48190);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 89:
      { stepper.runToNewPosition(1810);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 810:
      { stepper.runToNewPosition(5740);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 811:
      { stepper.runToNewPosition(9670);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 812:
      { stepper.runToNewPosition(13600);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 813:
      { stepper.runToNewPosition(17530);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 814:
      { stepper.runToNewPosition(21460);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 815:
      { stepper.runToNewPosition(25390);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 816:
      { stepper.runToNewPosition(29320);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 817:
      { stepper.runToNewPosition(33250);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 818:
      { stepper.runToNewPosition(37180);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 819:
      { stepper.runToNewPosition(41110);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 820:
      { stepper.runToNewPosition(45040);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 821:
      { stepper.runToNewPosition(48970);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 912:
      { stepper.runToNewPosition(890);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 913:
      { stepper.runToNewPosition(4820);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 914:
      { stepper.runToNewPosition(8750);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 915:
      { stepper.runToNewPosition(12680);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 916:
      { stepper.runToNewPosition(16610);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 917:
      { stepper.runToNewPosition(20540);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 918:
      { stepper.runToNewPosition(24470);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 919:
      { stepper.runToNewPosition(28400);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 920:
      { stepper.runToNewPosition(32330);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 921:
      { stepper.runToNewPosition(36260);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 922:
      { stepper.runToNewPosition(40190);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 923:
      { stepper.runToNewPosition(44120);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 924:
      { stepper.runToNewPosition(48050);
      digitalWrite (relayPin, LOW);
        break;
      }
    case 1016:
      { stepper.runToNewPosition(2890);
       digitalWrite (relayPin, HIGH);
        break;
      }
    case 1017:
      { stepper.runToNewPosition(6860);
       digitalWrite (relayPin, HIGH);
        break;
      }
    case 1018:
      { stepper.runToNewPosition(10780);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1019:
      { stepper.runToNewPosition(14710);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1020:
      { stepper.runToNewPosition(18640);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1021:
      { stepper.runToNewPosition(22570);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1022:
      { stepper.runToNewPosition(26500);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1023:
      { stepper.runToNewPosition(30430);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1024:
      { stepper.runToNewPosition(34360);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1025:
      { stepper.runToNewPosition(38290);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1026:
      { stepper.runToNewPosition(42220);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1027:
      { stepper.runToNewPosition(46150);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1118:
      { stepper.runToNewPosition(1850);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1119:
      { stepper.runToNewPosition(5780);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1120:
      { stepper.runToNewPosition(9710);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1121:
      { stepper.runToNewPosition(13620);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1122:
      { stepper.runToNewPosition(17570);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1123:
      { stepper.runToNewPosition(21500);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1124:
      { stepper.runToNewPosition(25430);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1125:
      { stepper.runToNewPosition(29360);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1126:
      { stepper.runToNewPosition(33290);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1127:
      { stepper.runToNewPosition(37220);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1128:
      { stepper.runToNewPosition(41150);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1129:
      { stepper.runToNewPosition(45040);
      digitalWrite (relayPin, HIGH);
        break;
      }
//    case 1220:
//      { stepper.runToNewPosition(2222);//Not used
//        break;
//      }
    case 1221:
      { stepper.runToNewPosition(3818);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1222:
      { stepper.runToNewPosition(7748);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1223:
      { stepper.runToNewPosition(11678);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1224:
      { stepper.runToNewPosition(15608);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1225:
      { stepper.runToNewPosition(19538);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1226:
      { stepper.runToNewPosition(23468);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1227:
      { stepper.runToNewPosition(27398);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1228:
      { stepper.runToNewPosition(31328);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1229:
      { stepper.runToNewPosition(35258);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1230:
      { stepper.runToNewPosition(39188);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1231:
      { stepper.runToNewPosition(43118);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1232:
      { stepper.runToNewPosition(47048);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1324:
      { stepper.runToNewPosition(2180);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1325:
      { stepper.runToNewPosition(6070);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1326:
      { stepper.runToNewPosition(10000);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1327:
      { stepper.runToNewPosition(13930);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1328:
      { stepper.runToNewPosition(17860);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1329:
      { stepper.runToNewPosition(21790);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1330:
      { stepper.runToNewPosition(25720);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1331:
      { stepper.runToNewPosition(29650);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1332:
      { stepper.runToNewPosition(33580);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1425:
      { stepper.runToNewPosition(1880);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1426:
      { stepper.runToNewPosition(5810);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1427:
      { stepper.runToNewPosition(9740);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1428:
      { stepper.runToNewPosition(13670);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1429:
      { stepper.runToNewPosition(17600);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1430:
      { stepper.runToNewPosition(21530);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1431:
      { stepper.runToNewPosition(25460);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1432:
      { stepper.runToNewPosition(29390);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1526:
      { stepper.runToNewPosition(1460);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1527:
      { stepper.runToNewPosition(5390);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1528:
      { stepper.runToNewPosition(9320);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1529:
      { stepper.runToNewPosition(13250);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1530:
      { stepper.runToNewPosition(17180);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1531:
      { stepper.runToNewPosition(21110);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1532:
      { stepper.runToNewPosition(25040);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1627:
      { stepper.runToNewPosition(1230);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1628:
      { stepper.runToNewPosition(5200);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1629:
      { stepper.runToNewPosition(9130);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1630:
      { stepper.runToNewPosition(13060);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1631:
      { stepper.runToNewPosition(16990);
      digitalWrite (relayPin, HIGH);
        break;
      }
    case 1632:
      { stepper.runToNewPosition(20920);
       digitalWrite (relayPin, HIGH);  
        break;
      }
      
      // cmri.set_bit(0, !digitalRead(HIGH));

     
    default:
      {
        Serial.print("Invalid position request: ");
        Serial.println(value);
        break;
      }
  } 
}


void loop()
{         
value = update_value_from_cmri(value);

  if (value > 0)
  {
    if (LOW == digitalRead(SFY_unobstructed_detector))
    {
      digitalWrite (SFYmoving, HIGH);                // Set pin 9 high and stays high untill stepper has moved into position
      move_stepper_to_position_for_value(value);
      digitalWrite (SFYmoving, LOW);                 // Set pin 9 low 
      digitalWrite (SFYclear, LOW);                  // Set sensor SFY Obstructed in panelPro to " SFY Clear"
      Serial.println("Clear To Move");
    }
    else
    {
      digitalWrite (SFYclear, HIGH);                 // Set sensor SFY Obstructed in panelPro to " SFY Obstructed"         
      Serial.println("Obstruction");
    }
  }
}
