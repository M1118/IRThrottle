#include <Wire.h>


#include <Arduino.h>
#include <SPI.h>
#include <MergCBUS.h>
#include <Message.h>
#include <MergCBUSThrottle.h>
#include <EEPROM.h>
#include <IRremote2.h>



/**
   The following block of #defines configures the pins that are
   used for various special functions:
     CHIPSELECT  is the select pin for the CANBUS interface
     INTPIN      is the "interrupt" pin used by the CANBUS interface
                 to signal that a CANBUS packet is ready to be read
     MODESELECT  is the jumper used to determine the operating mode
     RECV_PIN    is the IR reciever input
*/
#define LED           13     // Pin number for the LED
#define GREEN_LED      8    //merg green led port
#define YELLOW_LED     A0    //merg yellow led port
#define PUSH_BUTTON    7    //std merg push button
#define CANPORT       10
#define INTPIN         6
#define RECV_PIN       9


#define NUM_NODE_VARS  1    //the transition interval
#define NUM_EVENTS     20   //supported events
#define NUM_EVENT_VARS 0    //no need for supported event variables
#define NUM_DEVICES    1    //one device number
#define MODULE_ID      9   //module id
#define MANUFACTURER_ID 165 //manufacturer id
#define MIN_CODE       0    //min code version
#define MAX_CODE       1    //max code version

//throttle state
#define START 0
#define ACQUIRED 1
#define WAITING 2
#define SENT_SPEED 3
#define RUNNING 4
#define STOPPED 5
#define LOCO_RUN_TIME 10 //seconds

#define N_SESSIONS  2

#define UpKey 0
#define DownKey 1
#define LeftKey 2
#define RightKey 3

#define BUTTON_1        0xFFA25D
#define BUTTON_2        0xFF629D
#define BUTTON_3        0xFFE21D
#define BUTTON_4        0xFF22DD
#define BUTTON_5        0xFF02FD
#define BUTTON_6        0xFFC23D
#define BUTTON_7        0xFFE01F
#define BUTTON_8        0xFFA857
#define BUTTON_9        0xFF906F
#define BUTTON_0        0xFF9867
#define BUTTON_UP       0xFF18E7
#define BUTTON_DOWN     0xFF4AB5
#define BUTTON_LEFT     0xFF10EF
#define BUTTON_RIGHT    0xFF5AA5
#define BUTTON_OK       0xFF38C7
#define BUTTON_STAR     0xFF6897
#define BUTTON_HASH     0xFFB04F

//keep alive interval
#define KEEP_ALIVE_TIMEOUT 2000 //millisecs

MergCBUS cbus = MergCBUS(NUM_NODE_VARS, NUM_EVENTS, NUM_EVENT_VARS, NUM_DEVICES);
IRrecv irrecv(RECV_PIN);

decode_results results;

enum SessionState { SessionFree, SessionAttached };


typedef struct {
  uint8_t         session_id;     // CBUS session
  enum SessionState   state;
  int             throttle_state;
  int             previous_state;
  boolean         forwards;
  int             speed;
  uint16_t        locoNo;
  unsigned int    functions;
  MergCBUSThrottle *throttle;
} CAB_SESSION;

CAB_SESSION sessions[N_SESSIONS];
long last_time;
long stoptime;
int current_session = 0;


/**
   A number key has been pressed. Either a loco number is being entered
   or a function toggled.
*/
void NumberKey(int number) {
  if (sessions[current_session].state == SessionFree) {
    if (sessions[current_session].locoNo > 999) {
      sessions[current_session].locoNo = sessions[current_session].locoNo % 1000;
    }
    sessions[current_session].locoNo = (sessions[current_session].locoNo * 10) + number;
    updateDisplay();
  } else {
    unsigned int bitmask = 1 << number;
    if (sessions[current_session].functions & bitmask) {
      sessions[current_session].functions &= ~bitmask;
      functionOff(number);
    } else {
      sessions[current_session].functions |= bitmask;
      functionOn(number);
    }
  }
}

/**
   The OK key has been pressed. If the session is free then allocate one
   otherwise free the current session
*/
void OKKey() {
  if (sessions[current_session].state == SessionFree) {
    // Serial.print("Allocate session for: ");
    // Serial.println(sessions[current_session].locoNo);
    allocateSession();
    sessions[current_session].state = SessionAttached;
  } else {
    // Serial.println("Release session");
    releaseSession();
    sessions[current_session].state = SessionFree;
  }
  updateDisplay();
}

/**
   Arrow key pressed
*/
void ArrowKey(int key) {

  switch (key) {
    case UpKey:
      if (sessions[current_session].state != SessionFree) {
        sessions[current_session].speed++;
        if (sessions[current_session].speed < 0) {
          sessions[current_session].forwards = false;
        } else if (sessions[current_session].speed > 0) {
          sessions[current_session].forwards = true;
        }
        if (sessions[current_session].speed > 128) {
          sessions[current_session].speed = 128;
        }
        sessions[current_session].throttle->setSpeedDirection(sessions[current_session].session_id,
            sessions[current_session].forwards ? sessions[current_session].speed : -sessions[current_session].speed,
            sessions[current_session].forwards);
      }
      break;
    case DownKey:
      if (sessions[current_session].state != SessionFree) {
        sessions[current_session].speed--;
        if (sessions[current_session].speed < 0) {
          sessions[current_session].forwards = false;
        } else if (sessions[current_session].speed > 0) {
          sessions[current_session].forwards = true;
        }
        if (sessions[current_session].speed < -128) {
          sessions[current_session].speed = -128;
        }
        sessions[current_session].throttle->setSpeedDirection(sessions[current_session].session_id,
            sessions[current_session].forwards ? sessions[current_session].speed : -sessions[current_session].speed,
            sessions[current_session].forwards);
      }
      break;
    case LeftKey:
      current_session--;
      if (current_session < 0) {
        current_session = N_SESSIONS - 1;
      }
      break;
    case RightKey:
      current_session++;
      if (current_session >= N_SESSIONS) {
        current_session = 0;
      }
      break;
  }
  updateDisplay();
}

void irCommand(decode_results *results) {
  static int lastValue = 0;

  if (results->decode_type == NEC) {
    switch (results->value) {
      case BUTTON_0:
        NumberKey(0);
        break;
      case BUTTON_1:
        NumberKey(1);
        break;
      case BUTTON_2:
        NumberKey(2);
        break;
      case BUTTON_3:
        NumberKey(3);
        break;
      case BUTTON_4:
        NumberKey(4);
        break;
      case BUTTON_5:
        NumberKey(5);
        break;
      case BUTTON_6:
        NumberKey(6);
        break;
      case BUTTON_7:
        NumberKey(7);
        break;
      case BUTTON_8:
        NumberKey(8);
        break;
      case BUTTON_9:
        NumberKey(9);
        break;
      case BUTTON_OK:
        OKKey();
        break;
      case BUTTON_UP:
        ArrowKey(UpKey);
        break;
      case BUTTON_DOWN:
        ArrowKey(DownKey);
        break;
      case BUTTON_LEFT:
        ArrowKey(LeftKey);
        break;
      case BUTTON_RIGHT:
        ArrowKey(RightKey);
        break;
      case BUTTON_HASH:
        if (lastValue == BUTTON_HASH) {
          sessions[current_session].throttle->stopAll();
        }
        sessions[current_session].speed = 0;
        sessions[current_session].throttle->setSpeedDirection(sessions[current_session].session_id, 0, true);
        updateDisplay();
        break;
      case 0xFFFFFFFF:
        // Repeat code
        switch (lastValue) {
          case BUTTON_UP:
            ArrowKey(UpKey);
            break;
          case BUTTON_DOWN:
            ArrowKey(DownKey);
            break;
        }
        break;
      default:
        Serial.print("1K: ");
        Serial.println(results->value, HEX);
        break;
    }
    if (results->value != 0xFFFFFFFF) {
      lastValue = results->value;
    }
  }

}

void allocateSession() {
  sessions[current_session].throttle_state = START;
  sessions[current_session].throttle->getSession(sessions[current_session].locoNo);
  sessions[current_session].throttle_state = WAITING;
  sessions[current_session].speed = 0;
}

void releaseSession() {
  sessions[current_session].throttle->releaseSession(sessions[current_session].session_id);
  sessions[current_session].locoNo = 0;
  sessions[current_session].throttle_state = START;
}

void functionOn(uint8_t number) {
  Serial.print("1F");
  Serial.print(number);
  Serial.println(" On");
  uint8_t group = 0;
  uint8_t mask = 0;
  int i;
  if (number < 5) {
    group = 1;
    if (sessions[current_session].functions & 0x1) {
      mask |= 0x10;
    }
    if (sessions[current_session].functions & 0x2) {
      mask |= 0x01;
    }
    if (sessions[current_session].functions & 0x4) {
      mask |= 0x02;
    }
    if (sessions[current_session].functions & 0x8) {
      mask |= 0x04;
    }
    if (sessions[current_session].functions & 0x10) {
      mask |= 0x08;
    }

  } else {
    group = (number / 5) + 1;
    int base = (group - 1) * 5;

    for (i = 0; i < 5; i++) {
      if (sessions[current_session].functions & (1 << (i + base))) {
        mask |= (1 << i);
      }
    }
  }
  sessions[current_session].throttle->setFGOn(sessions[current_session].session_id, group, mask);
}

void functionOff(uint8_t number) {
  Serial.print("1F");
  Serial.print(number);
  Serial.println(" Off");
  uint8_t group = 0;
  uint8_t mask = 0;
  int i;
  if (number < 5) {
    group = 1;
    if (sessions[current_session].functions & 0x1) {
      mask |= 0x10;
    }
    if (sessions[current_session].functions & 0x2) {
      mask |= 0x01;
    }
    if (sessions[current_session].functions & 0x4) {
      mask |= 0x02;
    }
    if (sessions[current_session].functions & 0x8) {
      mask |= 0x04;
    }
    if (sessions[current_session].functions & 0x10) {
      mask |= 0x08;
    }

  } else {
    group = (number / 5) + 1;
    int base = (group - 1) * 5;
    for (i = 0; i < 5; i++) {
      if (sessions[current_session].functions & (1 << (i + base))) {
        mask |= (1 << i);
      }
    }
  }
  sessions[current_session].throttle->setFGOff(sessions[current_session].session_id, group, mask);
}

void updateDisplay() {
  char buf[80];
  if (sessions[current_session].throttle_state != ACQUIRED) {
    sprintf(buf, "0S: %04d", sessions[current_session].locoNo);
  } else {
    sprintf(buf, "0L: %04d\n1%s: %3d", sessions[current_session].locoNo,
            (sessions[current_session].speed < 0 ? "B" : "F"),
            (sessions[current_session].speed < 0 ? -sessions[current_session].speed : sessions[current_session].speed)
           );
  }
  Serial.println(buf);
}


void myUserFunc(Message * msg, MergCBUS * mcbus) {
  //  Serial.print("MSG Code: ");
  //  Serial.println(msg->getOpc());
}

void myUserFuncDCC(Message * msg, MergCBUS * mcbus) {
  //  Serial.print("DCC Code: ");
  //  Serial.println(msg->getOpc());
  main_throttle(msg);
}

void main_throttle(Message * msg) {

  if (cbus.isSelfEnumMode()) {
    //Serial.println("Node in self enum");
    return;
  }

  if (sessions[current_session].throttle_state == WAITING) {
    if (msg != NULL && msg->getOpc() == OPC_PLOC) {
      if (msg->getSession() > 0) {
        //Serial.print("got session ");
        //Serial.println(msg->getSession());
        sessions[current_session].previous_state = WAITING;
        sessions[current_session].session_id = msg->getSession();
        sessions[current_session].functions = 0;
        sessions[current_session].throttle->setSession(sessions[current_session].session_id,
            sessions[current_session].locoNo);
        sessions[current_session].throttle->setSpeedMode(sessions[current_session].session_id);
        sessions[current_session].throttle_state = ACQUIRED;
      }
    }
  }
}

void setup() {
  pinMode(LED, OUTPUT);

  //Configuration data for the node
  cbus.getNodeId()->setNodeName("CAB", 3);        //node name
  cbus.getNodeId()->setModuleId(MODULE_ID);            //module number
  cbus.getNodeId()->setManufacturerId(MANUFACTURER_ID);//merg code
  cbus.getNodeId()->setMinCodeVersion(MIN_CODE);       //Version 1
  cbus.getNodeId()->setMaxCodeVersion(MAX_CODE);
  cbus.getNodeId()->setProducerNode(true);
  cbus.getNodeId()->setConsumerNode(true);
  cbus.setPushButton(PUSH_BUTTON);//set the push button ports

  //used to manually reset the node. while turning on keep the button pressed
  //this forces the node for slim mode with an empty memory for learned events and devices
  if (digitalRead(PUSH_BUTTON) == LOW) {
    Serial.println("Setup new memory");
    cbus.setUpNewMemory();
    cbus.saveNodeFlags();
  }

  cbus.setLeds(GREEN_LED, YELLOW_LED); //set the led ports

  cbus.setUserHandlerFunction(&myUserFunc);//function that implements the node logic
  cbus.setDCCHandlerFunction(&myUserFuncDCC);
  cbus.initCanBus(CANPORT, CAN_125KBPS, MCP_8MHz, 20, 30);  //initiate the transport layer
  cbus.setFlimMode();

  pinMode(INTPIN, INPUT);                       // Setting pin 2 for /INT input

  for (int i = 0; i < N_SESSIONS; i++) {
    sessions[i].state = SessionFree;
    sessions[i].forwards = true;
    sessions[i].speed = 0;
    sessions[i].locoNo = 0;
    sessions[i].functions = 0;
    sessions[i].throttle = new MergCBUSThrottle(&cbus);
    sessions[i].throttle_state = START;
    sessions[i].previous_state = START;
  }

  Serial.begin(19200);
  irrecv.enableIRIn(); // Start the receiver

  Serial.println("0CANCABIR");
  Serial.println("1V1.0");
}

void loop() {
  int id;
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  if (irrecv.decode(&results)) {
    irCommand(&results);
    irrecv.resume(); // Receive the next value
  }

  cbus.cbusRead();
  cbus.run();//do all logic
  sessions[current_session].throttle->run();
  main_throttle(NULL);
}
