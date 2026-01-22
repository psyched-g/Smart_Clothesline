#define BLYNK_TEMPLATE_ID "TMPL3BbZoqoWd"
#define BLYNK_TEMPLATE_NAME "Automation"
#define BLYNK_AUTH_TOKEN "TstRgeal7ymwxr2YW6JdiYISFSRNsa7A"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi credentials
char ssid[] = "GAUTHAM";
char pass[] = "012345678";

// Hardware Serial for STM32
HardwareSerial STM32Serial(2);  // UART2 (RX=16, TX=17)

// Mode variable: 0 = Auto, 1 = Manual
int mode = 0;
int direction = 0;   // 0 = FORWARD, 1 = REVERSE

//------------------------------------------------------
// MODE SWITCH (V0): AUTO(0) / MANUAL(1)
//------------------------------------------------------
BLYNK_WRITE(V0)
{
  mode = param.asInt();

  if (mode == 1) {
    Serial.println("Mode: MANUAL");
    STM32Serial.println("MODE_MANUAL");

    // Enable manual controls
    Blynk.virtualWrite(V1, 0);
    Blynk.setProperty(V1, "isDisabled", false);
    Blynk.setProperty(V3, "isDisabled", false);

  } else {
    Serial.println("Mode: AUTOMATIC");
    STM32Serial.println("MODE_AUTO");

    // Disable manual controls
    Blynk.virtualWrite(V1, 0);
    Blynk.setProperty(V1, "isDisabled", true);
    Blynk.setProperty(V3, "isDisabled", true);
  }
}

//------------------------------------------------------
// DIRECTION SWITCH (V3): FORWARD(0) / REVERSE(1)
//------------------------------------------------------
BLYNK_WRITE(V2)
{
  direction = param.asInt();
   Serial.print("Direction changed to: ");
  Serial.println(direction);

  if (mode == 1) {
    if (direction == 0)
      Serial.println("Direction: FORWARD");
    else
      Serial.println("Direction: REVERSE");
  }
}

//------------------------------------------------------
// MANUAL CONTROL BUTTON (V1): ON / OFF
//------------------------------------------------------
BLYNK_WRITE(V1)
{
  int value = param.asInt();


  if (mode == 1)  // Only allowed in MANUAL mode
  {
    if (direction == 0)  // FORWARD mode
    {
      if (value == 1)
      {
        Serial.println("FORWARD ON");
        STM32Serial.println("FORWARD_ON");
      }
      else
      {
        Serial.println("FORWARD STOP");
        STM32Serial.println("FORWARD_STOP");
      }
    }
    else  // REVERSE mode
    {
      if (value == 1)
      {
        Serial.println("REVERSE ON");
        STM32Serial.println("REVERSE_ON");
      }
      else
      {
        Serial.println("REVERSE STOP");
        STM32Serial.println("REVERSE_STOP");
      }
    }
  }

  else
  {
    Serial.println("Ignored — in AUTO mode");
    Blynk.virtualWrite(V1, 0); // Reset visually
  }
}



void setup() {
  Serial.begin(115200);
  STM32Serial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initially in Auto mode → manual button disabled
  Blynk.setProperty(V1, "isDisabled", true);
  Blynk.setProperty(V3, "isDisabled", true);

  Serial.println("ESP32 connected to Wi-Fi & Blynk Cloud");
  Serial.println("System started in AUTO mode");
}

void loop() {
  Blynk.run();
}
