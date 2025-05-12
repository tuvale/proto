// Inkluderer biblioteker
#include <Arduino.h> // Hovedbiblioteket for Arduino
#include <Wire.h> // For I2C-kommunikasjon (modul til LCD-displayet)
#include <Adafruit_NeoPixel.h> // Kontrollere LED-stripe
#include <LiquidCrystal_I2C.h> // Bruke LCD-display med I2C-modul

// Definerer pinneverdier
const int analogPin       = A0;
const int motorPin1       = 5;
const int motorPin2       = 6;
const int enablePin       = 9;
const int resetButtonPin  = 7;
const int reedBane        = A1;
const int buzzerPin       = 8;
const int ledPin          = 10;

// Konfigurerer LED-stripen
#define LED_PIN    3 // Definerer pinneverdi
#define NUMPIXELS  47 // Det er 47 LED-piksler på stripen
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800); // Oppretter et objekt kalt pixels som representerer LED-stripen

// Oppretter et LCD-objekt for en tekstbasert LCD-skjerm med adresse 0x27, 16 tegn per rad og 2 rader via I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variabler:
int value = 0; // Midlertidig lagring av analog verdi fra reed relayen i pumpa
int sensor = -1; // sensor brukes til å indikere om stempelet har bevegd seg opp eller ned mellom reed relayene
int score = 1; // score er nåværende poengsum ila en runde. Den økes når stempelet i pumpa passerer en sensor i riktig rekkefølge (fra topp til bunn). Score brukes også til å starte bevegelse og vise tid i displayet.
int maxScore = 1; // maxScore er hvor mange poeng som må til før målpassering. Brukes i moveCart() for å justere hastigheten på motoren proporsjonalt med fremgangen.
int points = 1; // points er antall poeng som legges til per reed relay passert nedover i pumpa
int accountedScore = 1; // Hindrer at samme poeng blir registrert og utløser motorstart flere ganger
unsigned long currentTimerStart = 0; // Lagrer tidspunktet for når motoren kjører vogna ettersom poengsummen øker. Velger å bruke timer for å få presis kontroll på varigheten av handlingen i stedet for delay() for å ikke stoppe programflyten.
int pulseDuration = 70; // Motoren kjører i 70 ms per reed relay passert nedover i pumpa (per poeng registrert)
bool timerRunning = false; // Forteller om motoren er aktiv
bool resetting = false; // Forteller om vogna resetter seg i øyeblikket
unsigned long lastScoreUpdateTime = 0; // Lagrer tidspunktet for sist gang poengsummen økte (når vogna passerte en gyldig sensor og fikk poeng)

// Definerer variabler for seiersmelodi ved målgang
typedef int Melody_t;
Melody_t melody[] = { 392, 494, 587, 659, 587, 784 }; //Tall for toner
int noteDuration = 90; // Hver tone varer i 90 millisekunder

const int reedFinishMin   = 550; // Definerer intervall for verdien på reed relayen på slutten av banen
const int reedFinishMax   = 650;
const int reedResetThresh = 800; // Definerer minsteverdien for reed relayen på starten av banen. Hvis verdien er over dette, antar vi at vogna er ved startposisjon

bool started = false; // Sjekker om spillet er startet
unsigned long startTime = 0, stopTime = 0; // Lagrer tidspunktet for når vogna begynner å bevege seg og når den slutter å bevege seg. Brukes for å vise tiden man bruker på spillet på LCD-displayet.

unsigned long tid = 0; // Definerer tiden som vises på skjermen
// Deler opp tiden (millisekunder) som vises på skjermen i mer lesbare enhter:
byte sek = 0;
byte tidel = 0;
byte hundredel = 0;

// Kontrollerer fakkeleffekten på LED-stripen:
unsigned long flameStart = 0; // Tidspunktet for da siste flammeoppdatering startet
unsigned long flameDuration; // Hvor lenge den nåværende flammeeffekten skal vare. Verdien settes tilfeldig mellom 40 ms og 100 ms i setup()

void setup() {
  Serial.begin(9600);

// Setter hvilket modus hver pinne skal brukes som:
  pinMode(motorPin1, OUTPUT); // Styrer motoren fremover
  pinMode(motorPin2, OUTPUT); // Styrer motoren bakover
  pinMode(enablePin, OUTPUT); // Aktiverer motoren og kontrollerer hastigheten via analogWrite()
  pinMode(resetButtonPin, INPUT_PULLUP); // Lesing av resetknapp med innebygd pull-up-motstand
  pinMode(buzzerPin, OUTPUT); // Styrer buzzeren for seiersmelodien
  pinMode(ledPin, OUTPUT); // LED-strips

// Setter starttilstanden for LED-stripe og motorens retning:
  digitalWrite(ledPin, HIGH);
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

// Starter og oppdaterer LED-stripen:
  pixels.begin();
  pixels.show();

  flameDuration = random(40,100);

// Setter opp og initialiserer LCD-skjermen:
  lcd.init(); // Initialiserer LCD-skjermen (nødvendig for 12C-modulen)
  lcd.backlight(); // Slår på bakgrunnsbelysning
  lcd.setCursor(0, 0);
  lcd.print("Tid:");
}

void loop() {
  flameEffect(); // Kaller på funksjonen flameEffect(), som simulerer fakkeleffekten på LED-stripen 

  int reedVal = analogRead(reedBane); // Leser verdien fra reed relayene på banen og lagrer dne i reedVal

  //Serial.println(reedVal); // Skriver ut reedVal til serial monitor (ble brukt til debugging)

 // Leser av om resetknappen er aktivert, og om den er det og vogna ikke allerede resettes, så skal vogna resette seg:
  if (digitalRead(resetButtonPin) == LOW && !resetting) {
    if (started) {
      resetting = true;
      resetCart(); // Kaller på funksjon som tilbakestiller vogna
      resetting = false;
    } else {
      Serial.println("Vogna er allerede ved start – reset ignorert."); // Brukes til debugging
    }
  }

// Sjekker om vogna har passert målstreken (reed relayen på enden av banen). Da stoppes vogna (motoren stopper), displayet viser total tid brukt, seiersmelodien spilles og etter fem sekunder returnerer vogna tilbake til start av seg selv:
  if (!resetting && reedVal > reedFinishMin && reedVal < reedFinishMax) {
    Serial.println("Målpassering – spiller seiersmelodi"); // Brukes til debugging
    stopCart(); // Kaller på funksjon som stopper vogna ved målgang
    // Regner om og viser tiden brukt i sekunder, tideler og hundredeler til LCD-displayet:
      sek = tid / 1000;
  int rest = tid % 1000;
  tidel = rest / 100;
  hundredel = (rest % 100) / 10;
    sendToDisplay(sek, tidel, hundredel);
    spillSeiersmelodi();
    delay(5000);
    if (analogRead(reedBane) <= reedResetThresh) {
      resetting = true;
      resetCart();
      resetting = false;
    }
    started = false; // Når vogna har returnert til start 
    stopTime = startTime; // Setter sluttid lik starttid for å nullstille tidsregningen
  }

// Sjekker hvilken sensor som er aktiv og om den skal gi poeng eller ikke (funksjonen handleSensor() sikrer at poeng kun gis om stempelet i pumpa beveger seg nedover, ikke poeng om pumpa beveger seg oppover):
  int currentSensor = getActiveSensor();
  int newScore = handleSensorArrival(currentSensor);

// Sjekker om noen har pumpet nedover i pumpa og starter isåfall tidtaking:
  if (newScore > 0 && !started) {
    started = true;
    startTime = millis();
  }

// Oppdaterer siste tid for poengendringer:
  if (newScore > 0) {
    lastScoreUpdateTime = millis();
  }

  score += newScore; // Oppdaterer poengsummen

// Starter motoren dersom det er en åpengøkning og motoren ikke allerede går. Velger å bruke timer i stedet for delay for å ikke stoppe programflyten.
  if (!timerRunning && score > accountedScore) {
    startTimer();
  }

// Stopper motoren etter den har bevegd seg i 70 ms etter poengøkning:
  if (timerRunning && millis() > currentTimerStart + pulseDuration) {
    stopTimer();
  }

  // Automatisk reset  tilbake til start (reed relayen på starten av banen) etter 10 sekunder:
  if (started && millis() - lastScoreUpdateTime > 10000 && !resetting) {
    if (analogRead(reedBane) <= reedResetThresh) {
      Serial.println("Ingen poengøkning på 10 sekunder. Tilbakestiller..."); // Brukes for debugging
      resetting = true;
      resetCart();
      resetting = false;
      startTime = millis();
    } else {
      Serial.println("Vogna er allerede ved start – automatisk reset avbrutt."); // Brukes for debugging
    }
    started = false;
    stopTime  = startTime;
  }

  // Hvis spillet har startet, regnes tiden fra starttidspunktet til nåtid og vises på LCD-displayet:
  if (started) {
    tid = millis() - startTime;
  } else {
    tid = 0;
  }
  sek = tid / 1000;
  int rest = tid % 1000;
  tidel = rest / 100;
  hundredel = (rest % 100) / 10;

  sendToDisplay(sek, tidel, hundredel);
}


// Sjekker om stempelet på pumpa har bevegd seg oppover eller nedover. Gir kun økt poengsum dersom stempelet beveger seg nedover.
int handleSensorArrival(int newSensor) {
  if (newSensor == -1) return 0;
  else if (sensor == -1) {
    sensor = newSensor;
    return 0;
  } else if (newSensor > sensor) {
    sensor = newSensor;
    return 0;
  } else if (newSensor < sensor) {
    sensor = newSensor;
    return points;
  } else {
    return 0;
  }
}


// Leser av verdien og identifiserer hvilken reed relay som stempelet i pumpa passerer. Denne brukes videre i funksjonen handleSensorArrival() for å sjekke hvilken retning stempelet beveger seg.
int getActiveSensor() {
  value = analogRead(analogPin);

  if (value > 900 && value < 1023) { //Verdien på den nederste relayen er 932, brukt 1k ohm reistans
    return 0;
  } else if (value > 600 && value < 740) { //Verdien på den nest nederste relayen er 699, brukt 4.7k ohm resistans
    return 1;
  } else if (value > 740 && value < 900) { //Verdien på den nest øverste er 770, brukt 3.3k ohm resistans
    return 2;
  } else if (value > 400 && value < 600) { //Verdien på den øverste relayen er 512, brukt 10k ohm resistans
    return 3;
  } else {
    return -1;
  }
}

// Funksjon som definerer at motoren beveger seg forover:
void moveCart(int adding) {
  if (!resetting) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 255 * adding / maxScore);
  }
}

// Funksjon som stopper motoren:
void stopCart() {
  analogWrite(enablePin, 0);
}

// Funksjon for tilbakestilling av vogna:
void resetCart() {
  Serial.println("Tilbakestiller vogn..."); //Brukes til debugging
  started = false;
  startTime = 0;
  stopTime = 0;

  // Hvis vogna ikke allerede er i mål, skal den kort kjøre fremover før den sendes tilbake:
  if(!(reedBane > reedFinishMin && reedBane < reedFinishMax)){

    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 255);
    delay(100); // Motoren går kort fremover (100 ms) for å sikre at vogna ikke allerede er i startposisjon før retur.
    
  }

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  analogWrite(enablePin, 255);
  while (analogRead(reedBane) <= reedResetThresh) {
    delay(10);
  }

  analogWrite(enablePin, 0); // Motoren stopper ved start (når den har passert reed relayen ved starten av banen)

// Tilbakestiller verdier slik at de er klare for å sette i gang et nytt spill:
  sensor = -1;
  score = 1;
  accountedScore = 1;

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  Serial.println("Vogn tilbake til start."); // Brukes for debugging
}

// Aktiverer motoren kort (70 ms) etter poengøkning:
void startTimer() {
  moveCart(1);
  timerRunning = true;
  accountedScore++; // Hindrer at man dobbelttrigger for samme poeng.
  currentTimerStart = millis(); // Registrerer tidspunktet motoren ble startet
}

// Definerer at motoren stoppes når den ikke er aktivert lenger (etter 70 ms):
void stopTimer() {
  stopCart();
  timerRunning = false;
  stopTime = millis();
}

// Funksjon for å spille av seiersmelodi i buzzeren når vogna går i må:
void spillSeiersmelodi() {
  for (unsigned int i = 0; i < sizeof(melody)/sizeof(melody[0]); i++) { // Regner ut antall toner i melodien
    tone(buzzerPin, melody[i], noteDuration); // Spiller av tonen i buzzeren med den frekvensen/tonene vi ønsker i 90 ms
    delay(noteDuration);
    noTone(buzzerPin);
  }
}

// Funksjon for fakkeleffekten på LED-stripene:
void flameEffect() {
  if(millis() - flameStart < flameDuration) return; // Sjekker om det er for tidlig å oppdatere fakkeleffekten til en ny fakkeleffekt
  for (int i = 0; i < NUMPIXELS; i++) {
    int flicker = random(180, 240); // Lager en tilfeldig rød intensitet på mellom 180 og 240 for å lage fakkeleffekt
    // Definerer fargene. Ønsker mye rødt og litt oransj/gult for fakkeleffekt:
    int r = flicker;
    int g = random(flicker / 4, flicker / 2);
    int b = 0;
    pixels.setPixelColor(i, pixels.Color(r, g, b)); // Setter fargen på hvert LED, i
  }
  pixels.show();
  flameDuration = random(40,100);
}

// Viser tiden på LCD-displayet:
void sendToDisplay(byte sekunder, byte tideler, byte hundredeler) {
  lcd.setCursor(5, 0); // Plasserer markøren for å få tiden der vi ønsker på LCD-displayet
  lcd.print("        "); // Rydder bort gamle tall
  lcd.setCursor(5, 0); // Flytter markøren tilbake for ny tid
  // Skriver ut tiden på formen: x.xx s:
  lcd.print(sekunder);
  lcd.print(".");
  lcd.print(tideler);
  lcd.print(hundredeler);
  lcd.print("s");
}
