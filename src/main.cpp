//////////////////////////////////////////////////////////////////////////////
/// @file main.cpp
/// @author Kai R. ()
/// @brief PWM Fan Control
///        This program is supposed to control a PWM fan.
///        For this purpose, the temperature is determined via an analog input and,
///        depending on this, the speed of the fan is regulated using a 25 kHz PWM signal.
///
/// @date 2023-12-31
/// @version 1.0
///
/// @copyright Copyright (c) 2023
///
//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SendOnlySoftwareSerial.h>   //  https://github.com/nickgammon/SendOnlySoftwareSerial

//////////////////////////////////////////////////////////////////////////////
/// @brief Helper class for simplified use of millis()
///
//////////////////////////////////////////////////////////////////////////////
class Timer {
public:
  void start() { timeStamp = millis(); }
  bool operator()(const uint32_t duration) const { return (millis() - timeStamp >= duration) ? true : false; }

private:
  uint32_t timeStamp {0};
};

//////////////////////////////////////////////////////////////////////////////
/// @brief  Class for calculating a simple average value
///
/// @tparam T
/// @tparam ITEMS
/// @return T
//////////////////////////////////////////////////////////////////////////////
template <typename T, size_t ITEMS> T getAverage(const T (&dataArray)[ITEMS]) {
  T tmp = 0;
  for (auto value : dataArray) { tmp += value; }
  return tmp / ITEMS;
}

//////////////////////////////////////////////////
// Definitions
//////////////////////////////////////////////////

//////////////////////////////////////////////////
// Global constants and variables
//////////////////////////////////////////////////

// These values only apply to the G10K3435 thermistor!!!!!
// NTC Temperature - ADC (10Bit)
//  25°C              512
// 100°C              956
constexpr uint16_t ADCVAL_MAX_THRESHOLD {956};   // ADC Value at 100°C
constexpr uint16_t ADCVAL_MIN_THRESHOLD {512};   // ADC Value at  25°C
// ADC at 25°C = 512 at 100°C = 956 => Diff 444
constexpr uint16_t ADCMAX_DIFF {ADCVAL_MAX_THRESHOLD - ADCVAL_MIN_THRESHOLD};
// Set PWM frequency to ( 8MHz / 2) / (159 + 1) = 25KHz -> Prescaler = 2
// Set PWM frequency to (16MHz / 4) / (159 + 1) = 25KHz -> Prescaler = 4
constexpr uint8_t MAX_PWM_FREQUENCY {160};
constexpr uint8_t MIN_PWM_FREQUENCY {32};
constexpr uint8_t PWMFREQUENCY_DIFF {MAX_PWM_FREQUENCY - MIN_PWM_FREQUENCY};

constexpr uint8_t TX {PB0};
constexpr uint16_t INTERVAL_MS {1000};
constexpr uint8_t MAX_AVERAGE_IDX {12};

static_assert(ADCVAL_MAX_THRESHOLD > ADCVAL_MIN_THRESHOLD,
              "ADCVAL_MIN_THRESHOLD must not be greater than ADCVAL_MAX_THRESHOLD");
static_assert(MAX_PWM_FREQUENCY > MIN_PWM_FREQUENCY, "MIX_PWMFREQUENZ must not be greater than MAX_PWM_FREQUENCY");

volatile uint32_t durationInt0;
volatile uint32_t lastDurInt0;

SendOnlySoftwareSerial mySerial(TX);
Timer timer;
uint32_t rpmSigValues[MAX_AVERAGE_IDX];

//////////////////////////////////////////////////////////////////////////////
/// @brief ISR for the frequency counter
///
//////////////////////////////////////////////////////////////////////////////
ISR(INT0_vect) {
  uint32_t timestamp = micros();
  durationInt0 = (timestamp - lastDurInt0);
  lastDurInt0 = timestamp;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Determine the frequency of a square wave signal on the INT0 pin (PB2).
///
//////////////////////////////////////////////////////////////////////////////
void initINT0() {
  // Set MCU Control Register = Interrupt Sense Control Bits
  // => any logical change on INT0 generates an interrupt request.
  // With a square wave signal, one interrupt per rising edge should be generated with this setting.
  MCUCR = (1 << ISC01) | (1 << ISC00);
  GIMSK = (1 << INT0) | (0 << PCIE);   // Set GIMSK – General Interrupt Mask Register = enable INT0
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Init the Analog to Digital Converter for reading data from Pin PB4
///
//////////////////////////////////////////////////////////////////////////////
void initAdc() {
  ADMUX |= _BV(MUX1);   // use ADC2 for input (PB4), MUX bit 1
  ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
  while (ADCSRA & (1 << ADSC)) { ; }   // Wait for the conversion process to complete.
  static_cast<void>(ADCW);             // Discard the first reading.
}

//////////////////////////////////////////////////////////////////////////////
/// @brief ADC multiple measurement with averaging.
///
/// @param nsamples   Number of desired read operations
/// @return uint16_t  Average of the read values
//////////////////////////////////////////////////////////////////////////////
uint16_t adcReadAvg(uint8_t nsamples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < nsamples; ++i) {
    ADCSRA |= (1 << ADSC);               // One "single conversion"
    while (ADCSRA & (1 << ADSC)) { ; }   // Wait for the conversion to complete
    sum += ADCW;
  }
  return static_cast<uint16_t>(sum / nsamples);
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Init Timer Control Register 1 for synchronized PWM Mode
///        Output is PB1 OC1A (non inverted). Inverted Output is
///        not connected
///
/// @param dutyCycle
/// @param pwmFreq
//////////////////////////////////////////////////////////////////////////////
void initPwmSignal(uint8_t pwmFreq, uint8_t dutyCycle) {
  TCCR1 = _BV(PWM1A) | _BV(COM1A1) | _BV(CS11);
  OCR1A = dutyCycle;   // Set initial duty cycle
  OCR1C = pwmFreq;     // Set Top-Value for 25kHz
}

//////////////////////////////////////////////////////////////////////////////////////
// Main program
//
// Used Pins:
// Pin 1 PB5 = Reset
// Pin 2 PB3 = unsed
// Pin 3 PB4 = ADC Input (Temperature Voltage)
// Pin 4 GND
// PIN 5 PB0 = mySerial Output
// Pin 6 PB1 = PWM Signal Output
// Pin 7 PB2 = PWM FAN Signal Input (Frequency Counter)
// Pin 8 VCC = +5V
//			         +-\/-+
//  NC    *PB5 1 |    | 8 VCC   Red
//  NC     PB3 2 |    | 7 PB2*  Green
//  White *PB4 3 |    | 6 PB1*  Blue
//  Black  GND 4 |    | 5 PB0*  Orange
//               +----+
//////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set PIN PB1 as Output for the PWM Signal
  // Set PIN PB0 as Output for mySerial
  DDRB |= _BV(PB1) | _BV(PB0);

  initINT0();
  initAdc();
  initPwmSignal(MAX_PWM_FREQUENCY, MIN_PWM_FREQUENCY);
  sei();
  mySerial.begin(9600);
}

void loop() {
  static uint16_t oldValue {0};
  static uint8_t avgIdx {0};
  static uint8_t show {0};

  uint16_t adcRaw = adcReadAvg(4);
  if (adcRaw != oldValue) {
    oldValue = adcRaw;
    uint16_t adcCalc;
    if (adcRaw < ADCVAL_MIN_THRESHOLD) {   // Even if the temperature is lower,
      adcCalc = ADCVAL_MIN_THRESHOLD;      // the fan speed should correspond to that
                                           // at 25 °C
    } else if (adcRaw > ADCVAL_MAX_THRESHOLD) {
      adcCalc = ADCVAL_MAX_THRESHOLD;
    } else {
      adcCalc = adcRaw;
    }
    OCR1A = static_cast<uint8_t>((PWMFREQUENCY_DIFF * ((adcCalc - ADCVAL_MIN_THRESHOLD) * 100U / ADCMAX_DIFF) / 100U) +
                                 MIN_PWM_FREQUENCY);
  }

  if (timer(INTERVAL_MS) == true) {
    uint32_t frequency {0};
    if (durationInt0 > 0) { frequency = 100000000 / durationInt0; }
    rpmSigValues[avgIdx] = frequency;
    frequency = getAverage(rpmSigValues);
    if (++avgIdx >= MAX_AVERAGE_IDX) { avgIdx = 0; }

    if (show < (MAX_AVERAGE_IDX + 1)) {
      ++show;
    } else {
      mySerial.print("Frequenz1: ");
      mySerial.print(frequency / 100);
      mySerial.print('.');
      mySerial.print(frequency % 100);
      mySerial.println("Hz");

      frequency *= 60;
      frequency /= 200;
      mySerial.print("RPM1: ");
      mySerial.println(frequency);
    }
    timer.start();
  }
}
