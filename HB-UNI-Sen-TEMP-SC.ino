#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>
#include <ContactState.h>
#include <Button.h>

#include <sensors/Ntc.h>

// led for device activity
#define LED_PIN           9
// button to switch on/off the device
#define CONFIG_BTN_PIN    3
// pin to power ntc (or 0 if connected to vcc)
#define NTC_ACTIVATOR_PIN 5
// pin to measure ntc
#define NTC_SENSE_PIN    A0

#define RELAIS_PIN        6

// temperature where ntc has resistor value of R0
#define NTC_T0 25
// resistance both of ntc and known resistor
#define NTC_R0 10000
// b value of ntc (see datasheet)
#define NTC_B 3950
// number of additional bits by oversampling (should be between 0 and 6, highly increases number of measurements)
#define NTC_OVERSAMPLING 4

// measure all xx seconds
#define MEASURE_INTERVAL 5

#define PEERS_PER_CHANNEL 6

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x06, 0x01},          // Device ID
  "JPTEMPSC01",                // Device Serial
  {0xf3, 0x06},                // Device Model
  0x10,                        // Firmware Version
  as::DeviceType::THSensor,    // Device Type
  {0x01, 0x01}                 // Info Bytes
};

typedef AskSin<StatusLed<LED_PIN>, IrqInternalBatt, Radio<AvrSPI<10, 11, 12, 13>, 2>> Hal;
Hal hal;

DEFREGISTER(UReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x19, 0x20, 0x21)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    uint8_t powerOffBehaviour() const { return this->readRegister(0x19, 0); }
    bool powerOffBehaviour(uint8_t value) const { return this->writeRegister(0x19, value & 0xff); }

    void defaults () {
      clear();
      Sendeintervall(60);
      lowBatLimit(22);
      powerOffBehaviour(0);
    }
};

DEFREGISTER(Reg1_0, CREG_AES_ACTIVE)
class PWRSCList1 : public RegList1<Reg1_0> {
  public:
    PWRSCList1 (uint16_t addr) : RegList1<Reg1_0>(addr) {}
    void defaults () {
      clear();
    }
};

DEFREGISTER(Reg1_1, CREG_AES_ACTIVE, CREG_COND_TX_THRESHOLD_HI, CREG_COND_TX_THRESHOLD_LO)
class TEMPSCList1 : public RegList1<Reg1_1> {
  public:
    TEMPSCList1 (uint16_t addr) : RegList1<Reg1_1>(addr) {}
    void defaults () {
      clear();
      condTxThresholdHi(90);
      condTxThresholdLo(80);
    }
};


class PWRStateChannel : public Channel<Hal, PWRSCList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0> {
  private:
    uint8_t count, state;
  public:

    PWRStateChannel () :  count(0), state(0) {}
    virtual ~PWRStateChannel () {}

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
    }

    uint8_t status () const {
      return state;
    }

    uint8_t flags () const {
      return this->device().battery().low() ? 0x80 : 0x00;
    }

    void send () {
      SensorEventMsg& msg = (SensorEventMsg&)device().message();
      msg.init(device().nextcount(), number(), count++, state, device().battery().low());
      device().sendPeerEvent(msg, *this);
    }

    void setPwrState(bool s) {
      state = ( s == true ) ? 200 : 0;
      send();
     /* if (state == 0) {
        _delay_ms(1000);
        pinMode(PWR_HOLD_PIN, INPUT);
        //for debugging
        while(1){}
      }*/
    }
};

class SCChannel : public Channel<Hal, TEMPSCList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0> {
  private:
    uint8_t count, state;
  public:

    SCChannel () : Channel(), count(0), state(0) {}
    virtual ~SCChannel () {}

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
    }

    uint8_t status () const {
      return state;
    }

    uint8_t flags () const {
      return this->device().battery().low() ? 0x80 : 0x00;
    }

    void send () {
      SensorEventMsg& msg = (SensorEventMsg&)device().message();
      msg.init(device().nextcount(), number(), count++, state, device().battery().low());
      device().sendPeerEvent(msg, *this);
    }

    void setState(bool s) {
      state = ( s == true ) ? 200 : 0;
      send();
    }

    void configChanged() {
      Channel::configChanged();
      DPRINTLN("SCChannel configChanged");
      DPRINT("condTxThresholdHi  ");DDECLN(this->getList1().condTxThresholdHi());
      DPRINT("condTxThresholdLo  ");DDECLN(this->getList1().condTxThresholdLo());
    }
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
    class WeatherEventMsg : public Message {
      public:
        void init(uint8_t msgcnt, int temp, bool batlow, uint8_t flg) {
          uint8_t t1 = (temp >> 8) & 0x7f;
          uint8_t t2 = temp & 0xff;
          if ( batlow == true ) {
            t1 |= 0x80; // set bat low bit
          }
          Message::init(0xc, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
          pload[0] = flg;
        }
    };
  protected:
    WeatherEventMsg msg;
  private:
    Ntc<NTC_SENSE_PIN, NTC_R0, NTC_B, NTC_ACTIVATOR_PIN, NTC_T0, NTC_OVERSAMPLING> ntc;
    uint8_t measurecnt;
    bool sensOK, firstrun;
  public:
    WeatherChannel () : Channel(), Alarm(5), measurecnt(0), sensOK(false), firstrun(true) {}
    virtual ~WeatherChannel () {}

    void measure () {
      device().battery().setIdle();
      ntc.measure();
      device().battery().unsetIdle();
      sensOK = (ntc.temperature() > -500 && ntc.temperature() < 1200);
      DPRINT("T = "); DDECLN(ntc.temperature());
      DPRINT("B = "); DDECLN(device().battery().voltageHighRes());
    }

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      uint8_t msgcnt = device().nextcount();

      tick = seconds2ticks(MEASURE_INTERVAL);

      clock.add(*this);
      measure();
      measurecnt++;

      uint16_t sendInterval = max(10, device().getList0().Sendeintervall());

      if (firstrun || measurecnt * MEASURE_INTERVAL >= sendInterval) {
        msg.init(msgcnt, ntc.temperature(), device().battery().low(), flags());
        device().broadcastEvent(msg);
        measurecnt = 0;
        firstrun = false;
      }
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      ntc.init();
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return sensOK ? 0x00 : 0x01 << 1;
    }

    int16_t temperature() {
      return ntc.temperature();
    }
};

class TempScDeviceType : public ChannelDevice<Hal, VirtBaseChannel<Hal, UList0>, 3, UList0> {
    class CheckAlarm : public Alarm {
      private:
        bool isAboveThreshold;
      public:
        TempScDeviceType& wt;
        CheckAlarm(TempScDeviceType& w)  : Alarm(seconds2ticks(MEASURE_INTERVAL)), isAboveThreshold(false), wt(w) {}
        virtual ~CheckAlarm() {}

        virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
          //DPRINTLN("Checking for condition...");

          uint32_t CondHi = wt.scChannel().getList1().condTxThresholdHi() * 10;
          uint32_t CondLo = wt.scChannel().getList1().condTxThresholdLo() * 10;

          int16_t currentTemperature = wt.weatherChannel().temperature();

          DPRINT("condTxThresholdHi  ");DDECLN(CondHi);
          DPRINT("condTxThresholdLo  ");DDECLN(CondLo);
          DPRINT("currentTemperature ");DDECLN(currentTemperature);


          if ( currentTemperature > (int32_t)CondHi && isAboveThreshold == false ) {
            wt.led().invert(true);
            wt.scChannel().setState(true);
            isAboveThreshold = true;
            digitalWrite(RELAIS_PIN, HIGH);
          }

          if ( currentTemperature < (int32_t)CondLo && isAboveThreshold == true ) {
            wt.led().invert(false);
            wt.scChannel().setState(false);
            isAboveThreshold = false;
            digitalWrite(RELAIS_PIN, LOW);
          }

          set(seconds2ticks(MEASURE_INTERVAL));
          clock.add(*this);
        }

        void init() {
          sysclock.add(*this);
        }
    };

  public:
    VirtChannel<Hal, PWRStateChannel, UList0>     channel1;
    VirtChannel<Hal, SCChannel, UList0>           channel2;
    VirtChannel<Hal, WeatherChannel, UList0>      channel3;
    CheckAlarm ca;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, UList0>, 3, UList0> DeviceType;

    TempScDeviceType (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr), ca(*this) {
      DeviceType::registerChannel(channel1, 1);
      DeviceType::registerChannel(channel2, 2);
      DeviceType::registerChannel(channel3, 3);
    }
    virtual ~TempScDeviceType () {}

    PWRStateChannel& pwrChannel ()  {
      return channel1;
    }

    SCChannel& scChannel ()  {
      return channel2;
    }

    WeatherChannel& weatherChannel ()  {
      return channel3;
    }

    virtual void configChanged () {
      DeviceType::configChanged();
      DPRINT(F("*Sendeintervall   : ")); DDECLN(this->getList0().Sendeintervall());
      DPRINT(F("*LowBat           : ")); DDECLN(this->getList0().lowBatLimit());
      DPRINT(F("*PowerOffBehaviour: ")); DDECLN(this->getList0().powerOffBehaviour());
      battery().low(this->getList0().lowBatLimit());
    }

    void init(Hal& hal) {
      DeviceType::init(hal);
      ca.init();
    }
};

TempScDeviceType sdev(devinfo, 0x20);
ConfigButton<TempScDeviceType> cfgBtn(sdev);

void setup () {
  pinMode(RELAIS_PIN, OUTPUT);

  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  hal.initBattery(60UL * 60, 22, 19);
  //buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.scChannel().changed(true);
  sdev.pwrChannel().setPwrState(true);

  buttonISR(cfgBtn, CONFIG_BTN_PIN);

  while (sdev.battery().current() == 0);

  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    hal.activity.savePower<Sleep<>>(hal);
  }
}
