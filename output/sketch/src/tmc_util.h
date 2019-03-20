/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "MarlinConfig.h"
#include "ultralcd.h"
#if HAS_TRINAMIC
  #include <TMCStepper.h>
#endif

#define TMC_X_LABEL 'X', '0'
#define TMC_Y_LABEL 'Y', '0'
#define TMC_Z_LABEL 'Z', '0'

#define TMC_X2_LABEL 'X', '2'
#define TMC_Y2_LABEL 'Y', '2'
#define TMC_Z2_LABEL 'Z', '2'
#define TMC_Z3_LABEL 'Z', '3'

#define TMC_E0_LABEL 'E', '0'
#define TMC_E1_LABEL 'E', '1'
#define TMC_E2_LABEL 'E', '2'
#define TMC_E3_LABEL 'E', '3'
#define TMC_E4_LABEL 'E', '4'
#define TMC_E5_LABEL 'E', '5'

#define CHOPPER_DEFAULT_12V  { 3, -1, 1 }
#define CHOPPER_DEFAULT_19V  { 4,  1, 1 }
#define CHOPPER_DEFAULT_24V  { 4,  2, 1 }
#define CHOPPER_DEFAULT_36V  { 5,  2, 4 }
#define CHOPPER_PRUSAMK3_24V { 4,  1, 4 }
#define CHOPPER_MARLIN_119   { 5,  2, 3 }
#define CHOPPER_DUET_WIFI    { 4,  0, 4 }

template<char AXIS_LETTER, char DRIVER_ID>
class TMCStorage {
  protected:
    // Only a child class has access to constructor => Don't create on its own! "Poor man's abstract class"
    TMCStorage() {}

    uint16_t val_mA = 0;
    // geo-f: support TMC2660 senseless homing
    #if HAS_DRIVER(TMC2660)
      uint8_t val_stall_sentility_homing = 0;
      uint8_t val_stall_sentility = 0;
      uint16_t val_stall_current = 0;
      uint16_t val_stall_current_homing = 0;
    #endif

  public:
    #if ENABLED(MONITOR_DRIVER_STATUS)
      uint8_t otpw_count = 0,
              error_count = 0;
      bool flag_otpw = false;
      bool getOTPW() { return flag_otpw; }
      void clear_otpw() { flag_otpw = 0; }
    #endif

    uint16_t getMilliamps() { return val_mA; }
    uint8_t stall_sentility() {
      return this->val_stall_sentility;
    }   
    uint8_t stall_sentility_homing() {
      return this->val_stall_sentility_homing;
    } 
    uint16_t stall_current() {
      return this->val_stall_current;
    }
    uint16_t stall_current_homing() {
      return this->val_stall_current_homing;
    }

    void printLabel() {
      SERIAL_CHAR(AXIS_LETTER);
      if (DRIVER_ID > '0') SERIAL_CHAR(DRIVER_ID);
    }
};

template<class TMC, char AXIS_LETTER, char DRIVER_ID>
class TMCMarlin : public TMC, public TMCStorage<AXIS_LETTER, DRIVER_ID> {
  public:
    TMCMarlin(uint16_t cs_pin, float RS) :
      TMC(cs_pin, RS)
      {}
    TMCMarlin(uint16_t CS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
      TMC(CS, RS, pinMOSI, pinMISO, pinSCK)
      {}
    uint16_t rms_current() { return TMC::rms_current(); }
    void rms_current(uint16_t mA) {
      this->val_mA = mA;
      TMC::rms_current(mA);
    }
    void rms_current(uint16_t mA, float mult) {
      this->val_mA = mA;
      TMC::rms_current(mA, mult);
    }
    // geo-f:add 20190111
    #if HAS_DRIVER(TMC2660)
      void stall_sentility_homing_var(uint8_t val) {
        this->val_stall_sentility_homing = val;
      }       
      void stall_sentility_var(uint8_t val) {
        this->val_stall_sentility = val;
      }     
      void stall_current_var(uint16_t val) {
        this->val_stall_current = val;
      }
      void stall_current_homing_var(uint16_t val) {
        this->val_stall_current_homing = val;
      }
    #endif
};
template<char AXIS_LETTER, char DRIVER_ID>
class TMCMarlin<TMC2208Stepper, AXIS_LETTER, DRIVER_ID> : public TMC2208Stepper, public TMCStorage<AXIS_LETTER, DRIVER_ID> {
  public:
    TMCMarlin(Stream * SerialPort, float RS, bool has_rx=true) :
      TMC2208Stepper(SerialPort, RS, has_rx=true)
      {}
    TMCMarlin(uint16_t RX, uint16_t TX, float RS, bool has_rx=true) :
      TMC2208Stepper(RX, TX, RS, has_rx=true)
      {}
    uint16_t rms_current() { return TMC2208Stepper::rms_current(); }
    void rms_current(uint16_t mA) {
      this->val_mA = mA;
      TMC2208Stepper::rms_current(mA);
    }
    void rms_current(uint16_t mA, float mult) {
      this->val_mA = mA;
      TMC2208Stepper::rms_current(mA, mult);
    }
};

constexpr uint16_t _tmc_thrs(const uint16_t msteps, const int32_t thrs, const uint32_t spmm) {
  return 12650000UL * msteps / (256 * thrs * spmm);
}

template<typename TMC>
void tmc_get_current(TMC &st) {
  st.printLabel();
  SERIAL_ECHOLNPAIR(" driver current: ", st.getMilliamps());
}
template<typename TMC>
void tmc_set_current(TMC &st, const int mA) {
  st.rms_current(mA);
}
#if ENABLED(MONITOR_DRIVER_STATUS)
  template<typename TMC>
  void tmc_report_otpw(TMC &st) {
    st.printLabel();
    SERIAL_ECHOPGM(" temperature prewarn triggered: ");
    serialprintPGM(st.getOTPW() ? PSTR("true") : PSTR("false"));
    SERIAL_EOL();
  }
  template<typename TMC>
  void tmc_clear_otpw(TMC &st) {
    st.clear_otpw();
    st.printLabel();
    SERIAL_ECHOLNPGM(" prewarn flag cleared");
  }
#endif
template<typename TMC>
void tmc_get_pwmthrs(TMC &st, const uint16_t spmm) {
  st.printLabel();
  SERIAL_ECHOLNPAIR(" stealthChop max speed: ", _tmc_thrs(st.microsteps(), st.TPWMTHRS(), spmm));
}
template<typename TMC>
void tmc_set_pwmthrs(TMC &st, const int32_t thrs, const uint32_t spmm) {
  st.TPWMTHRS(_tmc_thrs(st.microsteps(), thrs, spmm));
}
template<typename TMC>
void tmc_get_sgt(TMC &st) {
  st.printLabel();
  SERIAL_ECHOPGM(" homing sensitivity: ");
  SERIAL_PRINTLN(st.sgt(), DEC);
}
template<typename TMC>
void tmc_set_sgt(TMC &st, const int8_t sgt_val) {
  st.sgt(sgt_val);
}

// gf:add 20190121
template<typename TMC>
void tmc_enable_stealthChop(TMC &st) {
  st.en_pwm_mode(true);
}


void monitor_tmc_driver();
void test_tmc_connection(const bool test_x, const bool test_y, const bool test_z, const bool test_e);

#if ENABLED(TMC_DEBUG)
  #if ENABLED(MONITOR_DRIVER_STATUS)
    void tmc_set_report_status(const bool status);
  #endif
  void tmc_report_all(const bool print_x, const bool print_y, const bool print_z, const bool print_e);
  void tmc_get_registers(const bool print_x, const bool print_y, const bool print_z, const bool print_e);
#endif

/**
 * TMC2130 specific sensorless homing using stallGuard2.
 * stallGuard2 only works when in spreadCycle mode.
 * spreadCycle and stealthChop are mutually exclusive.
 *
 * Defined here because of limitations with templates and headers.
 */
#if USE_SENSORLESS
  // Track enabled status of stealthChop and only re-enable where applicable
  struct sensorless_t {
    bool x, y, z;
  };

   #if HAS_DRIVER(TMC2130) // geo-f
    bool tmc_enable_stallguard(TMC2130Stepper &st);
    void tmc_disable_stallguard(TMC2130Stepper &st, const bool restore_stealth);
  #endif
/*
  #if HAS_DRIVER(TMC2660)
    template <char AXIS_LETTER, char DRIVER_ID>
      bool tmc_enable_stallguard(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID> &st);
    template <char AXIS_LETTER, char DRIVER_ID>
      void tmc_disable_stallguard(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID> &st,const bool);
  #endif
  */

  #if HAS_DRIVER(TMC2660)
    template <char AXIS_LETTER, char DRIVER_ID>
      bool tmc_enable_stallguard(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID> &st) {
        st.toff(st.savedToff());
        uint16_t a = st.stall_current_homing();
        //SERIAL_ECHOLNPAIR("a1:",a);
        st.rms_current(a);
        uint8_t s = st.stall_sentility_homing();
        //SERIAL_ECHOLNPAIR("s1:",s);
        st.sgt(s);
        st.sfilt(false);
        delay(10);
        return false;
      }
      
    template <char AXIS_LETTER, char DRIVER_ID>
      void tmc_disable_stallguard(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID> &st,const bool) {
        uint16_t a = st.stall_current();
        //SERIAL_ECHOLNPAIR("a2:",a);
        st.rms_current(a);
        uint8_t s = st.stall_sentility();
        //SERIAL_ECHOLNPAIR("s2:",s);
        st.sgt(s);
        st.sfilt(true);
        delay(10);
        return;
      }
  #endif

#endif

#if TMC_HAS_SPI
  void tmc_init_cs_pins();
#endif
