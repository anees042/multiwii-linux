/*
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 *
 * config.c
 *
 */
#include "config.h"
#include "def.h"
#include "multiwii.h"
#include "util.h"

static uint8_t checkNewConf = 151;
config_t cfg;


void config_save(void)
{
	//TODO save config to file
//    eeprom_open();
//    eeprom_write_block(&cfg, (void *)NULL, sizeof(cfg));
//    eeprom_close();
//   blinkLED(15, 20, 1);
}

void config_init(void)
{
    DEBUG("config_init");
//    eeprom_open();
//    eeprom_read_block(&test_val, (void *)0, 1);
//    eeprom_close();

//TODO read config from file; cmdline

    cfg.version = checkNewConf;
    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 0;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 16;
    cfg.I8[PIDALT] = 15;
    cfg.D8[PIDALT] = 7;
    cfg.P8[PIDGPS] = 50;
    cfg.I8[PIDGPS] = 0;
    cfg.D8[PIDGPS] = 15;
    cfg.P8[PIDVEL] = 0;
    cfg.I8[PIDVEL] = 0;
    cfg.P8[PIDLEVEL] = 90;
    cfg.I8[PIDLEVEL] = 45;
    cfg.D8[PIDLEVEL] = 100;
    cfg.P8[PIDMAG] = 40;
    cfg.rcRate8 = 45;               // = 0.9 in GUI
    cfg.rcExpo8 = 65;
    cfg.rollPitchRate = 0;
    cfg.yawRate = 0;
    cfg.dynThrPID = 0;
    for (int i = 0; i < CHECKBOXITEMS; i++) {
        cfg.activate1[i] = 0;
        cfg.activate2[i] = 0;
    }
    cfg.accTrim[0] = 0;
    cfg.accTrim[1] = 0;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    cfg.gimbal_flags = 0;
    cfg.gimbal_gain_pitch = TILT_PITCH_PROP;
    cfg.gimbal_gain_roll = TILT_ROLL_PROP;
    cfg.vbatscale = 110;
    cfg.vbatmaxcellvoltage = 43;
    cfg.vbatmincellvoltage = 33;

    cfg.FAILSAFE = 1;
    cfg.GYRO = 1;
    cfg.ACC = 1;
    cfg.BARO = 1;
    cfg.MAG = 1;
    cfg.SONAR = 1;

    cfg.flap_channel = FLAP_CHANNEL;
    cfg.motor_disarmed_value= MINTHROTTLE;

    config_save();
}