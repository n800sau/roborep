#ifndef __MAG3110_V10_H

#define __MAG3110_V10_H

/*
  MAG3110 Breakout Code Header File
*/

#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change

void mag_config();
int mag_readx();
int mag_ready();
int mag_readz();

void mag_print_values();

#endif __MAG3110_V10_H
