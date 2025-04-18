/*
ESPeccy - Sinclair ZX Spectrum emulator for the Espressif ESP32 SoC

Copyright (c) 2024 Juan José Ponteprino [SplinterGU]
https://github.com/SplinterGU/ESPeccy

This file is part of ESPeccy.

Based on previous work by:
- Víctor Iborra [Eremus] and David Crespo [dcrespo3d] (ESPectrum)
  https://github.com/EremusOne/ZX-ESPectrum-IDF
- David Crespo [dcrespo3d] (ZX-ESPectrum-Wiimote)
  https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote
- Ramón Martinez and Jorge Fuertes (ZX-ESPectrum)
  https://github.com/rampa069/ZX-ESPectrum
- Pete Todd (paseVGA)
  https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef TZX_HEADERS_H
#define TZX_HEADERS_H

#pragma pack(push, 1)

/*
        0       "ZXTape!"
        7       0x1A
        8       Major version number
        9       Minor version number
        A       ID of first block
        B       Body of first block

 */
typedef struct
{
    char zx_tape[7];
    char _1a;
    char Va;
    char Vb;
}
st_head_tzx;

/*
ID : 10 -

00 2  Pause After this block in milliseconds (ms)                      [1000]
02 2  Length of following data
04 x  Data, as in .TAP File

- Length: [02,03]+04
*/

typedef struct
{
    char ID;
    unsigned short pause;
    unsigned short long_block_tap;
    unsigned char flag;             //ff para data
    char tipo        ; //Type (0,1,2 or 3) 3 para code
    char nombre [10 ]  ;
    unsigned short longitud ; //  (lsb fisrt)
    unsigned short dir_ini        ; //  (lsb fisrt)
    unsigned short pam2        ;
    char chksm_hd  ;
}
st_block_10;


/*
ID : 11 - Turbo Loading Data Block
This block is very similar to the normal TAP block but with some additional info on the timings and other important differences. The same tape encoding is used as for the standard speed data block. If a block should use some non-standard sync or pilot tones (i.e. all sorts of protection schemes) then use the next three blocks to describe it.
00 2  Length of PILOT pulse                                            [2168]
02 2  Length of SYNC First pulse                                        [667]
04 2  Length of SYNC Second pulse                                       [735]
06 2  Length of ZERO bit pulse                                          [855]
08 2  Length of ONE bit pulse                                          [1710]
0A 2  Length of PILOT tone (in PILOT pulses)         [8064 Header, 3220 Data]
0C 1  Used bits in last byte (other bits should be 0)                     [8]
      i.e. if this is 6 then the bits (x) used in last byte are: xxxxxx00
0D 2  Pause After this block in milliseconds (ms)                      [1000]
0F 3  Length of following data
12 x  Data; format is as for TAP (MSb first)

- Length: [0F,10,11]+18
*/


typedef struct
{
    char ID;
    unsigned short l_pilot;
    unsigned short l_sync1;
    unsigned short l_sync2;
    unsigned short l_zero;
    unsigned short l_one;
    unsigned short n_pilot;
    char  u_bits;
    unsigned short pause;

    unsigned short  long_1;
    unsigned char   long_2;
    unsigned char   data1; // [1];

}
st_block_11 ;

/*
ID : 12  -  Pure tone
-------
        This will produce a tone which is  basically the same as the pilot tone
        in the first two data blocks. You can define how long the pulse is  and
        how many pulses are in the tone.

00 2  Length of pulse in T-States
02 2  Number of pulses

- Length: 04
*/

typedef struct
{
    char ID;
    unsigned short  ancho_pulso;
    unsigned short  n_pulsos;
}
st_block_12 ;

/*
ID : 13  -  Sequence of pulses of different lengths
-------
        This  will  produce  n  pulses,  each having its own timing.  Up to 255
        pulses  can be  stored in  this  block; this is useful for non-standard
        sync tones used by some protection schemes.

00 1  Number of pulses
01 2  Length of first pulse in T-States
03 2  Length of second pulse...
.. .  etc.

- Length: [00]*02+01
*/


typedef struct
{
    char ID;
    unsigned char n_pulses;
    unsigned short ancho_pulso [1];

}
st_block_13 ;

/*
ID : 14  -  Pure data block
-------
        This is the same as in the turbo loading data block, except that it has
        no pilot or sync pulses.

00 2  Length of ZERO bit pulse
02 2  Length of ONE bit pulse
04 1  Used bits in LAST Byte
05 2  Pause after this block in milliseconds (ms)
07 3  Length of following data
0A x  Data (MSb fitst)

- Length: [07,08,09]+0A
*/



typedef struct
{
    char ID;
    unsigned short  zero_length;
    unsigned short  one_length;
    unsigned char used_bits;
    unsigned short pause ;

    unsigned short  long_1;
    unsigned char   long_2;
    unsigned char   data1;  // [1];

}
st_block_14 ;

/*

ID : 15  -  Direct recording
-------
        This block is used for tapes  which have some  parts in  a format  such
        that the  turbo loader  block cannot  be used. This is *not* like a VOC
        file, since the information is much more compact.  Each sample value is
        represented  by one  bit only  (0 for low, 1 for high) which means that
        the block will be at most 1/8 the size of the equivalent VOC.

        *Please* use this  block  *only*  if  you cannot  use the turbo loading
        block.  The preffered sampling frequencies are  22050 (158 T states) or
        44100  Hz  (79 T states/sample).  Please, if  you can, don't  use other
        sampling frequencies.

00 2  Number of T states per sample (bit of data)
02 2  Pause after this block in milliseconds (ms)
04 1  Used bits (samples) in last byte of data (1-8)
      i.e. If this is 2 only first two samples of the last byte will be played
05 3  Data length
08 x  Data. Each bit represents a state on the EAR port (i.e. one sample);
      MSb is played first.

- Length: [05,06,07]+08
*/


typedef struct
{
    char            ID;
    unsigned short  tstates_sample;
    unsigned short  pause;
    unsigned char   used_bits;
    unsigned short  long_1;
    unsigned char   long_2;
    unsigned char   data [1];

}
st_block_15 ;

#pragma pack(pop)


int tzxread(const char *filename, const char *output_screen);

//---------------------------------------------------------------------------
#endif
