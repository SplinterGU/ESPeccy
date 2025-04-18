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


// 48K ROMS
#include "roms/romSinclair48K.h"
#include "roms/rom48Kspanish.h"
#include "roms/rom48Kcustom.h"

// 128K ROMS
#include "roms/romSinclair128K.h"
#include "roms/rom128Kspanish.h"
#include "roms/romPlus2.h"
#include "roms/romPlus2spanish.h"
#include "roms/romPlus2french.h"
#include "roms/rom128Kcustom.h"

// +2A/+3 ROMS
#include "roms/rom+2A+3_4.0.h"
#include "roms/rom+2A+3_4.0es.h"
#include "roms/rom+2A+3_4.1.h"
#include "roms/rom+2A+3_4.1es.h"
#include "roms/rom+2A+3custom.h"

// Pentagon 128K ROMS
#include "roms/rompentagon128k.h"

// TK ROMS
#include "roms/romTK90X_v1.h"
#include "roms/romTK90X_v2.h"
#include "roms/romTK90X_v3ES.h"
#include "roms/romTK90X_v3PT.h"
#include "roms/romTK90X_v3EN.h"
#include "roms/romTK95ES.h"
#include "roms/romTKcustom.h"

// TR-DOS ROM
#include "roms/trdos.h"

// ZX81+ Paul Farrow's IF2 ROM
#include "roms/S128_ZX81+_ROM.h"

// END