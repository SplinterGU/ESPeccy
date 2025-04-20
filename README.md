![ESPeccy](logos/ESPeccy.png)

**ESPeccy** is an emulator of the Sinclair ZX Spectrum computer running on Espressif ESP32 SoC powered boards.

Currently, it can be used with Lilygo's TTGo VGA32 board, Antonio Villena's ESPectrum board, and ESP32-SBC-FabGL board from Olimex.

To use it, simply connect a VGA monitor or CRT TV (a special VGA-RGB cable is required), a PS/2 keyboard, prepare an SD card as needed, and power it via microUSB.

This project is maintained and developed by Juan José Ponteprino [SplinterGU].

## Features Overview

### **Machine Emulation:**
- **ZX Spectrum 48K, 128K, and Pentagon 128K** 100% cycle accurate emulation (no PSRAM needed).
- **Microdigital TK90X and TK95 models** (w/Microdigital ULA at 50 and 60hz) emulation.
- **Spectrum +2A model** support.

### **Component and Effect Emulation:**
- **State of the art Z80 emulation** (Authored by [José Luis Sánchez](https://github.com/jsanchezv/z80cpp)).
- **Contended memory and contended I/O** emulation.
- **AY-3-8912 sound emulation**.
- **Beeper & Mic emulation** (Cobra’s Arc).
- **Dual PS/2 keyboard support**: Connect two devices using PS/2 protocol simultaneously.
- **ZXUnoPS2 (.ZXPURE) support** (selectable from BIOS).
- **PS/2 Joystick emulation** (Cursor, Sinclair, Kempston, and Fuller).
- **Two real joysticks support** (Up to 8 button joysticks) using [ESPjoy adapter](https://antoniovillena.es/store/product/espjoy-for-espectrum/) or DIY DB9 to PS/2 converter.
- **Kempston mouse support** with configurable sample rate, DPI, and scaling.
- **Covox sound support**.
- **Fullerbox sound in ZX-Spectrum**.
- **Zon-X sound emulation in ZX81**.
- **Snow effect accurate emulation** (as [described](https://spectrumcomputing.co.uk/forums/viewtopic.php?t=8240) by Weiv and MartianGirl).
- **Floating bus effect emulated** (Arkanoid, Sidewize).
- **Multicolor attribute effects emulated** (Bifrost*2, Nirvana, and Nirvana+ engines).
- **Border effects emulated** (Aquaplane, The Sentinel, Overscan demo).
- **Issue 2 emulation**.

### **ROM and File Support:**
- **Selectable Sinclair 48K, Sinclair 128K, and Amstrad +2 English and Spanish ROMs**.
- **Selectable TK90X v1, v2, and v3** (Rodolfo Guerra) ROMs.
- **Possibility of using one 48K, one 128K, and one TK90X custom ROM** with easy flashing procedure from SD card.
- **ZX81+ IF2 ROM** by courtesy of Paul Farrow with .P file loading from SD card.
- **Rodolfo Guerra's ROMs fast load routines support** with on-the-fly standard speed blocks translation.
- **Real tape support (SAVE/LOAD)**.
- **Realtime (with OSD) TZX and TAP file loading**.
- **Flashload of TAP files**.
- **SCR file support** and screen autodetection in tape/snapshot/ROM files.
- **SNA, Z80, and SP snapshot saving and loading**.
- **Save, load, rename, and delete game states with screen preview**.
- **Virtual tape system** with support for the SAVE command and block renaming, deleting, and moving.
- **BMP screen capture to SD Card** (thanks David Crespo 😉).

### **Graphics Output and Modes:**
- **6 bpp VGA output in three modes**: Standard VGA (60 and 70hz), VGA with every machine's real vertical frequency, and CRT 15khz with real vertical frequency.
- **VGA fake scanlines effect**.
- **Support for two aspect ratios**: 16:9 or 4:3 monitors (using 360x200 or 320x240 modes).
- **Complete overscan supported in CRT 15Khz mode** (at 352x272 resolution for 50hz machines and 352x224 for 60hz ones).

### **Menu and Configuration System:**
- **Complete OSD menu** in three languages: English, Spanish, and Portuguese.
- **Menu navigation with cursor keys**.
- **Configuration backup/restore feature to SD card**.
- **BIOS support with boot-time activation**:
    - PS2 keyboard: Repeatedly press and release the F2 key (or 2 key) for VGA mode, or the F3 key (or 3 key) for CRT mode.
    - ZX Keyboard: Press and hold the 2 key for VGA mode, or press and hold the 3 key for CRT mode.
    - ZXUnoPS2 (.ZXPURE): Repeatedly press and release the 2 key for VGA mode, or the 3 key for CRT mode.
    - LilyGO (button labeled GPIO36): Press and hold the button for less than 10 seconds for VGA mode, or press and hold it for more than 10 seconds for CRT mode.
- **Configurable IO36 Button functionality** on LilyGo (from BIOS) (RESET, NMI, CHEATS, POKE, STATS, MENU).
- **Complete file navigation system** with auto-indexing, folder support, and search functions.

## Work in progress

- +3 models.
- DSK support.

## Installing

You can flash the binaries directly to the board if you do not want to mess with code and compilers. Check the [releases section](https://github.com/SplinterGU/ESPeccy/releases).

## Compiling and installing

Quick start from PlatformIO:
- Clone this repo and Open from VSCode/PlatformIO
- Execute task: Upload
- Enjoy

Windows, GNU/Linux, and MacOS/X. This version has been developed using PlatformIO.

#### Install PlatformIO:

- There is an extension for Atom and VSCode, please check [this website](https://platformio.org/).
- Select your board, pico32, which behaves just like the TTGo VGA32.

#### Compile and flash it

`PlatformIO > Project Tasks > Build`, then

`PlatformIO > Project Tasks > Upload`.

Run these tasks (`Upload` also does a `Build`) whenever you make any change in the code.

#### Prepare micro SD Card

The SD card should be formatted in FAT16 / FAT32.

Just that: then put your .sna, .z80, .p, .tap, .trd, and .scl wherever you like and create and use folders as you need.

There's also no need to sort files using external utilities: the emulator creates and updates indexes to sort files in folders by itself.

## PS/2 Keyboard functions

- F1 Main menu
- F2 Load state
- F5 File browser
- F6 Play/Stop tape
- F8 CPU / Tape load stats ( [CPU] microsecs per CPU cycle, [IDL] idle microsecs, [FPS] Frames per second, [FND] FPS w/no delay applied )
- F9 Volume down
- F10 Volume up
- F11 Hard reset
- F12 Reset ESP32
- PrntScr SCR screen capture
- ScrollLock Switch "Cursor keys as joy" setting
- Pause Pause
- SHIFT + F1 Hardware info
- SHIFT + F2 Save state
- SHIFT + F5 Tape Browser
- SHIFT + F6 Eject tape
- SHIFT + F9 Cheats (POK)
- SHIFT + PrintScr BMP screen capture (Folder /.c at SDCard)
- CTRL + F1 Show current machine keyboard layout
- CTRL + F2 Cycle turbo modes -> 100% speed (blue OSD), 125% speed (red OSD), 150% speed (magenta OSD), and MAX speed (black speed and no sound)
- CTRL + F5..F8 Screen centering in CRT 15K/50hz mode
- CTRL + F9 Input poke
- CTRL + F10 NMI
- CTRL + F11 Reset to TR-DOS

## ZX Keyboard functions

Press CAPS SHIFT + SYMBOL SHIFT and:

- 1 Main menu
- 2 Load state
- 3 Save state
- 5 File browser
- 6 Tape browser
- 7 Play/Stop tape
- 8 CPU / Tape load stats ( [CPU] microsecs per CPU cycle, [IDL] idle microsecs, [FPS] Frames per second, [FND] FPS w/no delay applied )
- 9 Volume down
- 0 Volume up
- Q Hard reset
- W Reset ESP32
- E Eject tape
- R Reset to TR-DOS
- T Cycle turbo modes -> 100% speed (blue OSD), 125% speed (red OSD), 150% speed (magenta OSD), and MAX speed (black speed and no sound)
- I Hardware info
- U Cheats (POK)
- O Input poke
- P Pause
- K Show current machine keyboard layout
- Z,X,C,V Screen centering in CRT 15K mode
- G SCR screen capture
- B BMP screen capture (Folder /.c at SDCard)
- N NMI
- J Enable real tape load

Press SYMBOL SHIFT and:

- S Stop real tape load

## How to flash custom ROMs

Three custom ROMs can be installed: one for the 48K architecture, another for the 128K architecture, and another one for TK90X.

The "Update firmware" option is now changed to the "Update" menu with four options: firmware, custom ROM 48K, custom ROM 128K, and custom ROM TK90X.

You can put the ROM files (.rom extension) anywhere and select it with the browser.

For the 48K and TK90X architectures, the ROM file size must be 16384 bytes.

For the 128K architecture, it can be either 16kb or 32kb. If it's 16kb, the second bank of the custom ROM will be flashed with the second bank of the standard Sinclair 128K ROM.

It is important to note that for custom ROMs, fast loading of taps can be used, but the loading should be started manually, considering the possibility that the "traps" of the ROM loading routine might not work depending on the flashed ROM. For example, with Rodolfo Guerra's ROMs, both loading and recording traps using the SAVE command work perfectly.

Finally, keep in mind that when updating the firmware, you will need to re-flash the custom ROMs afterward, so I recommend leaving ROM files you wish to use on the SD card.

## Hardware configuration and pinout

Pin assignment in `hardpins.h` is set to match the boards we've tested emulator in, use it as-is, or change it to your own preference.

## Supported hardware

- [Lilygo FabGL VGA32](https://www.lilygo.cc/products/fabgl-vga32?_pos=1&_sid=b28e8cac0&_ss=r)
- [Antonio Villena's ESPectrum board](https://antoniovillena.es/store/product/espectrum/) and [ESPjoy add-on](https://antoniovillena.es/store/product/espjoy-for-espectrum/)
- [ESP32-SBC-FabGL board from Olimex](https://www.olimex.com/Products/Retro-Computers/ESP32-SBC-FabGL/open-source-hardware)

## Based on work from:

- Víctor Iborra [Eremus] and David Crespo [dcrespo3d] [ESPectrum](https://github.com/EremusOne/ZX-ESPectrum-IDF)
- David Crespo [dcrespo3d] [ZX-ESPectrum-Wiimote](https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote)
- Ramón Martinez and Jorge Fuertes [ZX-ESPectrum](https://github.com/rampa069/ZX-ESPectrum)
- Pete Todd [paseVGA](https://github.com/retrogubbins/paseVGA)

## Acknowledgements

Special thanks to all the developers and contributors involved in the aforementioned projects for their outstanding work and commitment to open-source retro computing. For more details, please refer to each project's page.

## Project links

[GitHub](https://github.com/SplinterGU/ESPeccy)

[Telegram](https://t.me/ESPeccy)

[Discord](https://discord.gg/jRqMbTHhzt)

[Patreon](https://www.patreon.com/SplinterGU)

## Support the Project

If you enjoy using **ESPeccy** and would like to show your support, you have the option to make a donation. Thank you for being part of this journey!

[![Donate to this project](https://www.paypalobjects.com/webstatic/en_US/i/buttons/PP_logo_h_100x26.png)](https://paypal.me/bennugd?country.x=AR&locale.x=es_XC)

Or become a patron on Patreon and support continued development:

[![Become a Patron](https://c5.patreon.com/external/logo/become_a_patron_button.png)](https://www.patreon.com/SplinterGU)
