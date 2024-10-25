/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
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

To Contact the dev team you can write to zxespectrum@gmail.com or
visit https://zxespectrum.speccy.org/contacto

*/

#ifndef ESPECTRUM_MESSAGES_h
#define ESPECTRUM_MESSAGES_h

#define EMU_VERSION "       v1.3 "

// Language files
#include "messages_en.h"
#include "messages_es.h"
#include "messages_pt.h"

// General
#define NLANGS 3

static const int LANGCODEPAGE[NLANGS] = { 437, 437, 860};

#define MSG_LOADING_SNA "Loading SNA file"
#define MSG_LOADING_Z80 "Loading Z80 file"
#define MSG_SAVE_CONFIG "Saving config file"
#define MSG_VGA_INIT "Initializing VGA"

// Error
#define ERROR_TITLE "  !!!   ERROR - CLIVE MEDITATION   !!!  "
#define ERROR_BOTTOM "  Sir Clive is smoking in the Rolls...  "
#define ERR_READ_FILE "Cannot read file!"
#define ERR_BANK_FAIL "Failed to allocate RAM bank"
#define ERR_FS_INT_FAIL "Cannot mount internal storage!"

static const char *ERR_FS_EXT_FAIL[NLANGS] = { ERR_FS_EXT_FAIL_EN, ERR_FS_EXT_FAIL_ES, ERR_FS_EXT_FAIL_PT };

#define ERR_DIR_OPEN "Cannot open directory!"

// OSD
#define OSD_TITLE  " ESPeccy - The ESP32 powered emulator       "
//#define OSD_BOTTOM " zxespectrum.speccy.org     " EMU_VERSION
#define OSD_BOTTOM "                            "

static const char *OSD_MSGDIALOG_YES[NLANGS] = { OSD_MSGDIALOG_YES_EN, OSD_MSGDIALOG_YES_ES, OSD_MSGDIALOG_YES_PT };
static const char *OSD_MSGDIALOG_NO[NLANGS] = { OSD_MSGDIALOG_NO_EN, OSD_MSGDIALOG_NO_ES, OSD_MSGDIALOG_NO_PT };

static const char *OSD_PAUSE[NLANGS] = { OSD_PAUSE_EN,OSD_PAUSE_ES,OSD_PAUSE_PT };

#define OSD_PSNA_NOT_AVAIL "No Snapshot Available"
#define OSD_PSNA_LOADING "Loading Snapshot"
#define OSD_PSNA_SAVING  "Saving Snapshot"
#define OSD_PSNA_SAVE_WARN "Disk error. Trying slow mode, be patient"
#define OSD_PSNA_SAVE_ERR "ERROR Saving Snapshot"
#define OSD_PSNA_LOADED  "Snapshot Loaded"
#define OSD_PSNA_LOAD_ERR "ERROR Loading Snapshot"
#define OSD_PSNA_SAVED  "Snapshot Saved"
#define OSD_TAPE_LOAD_ERR "ERROR Loading tape file"
#define OSD_TAPE_SAVE_ERR "ERROR Saving tape file"
#define OSD_BETADISK_LOAD_ERR "ERROR Loading Disk file"

#define OSD_PLEASE_WAIT "Please Wait..."

static const char *OSD_READONLY_FILE_WARN[NLANGS] = { OSD_READONLY_FILE_WARN_EN, OSD_READONLY_FILE_WARN_ES, OSD_READONLY_FILE_WARN_PT };

static const char *OSD_TAPE_FLASHLOAD[NLANGS] = { OSD_TAPE_FLASHLOAD_EN, OSD_TAPE_FLASHLOAD_ES, OSD_TAPE_FLASHLOAD_PT };

static const char *OSD_TAPE_INSERT[NLANGS] = { OSD_TAPE_INSERT_EN, OSD_TAPE_INSERT_ES, OSD_TAPE_INSERT_PT };

static const char *POKE_ERR_ADDR1[NLANGS] = { POKE_ERR_ADDR1_EN, POKE_ERR_ADDR1_ES, POKE_ERR_ADDR1_PT };

static const char *POKE_ERR_ADDR2[NLANGS] = { POKE_ERR_ADDR2_EN, POKE_ERR_ADDR2_ES, POKE_ERR_ADDR2_PT };

static const char *POKE_ERR_VALUE[NLANGS] = { POKE_ERR_VALUE_EN, POKE_ERR_VALUE_ES, POKE_ERR_VALUE_PT };

static const char *OSD_INVALIDCHAR[NLANGS] = { OSD_INVALIDCHAR_EN, OSD_INVALIDCHAR_ES, OSD_INVALIDCHAR_PT };

static const char *OSD_TAPE_SAVE[NLANGS] = { OSD_TAPE_SAVE_EN, OSD_TAPE_SAVE_ES, OSD_TAPE_SAVE_PT };

static const char *OSD_TAPE_SAVE_EXIST[NLANGS] = { OSD_TAPE_SAVE_EXIST_EN, OSD_TAPE_SAVE_EXIST_ES, OSD_TAPE_SAVE_EXIST_PT };

static const char *OSD_PSNA_SAVE[NLANGS] = { OSD_PSNA_SAVE_EN, OSD_PSNA_SAVE_ES, OSD_PSNA_SAVE_PT };

static const char *OSD_PSNA_EXISTS[NLANGS] = { OSD_PSNA_EXISTS_EN, OSD_PSNA_EXISTS_ES, OSD_PSNA_EXISTS_PT };

static const char *OSD_TAPE_SELECT_ERR[NLANGS] = { OSD_TAPE_SELECT_ERR_EN,OSD_TAPE_SELECT_ERR_ES, OSD_TAPE_SELECT_ERR_PT };

static const char *OSD_FILE_INDEXING[NLANGS] = { OSD_FILE_INDEXING_EN, OSD_FILE_INDEXING_ES, OSD_FILE_INDEXING_PT };

static const char *OSD_FILE_INDEXING_1[NLANGS] = { OSD_FILE_INDEXING_EN_1, OSD_FILE_INDEXING_ES_1, OSD_FILE_INDEXING_PT_1 };

static const char *OSD_FILE_INDEXING_2[NLANGS] = { OSD_FILE_INDEXING_EN_2, OSD_FILE_INDEXING_ES_2, OSD_FILE_INDEXING_PT_2 };

static const char *OSD_FILE_INDEXING_3[NLANGS] = { OSD_FILE_INDEXING_EN_3, OSD_FILE_INDEXING_ES_3, OSD_FILE_INDEXING_PT_3 };

static const char *OSD_FIRMW_UPDATE[NLANGS] = { OSD_FIRMW_UPDATE_EN,OSD_FIRMW_UPDATE_ES, OSD_FIRMW_UPDATE_PT};

static const char *OSD_DLG_SURE[NLANGS] = { OSD_DLG_SURE_EN, OSD_DLG_SURE_ES, OSD_DLG_SURE_PT};

static const char *OSD_DLG_JOYSAVE[NLANGS] = { OSD_DLG_JOYSAVE_EN, OSD_DLG_JOYSAVE_ES, OSD_DLG_JOYSAVE_PT};

static const char *OSD_DLG_JOYDISCARD[NLANGS] = { OSD_DLG_JOYDISCARD_EN, OSD_DLG_JOYDISCARD_ES, OSD_DLG_JOYDISCARD_PT};

static const char *OSD_DLG_SETJOYMAPDEFAULTS[NLANGS] = { OSD_DLG_SETJOYMAPDEFAULTS_EN, OSD_DLG_SETJOYMAPDEFAULTS_ES, OSD_DLG_SETJOYMAPDEFAULTS_PT};

static const char *OSD_FIRMW[NLANGS] = { OSD_FIRMW_EN,OSD_FIRMW_ES, OSD_FIRMW_PT};

static const char *OSD_FIRMW_BEGIN[NLANGS] = { OSD_FIRMW_BEGIN_EN,OSD_FIRMW_BEGIN_ES, OSD_FIRMW_BEGIN_PT};

static const char *OSD_FIRMW_WRITE[NLANGS] = { OSD_FIRMW_WRITE_EN,OSD_FIRMW_WRITE_ES, OSD_FIRMW_WRITE_PT};

static const char *OSD_FIRMW_END[NLANGS] = { OSD_FIRMW_END_EN,OSD_FIRMW_END_ES, OSD_FIRMW_END_PT};

static const char *OSD_NOFIRMW_ERR[NLANGS] = { OSD_NOFIRMW_ERR_EN,OSD_NOFIRMW_ERR_ES, OSD_NOFIRMW_ERR_PT};

static const char *OSD_FIRMW_ERR[NLANGS] = { OSD_FIRMW_ERR_EN,OSD_FIRMW_ERR_ES, OSD_FIRMW_ERR_PT};

static const char *MENU_ROM_TITLE[NLANGS] = { MENU_ROM_TITLE_EN,MENU_ROM_TITLE_ES,MENU_ROM_TITLE_PT};

static const char *OSD_ROM_ERR[NLANGS] = { OSD_ROM_ERR_EN,OSD_ROM_ERR_ES,OSD_ROM_ERR_PT};

static const char *OSD_NOROMFILE_ERR[NLANGS] = { OSD_NOROMFILE_ERR_EN,OSD_NOROMFILE_ERR_ES,OSD_NOROMFILE_ERR_PT};

static const char *OSD_ROM[NLANGS] = { OSD_ROM_EN,OSD_ROM_ES,OSD_ROM_PT};

static const char *OSD_ROM_BEGIN[NLANGS] = { OSD_ROM_BEGIN_EN,OSD_ROM_BEGIN_ES,OSD_ROM_BEGIN_PT};

static const char *OSD_ROM_WRITE[NLANGS] = { OSD_ROM_WRITE_EN,OSD_ROM_WRITE_ES,OSD_ROM_WRITE_PT};

static const char *MENU_SNA_TITLE[NLANGS] = { MENU_SNA_TITLE_EN,MENU_SNA_TITLE_ES,MENU_SNA_TITLE_PT};

static const char *MENU_SAVE_SNA_TITLE[NLANGS] = { MENU_SAVE_SNA_TITLE_EN,MENU_SAVE_SNA_TITLE_ES,MENU_SAVE_SNA_TITLE_PT};

static const char *MENU_TAP_TITLE[NLANGS] = { MENU_TAP_TITLE_EN,MENU_TAP_TITLE_ES,MENU_TAP_TITLE_PT};

static const char *MENU_DSK_TITLE[NLANGS] = { MENU_DSK_TITLE_EN,MENU_DSK_TITLE_ES,MENU_DSK_TITLE_PT};

static const char *MENU_ESP_LOAD_TITLE[NLANGS] = { MENU_ESP_LOAD_TITLE_EN,MENU_ESP_LOAD_TITLE_ES,MENU_ESP_LOAD_TITLE_PT};

static const char *MENU_ESP_SAVE_TITLE[NLANGS] = { MENU_ESP_SAVE_TITLE_EN,MENU_ESP_SAVE_TITLE_ES,MENU_ESP_SAVE_TITLE_PT};

static const char *MENU_CHT_TITLE[NLANGS] = { MENU_CHT_TITLE_EN,MENU_CHT_TITLE_ES,MENU_CHT_TITLE_PT};

static const char *MENU_DELETE_TAP_BLOCKS[NLANGS] = { MENU_DELETE_TAP_BLOCKS_EN,MENU_DELETE_TAP_BLOCKS_ES,MENU_DELETE_TAP_BLOCKS_PT};

static const char *MENU_DELETE_CURRENT_TAP_BLOCK[NLANGS] = { MENU_DELETE_CURRENT_TAP_BLOCK_EN,MENU_DELETE_CURRENT_TAP_BLOCK_ES,MENU_DELETE_CURRENT_TAP_BLOCK_PT};

static const char *OSD_BLOCK_SELECT_ERR[NLANGS] = { OSD_BLOCK_SELECT_ERR_EN,OSD_BLOCK_SELECT_ERR_ES,OSD_BLOCK_SELECT_ERR_PT};

static const char *OSD_BLOCK_TYPE_ERR[NLANGS] = { OSD_BLOCK_TYPE_ERR_EN,OSD_BLOCK_TYPE_ERR_ES,OSD_BLOCK_TYPE_ERR_PT};

static const char *MENU_DELETE_CURRENT_FILE[NLANGS] = { MENU_DELETE_CURRENT_FILE_EN,MENU_DELETE_CURRENT_FILE_ES,MENU_DELETE_CURRENT_FILE_PT};

static const char *OSD_TAPE_EJECT[NLANGS] = { OSD_TAPE_EJECT_EN,OSD_TAPE_EJECT_ES,OSD_TAPE_EJECT_PT};

static const char *MENU_DELETE_SNA[NLANGS] = { MENU_DELETE_SNA_EN,MENU_DELETE_SNA_ES,MENU_DELETE_SNA_PT};

static const char *TRDOS_RESET_ERR[NLANGS] = { TRDOS_RESET_ERR_EN,TRDOS_RESET_ERR_ES,TRDOS_RESET_ERR_PT};

static const char *MENU_SNA[NLANGS] = { MENU_SNA_EN,MENU_SNA_ES,MENU_SNA_PT};

static const char *MENU_TAPE[NLANGS] = { MENU_TAPE_EN,MENU_TAPE_ES,MENU_TAPE_PT};

static const char *MENU_TAPEPLAYER[NLANGS] = { "Player mode\n", "Modo reproductor\n", "Modo reprodutor\n" };

static const char *MENU_BETADISK[NLANGS] = { MENU_BETADISK_EN,MENU_BETADISK_ES,MENU_BETADISK_PT};

static const char *MENU_BETADRIVE[NLANGS] = { MENU_BETADRIVE_EN,MENU_BETADRIVE_ES,MENU_BETADRIVE_PT};

static const char *MENU_MAIN[3] = { MENU_MAIN_EN,MENU_MAIN_ES,MENU_MAIN_PT};

static const char *MENU_OPTIONS[NLANGS] = { MENU_OPTIONS_EN,MENU_OPTIONS_ES,MENU_OPTIONS_PT};

static const char *MENU_UPDATE_FW[NLANGS] = { MENU_UPDATE_EN,MENU_UPDATE_ES,MENU_UPDATE_PT};

static const char *MENU_VIDEO[NLANGS] = { MENU_VIDEO_EN, MENU_VIDEO_ES, MENU_VIDEO_PT};

static const char *MENU_RENDER[NLANGS] = { MENU_RENDER_EN, MENU_RENDER_ES, MENU_RENDER_PT};

static const char *MENU_ASPECT[NLANGS] = { MENU_ASPECT_EN, MENU_ASPECT_ES, MENU_ASPECT_PT};

static const char *MENU_SCANLINES[NLANGS] = { "Scanlines\n", "Scanlines\n", "Scanlines\n"};

static const char *MENU_RESET[NLANGS] = { MENU_RESET_EN, MENU_RESET_ES, MENU_RESET_PT};

static const char *MENU_PERSIST_SAVE[NLANGS] = { MENU_PERSIST_SAVE_EN, MENU_PERSIST_SAVE_ES, MENU_PERSIST_SAVE_PT};

static const char *MENU_PERSIST_LOAD[NLANGS] = { MENU_PERSIST_LOAD_EN, MENU_PERSIST_LOAD_ES, MENU_PERSIST_LOAD_PT};

static const char *MENU_STORAGE[NLANGS] = { MENU_STORAGE_EN, MENU_STORAGE_ES, MENU_STORAGE_PT};

static const char *MENU_YESNO[NLANGS] = { MENU_YESNO_EN, MENU_YESNO_ES, MENU_YESNO_PT};

static const char *MENU_DISKCTRL[NLANGS] = { "Betadisk\n" , "Betadisk\n", "Betadisk\n"};

static const char *MENU_FLASHLOAD[NLANGS] = { "Flash load\n" , "Carga r\xA0pida\n", "Carga r\xA0pida\n"};

static const char *MENU_RGTIMINGS[NLANGS] = { "R.G. Timings\n" , "Timings R.G.\n", "Timings R.G.\n"};

static const char *MENU_OTHER[NLANGS] = { MENU_OTHER_EN, MENU_OTHER_ES, MENU_OTHER_PT};

static const char *MENU_AY48[NLANGS] = { "AY on 48K \n" , "AY en 48K \n" , "AY em 48K \n"};

#define MENU_ALUTK "Ferranti\t[F]\n"\
    "Microdigital 50hz\t[5]\n"\
	"Microdigital 60hz\t[6]\n"
static const char *MENU_ALUTK_PREF[NLANGS] = { "TK ULA\n" MENU_ALUTK, "ULA TK\n" MENU_ALUTK, "ULA TK\n" MENU_ALUTK};

static const char *MENU_KBD2NDPS2[NLANGS] = { MENU_KBD2NDPS2_EN, MENU_KBD2NDPS2_ES, MENU_KBD2NDPS2_PT};

static const char *MENU_ALUTIMING[NLANGS] = { MENU_ALUTIMING_EN, MENU_ALUTIMING_ES, MENU_ALUTIMING_PT};

static const char *MENU_ISSUE2[NLANGS] = { "48K Issue 2\n", "48K Issue 2\n", "48K Issue 2\n"};

#define MENU_ARCHS "Spectrum 48K\t>\n"\
    "Spectrum 128K\t>\n"\
	"Pentagon 128K\n"\
	"TK90X\t>\n"\
	"TK95\t>\n"

static const char *MENU_ARCH[NLANGS] = { MENU_ARCH_EN MENU_ARCHS, MENU_ARCH_ES MENU_ARCHS, MENU_ARCH_PT MENU_ARCHS};

static const char *MENU_ROMS48[NLANGS] = { MENU_ROMS48_EN, MENU_ROMS48_ES, MENU_ROMS48_PT};

static const char *MENU_ROMS128[NLANGS] = { MENU_ROMS128_EN, MENU_ROMS128_ES, MENU_ROMS128_PT};

static const char *MENU_ROMSTK[NLANGS] = { MENU_ROMSTK_EN, MENU_ROMSTK_ES, MENU_ROMSTK_PT};

static const char *MENU_ROMSTK95[NLANGS] = { MENU_ROMSTK95_EN, MENU_ROMSTK95_ES, MENU_ROMSTK95_PT};

#define MENU_ARCHS_PREF "Spectrum 48K\t[4]\n"\
    "Spectrum 128K\t[1]\n"\
	"Pentagon 128K\t[P]\n"\
	"TK90X\t[T]\n"\
	"TK95\t[9]\n"

static const char *MENU_ARCH_PREF[NLANGS] = {
	"Preferred machine\n" MENU_ARCHS_PREF "Last used\t[L]\n",
	"Modelo preferido\n" MENU_ARCHS_PREF "Ultimo utilizado\t[L]\n",
	"Hardware favorito\n" MENU_ARCHS_PREF "Usado por \xA3ltimo\t[L]\n"
	};

#define MENU_ROMS_PREF "Spectrum 48K\t>\n"\
    "Spectrum 128K\t>\n"\
	"TK90X\t>\n"\
	"TK95\t>\n"

static const char *MENU_ROM_PREF[NLANGS] = { "Preferred ROM\n" MENU_ROMS_PREF, "ROM preferida\n" MENU_ROMS_PREF, "ROM favorita\n" MENU_ROMS_PREF};

static const char *MENU_ROM_PREF_48[NLANGS] = {
	"Select ROM\n" MENU_ROMS48_PREF_EN "Last used\t[Last]\n",
	"Elija ROM\n" MENU_ROMS48_PREF_ES "Ultima usada\t[Last]\n",
	"Escolha ROM\n" MENU_ROMS48_PREF_PT "Usada por \xA3ltimo\t[Last]\n"
	};

static const char *MENU_ROM_PREF_TK90X[NLANGS] = {
	"Select ROM\n" MENU_ROMSTK90X_PREF_EN "Last used\t[Last ]\n",
	"Elija ROM\n" MENU_ROMSTK90X_PREF_ES "Ultima usada\t[Last ]\n",
	"Escolha ROM\n" MENU_ROMSTK90X_PREF_PT "Usada por \xA3ltimo\t[Last ]\n"
	};

static const char *MENU_ROM_PREF_128[NLANGS] = {
	"Select ROM\n" MENU_ROMS128_PREF_EN "Last used\t[Last]\n",
	"Elija ROM\n" MENU_ROMS128_PREF_ES "Ultima usada\t[Last]\n",
	"Escolha ROM\n" MENU_ROMS128_PREF_PT "Usada por \xA3ltimo\t[Last]\n"
	};

static const char *MENU_ROM_PREF_TK95[NLANGS] = {
	"Select ROM\n" MENU_ROMSTK95_PREF_EN"Last used\t[Last ]\n",
	"Elija ROM\n" MENU_ROMSTK95_PREF_ES "Ultima usada\t[Last ]\n",
	"Escolha ROM\n" MENU_ROMSTK95_PREF_PT "Usada por \xA3ltimo\t[Last ]\n"
	};

static const char *MENU_INTERFACE_LANG[3] = { MENU_INTERFACE_LANG_EN, MENU_INTERFACE_LANG_ES, MENU_INTERFACE_LANG_PT };

#define MENU_JOYS "Joystick 1\n"\
    "Joystick 2\n"

static const char *MENU_JOY[NLANGS] = { MENU_JOY_EN MENU_JOYS, MENU_JOY_ES MENU_JOYS, MENU_JOY_PT MENU_JOYS};

#define MENU_DEFJOY_TITLE "Joystick#\n"\

#define MENU_DEFJOYS "Cursor\t[ ]\n"\
    "Kempston\t[ ]\n"\
    "Sinclair 1\t[ ]\n"\
    "Sinclair 2\t[ ]\n"\
    "Fuller\t[ ]\n"

static const char *MENU_DEFJOY[NLANGS] = { MENU_DEFJOY_TITLE MENU_DEFJOYS MENU_DEFJOY_EN, MENU_DEFJOY_TITLE MENU_DEFJOYS MENU_DEFJOY_ES, MENU_DEFJOY_TITLE MENU_DEFJOYS MENU_DEFJOY_PT};

static const char *MENU_JOYPS2[NLANGS] = { MENU_JOYPS2_EN, MENU_JOYPS2_ES, MENU_JOYPS2_PT};

static const char *MENU_PS2JOYTYPE[NLANGS] = { "Joy type\n" MENU_DEFJOYS, "Tipo joystick\n" MENU_DEFJOYS, "Tipo joystick\n" MENU_DEFJOYS};

static const char *MENU_CURSORJOY[NLANGS] = { "Cursor as Joy\n" , "Joy en Cursor\n" , "Joy no Cursor\n"};

static const char *MENU_TABASFIRE[NLANGS] = { "TAB as fire 1\n" , "TAB disparo 1\n" , "TAB fire 1\n"};

static const char *DLG_TITLE_INPUTPOK[NLANGS] = { DLG_TITLE_INPUTPOK_EN, DLG_TITLE_INPUTPOK_ES, DLG_TITLE_INPUTPOK_PT };

static const char *POKE_BANK_MENU[NLANGS] = { " Bank  \n" , " Banco \n" , " Banco \n"};

static const char *MENU_OSD[NLANGS] = { MENU_OSD_EN, MENU_OSD_ES, MENU_OSD_PT };

static const char *MENU_OSD_OPT1[NLANGS] = {"Option \n","Opcion \n","Opo \n"};

static const char *MENU_OSD_OPT2[NLANGS] = {"Option \n","Opcion \n","Opo \n"};


#define DEDICATORIA "\nF1Dedicado especialmente a:\r"\
	"\nB1      _       _ _\r"\
	"\nB1     | |     | (_)          \nA1d88b d88b\r"\
	"\nB1     | |_   _| |_  __ _    \nA188888888888\r"\
	"\nB1 _   | | | | | | |/ _` |   \nA1`Y8888888Y'\r"\
	"\nB1| |__| | |_| | | | (_| |     \nA1`Y888Y'\r"\
	"\nB1 \\____/ \\___/|_|_|\\__,_|       \nA1`Y'\r"\
	"\nF1 _   _    \nE1 __  __            _ \r"\
	"\nF1| | | |   \nE1|  \\/  |          | |\r"\
	"\nF1| |_| |   \nE1| \\  / | __ _ _ __| |_ __ _\r"\
	"\nF1 \\__, |   \nE1| |\\/| |/ _` | '__| __/ _` |\r"\
	"\nF1  __/ |   \nE1| |  | | (_| | |  | || (_| |\r"\
	"\nF1 |___/    \nE1|_|  |_|\\__,_|_|   \\__\\__,_|"

// 38 characters wide for about text
#define PATREONS "\r"\
	"\nA1The Mega Trees:\r"\
	"\r"\
	"\nB1V\xA1" "ctor Llamazares \nC1Antonio Villena\r"\
	"\r"\
	"\nA1The Jet Set Willys:\r"\
	"\r"\
	"\nB1DopierRex \nC1Germ\xA0n Filera \nD1Juan C. Galea\r"\
	"\nE1Juanje \nB1Ra\xA3l Jim\x82nez \nC1Juanma Mart\xA1n\r"\
	"\nD1Seraf\xA1n Morat\xA2n \nE1Eduard Ruiz\r"\
	"\nB1Igor Peruchi \nC1In\xA0" "cio Santos\r"\
	"\nD1Destroyer"

#define PATREONS2 "\r"\
	"\nA1The Manic Miners:\r"\
	"\r"\
	"\nB1Lencio Asimov \nC1Fernando Bonilla\r"\
	"\nD1Juan Conde Luque \nE1Fidel Fern\xA0ndez\r"\
	"\nB1Alberto Garc\xA1" "a \nC1Francisco Garc\xA1" "a\r"\
	"\nD1Jorge Garc\xA1" "a \nE1Jos\x82 Luis Garc\xA1" "a\r"\
	"\nB1Nacho Izquierdo \nC1kounch \nD1V\xA1" "ctor Lorenzo\r"\
	"\nE1Luis Maldonado \nB1Mananuk \nC1Ignacio Monge\r"\
	"\nD1Vicente Morales \nE1Pablo Mu\xA4oz\r"\
	"\nB1Javi Ortiz \nC1Miguel Angel P\x82rez\r"\
	"\nD1Pascual P\x82rez \nE1Juan Jos\x82 Piernas"\

#define PATREONS3 "\r"\
	"\nA1The Manic Miners:\r"\
	"\r"\
	"\nB1Radastan \nC1Jordi Ramos \nD1Gustavo Reynaga\r"\
	"\nE1Jos\x82 M. Rodr\xA1guez \nB1Marco A. Rodr\xA1guez\r"\
	"\nC1Santiago Romero \nD1Julia Salvador\r"\
	"\nE1Juan Diego S\xA0nchez \nB1Marta Sicilia\r"\
	"\nC1Fco. Jos\x82 Soldado \nD1Vida Extra Retro\r"\
	"\nE1Radek Wojciechowski \nB1Jes\xA3s Mu\xA4oz\r"\
	"\nC1Antonio Jes\xA3s S\xA0nchez \nD1Gregorio P\x82rez\r"\
	"\nE1Leonardo Coca\xA4" "a \nB1Manuel Cuenca\r"\
	"\nC1Ovi P. \nD1Jos\x82 Medina \nE1Miguel A. Montejo"

#define PATREONS4 "\r"\
	"\nA1The Manic Miners:\r"\
	"\r"\
	"\nB1Jakub Rzepecki \nC1Seb \nD1Sim\xA2n G\xA2mez\r"\
	"\nE1V\xA1" "ctor Salado \nB1Miguel A. Gonz\xA0lez\r"\
	"\nC1Alberto Navarro \nD1Alejandro Molina\r"\
	"\nE1Alfonso D\xA1" "az \nB1Carlos Mart\xA1" "nez\r"\
	"\nC1Clovis Friolani \nD1Daniel S\xA0" "ez\r"\
	"\nE1Denis Estevez \nB1Fasih Rehman\r"\
	"\nC1Fernando Mirones \nD1Ismael Salvador\r"\
	"\nE1Jorge Plazas \nB1Juan A. Rubio\r"\
	"\nC1Lucio Rial \nD1Mark Cohen"

#define PATREONS5 "\r"\
	"\nA1The Manic Miners:\r"\
	"\r"\
	"\nB1Mike van der Lee \nC1Monica Compa\xA4\r"\
	"\nD1Roberto Paciello \nE1Ronny Verminck\r"\
	"\nB1Trevor Boys \nC1Wayne Burton"

static const char *AboutMsg[NLANGS][11] = {
	{
	"\nF1(C)2024 Juan Jos\x82 Ponteprino \"SplinterGU\"\r"\
	"\nF1(C)2023-24 V\xA1" "ctor Iborra \"Eremus\"\r"\
	"   2023 David Crespo  \"dcrespo3d\"\r"\
	"\r"\
	"\nA1Based on ZX-ESPectrum-Wiimote\r"\
	"(C)2020-2023 David Crespo\r"\
	"\r"\
	"\nB1Inspired by previous projects\r"\
	"from Pete Todd and Rampa & Queru\r"\
	"\r"\
	"\nC1Z80 emulation by J.L. S\xA0nchez\r"\
	"\nD1VGA driver by BitLuni\r"\
	"\nE1AY-3-8912 library by A. Sashnov\r"\
	"\nF1PS2 driver by Fabrizio di Vittorio"
	,
	"\nF1Collaborators:\r"\
	"\r"\
	"\nA1ackerman        \nF1Code & ideas\r"\
	"\nB1Armand          \nF1Testing & broadcasting\r"\
	"\nC1azesmbog        \nF1Testing & ideas\r"\

	"\nD1Carlo Brini     \nF1Our UK guy ;)\r"\
	"\nE1David Carri\xA2n   \nF1H/W code, ZX kbd\r"\
	"\nA1Rodolfo Guerra  \nF1Our LATAM guy ;)"
	,
	"\nF1Collaborators:\r"\
	"\r"\
	"\nB1Ram\xA2n Mart\xA1nez  \nF1AY emul. improvements\r"\
	"\nD1Ron             \nF1Testing & broadcasting\r"\
	"\nE1J.L. S\xA0nchez    \nF1Z80 core improvements\r"\
	"\nA1Antonio Villena \nF1Hardware support\r"\
	"\nB1ZjoyKiLer       \nF1Testing & ideas"
	,
	"\nF1Big thanks to our Patreons:\r"\
	PATREONS
	,
	"\nF1Big thanks to our Patreons:\r"\
	PATREONS2
	,
	"\nF1Big thanks to our Patreons:\r"\
	PATREONS3
	,
	"\nF1Big thanks to our Patreons:\r"\
	PATREONS4
	,
	"\nF1Big thanks to our Patreons:\r"\
	PATREONS5
	,
	"\nF1Thanks for help and donations to:\r"\
	"\r"\
	"\nA1Abel Bayon @Baycorps \nF1Amstrad Eterno\r"\
	"\nB1Pablo Forcen Soler \nF1AUA\r"\
	"\nC1Jordi Ramos Montes\r"
	"\nD1Tsvetan Usunov \nF1Olimex Ltd.\r"\
	"\r"\
	"\nF1ZX81+ ROM included courtesy of:\r"\
	"\r"\
	"\nA1Paul Farrow"
	,
	"\nF1Thanks also to:\r"\
	"\r"\
	"\nA1Retrowiki.es \nF1and its great community\r"\
	"\nB1Ron \nF1for his cool RetroCrypta\r"\
	"\nC1Viejoven FX\nF1, \nD1J.Ortiz \"El Spectrumero\"\r"
	"\nE1J.C. Gonz\xA0lez Amestoy \nF1for RVM\r"\
	"\nA1VidaExtraRetro, \nB1C\x82sar Nicol\xA0s-Gonz\xA0lez\r"\
	"\nC1Rodolfo Guerra, \nD1All creators in\r"\
	"ZX Spectrum server at Discord\r"\
	"\r"\
	"\nF1and, of course, to:\r"\
	"\r"\
	"\nD1Sir Clive Sinclair \nF1& \nA1M\nE1a\nC1t\nD1t\nB1h\nA1e\nE1w \nC1S\nD1m\nB1i\nA1t\nE1h"
	,
	DEDICATORIA
	},
	{
	"\nF1(C)2024 Juan Jos\x82 Ponteprino \"SplinterGU\"\r"\
	"\nF1(C)2023-24 V\xA1" "ctor Iborra \"Eremus\"\r"\
	"   2023 David Crespo  \"dcrespo3d\"\r"\
	"\r"\
	"\nA1Basado en ZX-ESPectrum-Wiimote\r"\
	"(C)2020-2023 David Crespo\r"\
	"\r"\
	"\nB1Inspirado en proyectos anteriores\r"\
	"de Pete Todd y Rampa & Queru\r"\
	"\r"\
	"\nC1Emulaci\xA2n Z80 por J.L. S\xA0nchez\r"\
	"\nD1Driver VGA por BitLuni\r"\
	"\nE1Librer\xA1" "a AY-3-8912 por A. Sashnov\r"\
	"\nF1Driver PS2 por Fabrizio di Vittorio"
	,
	"\nF1Colaboradores:\r"\
	"\r"\
	"\nA1ackerman        \nF1C\xA2" "digo e ideas\r"\
	"\nB1Armand          \nF1Testing y difusi\xA2n\r"\
	"\nC1azesmbog        \nF1Testing e ideas\r"\
	"\nD1Carlo Brini     \nF1ESPectrum en UK ;)\r"\
	"\nE1David Carri\xA2n   \nF1C\xA2" "digo h/w, teclado ZX\r"\
	"\nA1Rodolfo Guerra  \nF1ESPectrum en LATAM ;)"\
	,
	"\nF1Colaboradores:\r"\
	"\r"\
	"\nE1Ramon Mart\xA1nez  \nF1Mejoras emulaci\xA2n AY\r"\
	"\nB1Ron             \nF1Testing y difusi\xA2n\r"\
	"\nC1J.L. S\xA0nchez    \nF1Mejoras core Z80\r"\
	"\nD1Antonio Villena \nF1Soporte hardware\r"\
	"\nE1ZjoyKiLer       \nF1Testing e ideas"\
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS2
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS3
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS4
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS5
	,
	"\nF1Gracias por su ayuda y donaciones a:\r"\
	"\r"\
	"\nA1Abel Bayon @Baycorps \nF1Amstrad Eterno\r"\
	"\nB1Pablo Forcen Soler \nF1AUA\r"\
	"\nC1Jordi Ramos Montes\r"
	"\nD1Tsvetan Usunov \nF1Olimex Ltd.\r"\
	"\r"\
	"\nF1ZX81+ ROM incluida por cortes\xA1" "a de:\r"\
	"\r"\
	"\nA1Paul Farrow"\
	,
	"\nF1Gracias tambi\x82n a:\r"\
	"\r"\
	"\nA1Retrowiki.es \nF1y su magn\xA1" "fica comunidad\r"\
	"\nB1Ron \nF1por su genial RetroCrypta\r"\
	"\nC1Viejoven FX\nF1, \nD1J.Ortiz \"El Spectrumero\"\r"
	"\nE1J.C. Gonz\xA0lez Amestoy \nF1por RVM\r"\
	"\nA1VidaExtraRetro, \nB1C\x82sar Nicol\xA0s-Gonz\xA0lez\r"\
	"\nC1Rodolfo Guerra, \nD1Todos los creadores en\r"\
	"el servidor ZXSpectrum en Discord\r"\
	"\r"\
	"\nF1y, por supuesto, a:\r"\
	"\r"\
	"\nD1Sir Clive Sinclair \nF1& \nA1M\nE1a\nC1t\nD1t\nB1h\nA1e\nE1w \nC1S\nD1m\nB1i\nA1t\nE1h"
	,
	DEDICATORIA
	},
	{
	"\nF1(C)2024 Juan Jos\x82 Ponteprino \"SplinterGU\"\r"\
	"\nF1(C)2023-24 V\xA1" "ctor Iborra \"Eremus\"\r"\
	"   2023 David Crespo  \"dcrespo3d\"\r"\
	"\r"\
	"\nA1Basado en ZX-ESPectrum-Wiimote\r"\
	"(C)2020-2023 David Crespo\r"\
	"\r"\
	"\nB1Inspirado en proyectos anteriores\r"\
	"de Pete Todd y Rampa & Queru\r"\
	"\r"\
	"\nC1Emulaci\xA2n Z80 por J.L. S\xA0nchez\r"\
	"\nD1Driver VGA por BitLuni\r"\
	"\nE1Librer\xA1" "a AY-3-8912 por A. Sashnov\r"\
	"\nF1Driver PS2 por Fabrizio di Vittorio"
	,
	"\nF1Colaboradores:\r"\
	"\r"\
	"\nA1ackerman        \nF1C\xA2" "digo e ideas\r"\
	"\nB1Armand          \nF1Testing y difusi\xA2n\r"\
	"\nC1azesmbog        \nF1Testing e ideas\r"\
	"\nD1Carlo Brini     \nF1ESPectrum en UK ;)\r"\
	"\nE1David Carri\xA2n   \nF1C\xA2" "digo h/w, teclado ZX\r"\
	"\nA1Rodolfo Guerra  \nF1ESPectrum en LATAM ;)"\
	,
	"\nF1Colaboradores:\r"\
	"\r"\
	"\nE1Ramon Mart\xA1nez  \nF1Mejoras emulaci\xA2n AY\r"\
	"\nB1Ron             \nF1Testing y difusi\xA2n\r"\
	"\nC1J.L. S\xA0nchez    \nF1Mejoras core Z80\r"\
	"\nD1Antonio Villena \nF1Soporte hardware\r"\
	"\nE1ZjoyKiLer       \nF1Testing e ideas"\
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS2
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS3
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS4
	,
	"\nF1Muchas gracias a nuestros Patreons:\r"\
	PATREONS5
	,
	"\nF1Gracias por su ayuda y donaciones a:\r"\
	"\r"\
	"\nA1Abel Bayon @Baycorps \nF1Amstrad Eterno\r"\
	"\nB1Pablo Forcen Soler \nF1AUA\r"\
	"\nC1Jordi Ramos Montes\r"
	"\nD1Tsvetan Usunov \nF1Olimex Ltd.\r"\
	"\r"\
	"\nF1ZX81+ ROM incluida por cortes\xA1" "a de:\r"\
	"\r"\
	"\nA1Paul Farrow"\
	,
	"\nF1Gracias tambi\x82n a:\r"\
	"\r"\
	"\nA1Retrowiki.es \nF1y su magn\xA1" "fica comunidad\r"\
	"\nB1Ron \nF1por su genial RetroCrypta\r"\
	"\nC1Viejoven FX\nF1, \nD1J.Ortiz \"El Spectrumero\"\r"
	"\nE1J.C. Gonz\xA0lez Amestoy \nF1por RVM\r"\
	"\nA1VidaExtraRetro, \nB1C\x82sar Nicol\xA0s-Gonz\xA0lez\r"\
	"\nC1Rodolfo Guerra, \nD1Todos los creadores en\r"\
	"el servidor ZXSpectrum en Discord\r"\
	"\r"\
	"\nF1y, por supuesto, a:\r"\
	"\r"\
	"\nD1Sir Clive Sinclair \nF1& \nA1M\nE1a\nC1t\nD1t\nB1h\nA1e\nE1w \nC1S\nD1m\nB1i\nA1t\nE1h"
	,
	DEDICATORIA
	}
};

    // "           CPU: microsec. per CPU cycle\n"\
    // "           IDL: unused microsec.\n"\
    // "           FPS: Frames per second\n"\
    // "           FND: FPS without delay\n"\

// #define OSD_HELP_EN \
//     " [F1]         Main menu\n"\
//     " [F2]         Load (SNA,Z80,P)\n"\
//     " [F3-F4]      Load / Save snapshot\n"\
//     " [F5]         Select tape file\n"\
//     " [F6]         Play/Stop tape\n"\
//     " [F7]         Tape browser\n"\
//     " [F8]         CPU / Tape load stats\n"\
//     " [F9-F10]     Volume down-up\n"\
// 	" [F11]        Hard reset\n"\
//     " [F12]        Reset ESP32\n"\
//     " [CTRL+F1]    Hardware info\n"\
//     " [CTRL+F2]    Turbo mode\n"\
//     " [CTRL+F5-F8] Center CRT Screen\n"\
//     " [CTRL+F9]    Input poke\n"\
//     " [CTRL+F10]   NMI\n"\
//     " [CTRL+F11]   Reset to TR-DOS\n"\
//     " [Pause]      Pause\n"\
//     " [PrtScr]     BMP capture (folder /.c)\n"

//     // "            CPU: microsg. por ciclo CPU\n"\
//     // "            IDL: microsg. sin usar\n"\
//     // "            FPS: Frames por segundo\n"\
//     // "            FND: FPS sin delay\n"\

// #define OSD_HELP_ES \
//     " [F1]         Men\xA3 principal\n"\
//     " [F2]         Cargar (SNA,Z80,P)\n"\
//     " [F3-F4]      Cargar / Guardar snapshot\n"\
//     " [F5]         Elegir archivo de cinta\n"\
//     " [F6]         Play/Stop cinta\n"\
//     " [F7]         Explorador cinta\n"\
//     " [F8]         Status CPU / Carga cinta\n"\
//     " [F9-F10]     Bajar-Subir volumen\n"\
//     " [F11]        Reset completo\n"\
//     " [F12]        Resetear ESP32\n"\
//     " [CTRL+F1]    Info hardware\n"\
//     " [CTRL+F2]    Modo turbo\n"\
//     " [CTRL+F5-F8] Centrar pantalla CRT\n"\
//     " [CTRL+F9]    Introducir poke\n"\
//     " [CTRL+F10]   NMI\n"\
//     " [CTRL+F11]   Reset a TR-DOS\n"\
//     " [Pause]      Pausa\n"\
//     " [ImpPant]    Captura BMP (Carpeta /.c)\n"

// #define OSD_HELP_EN_ZX \
//     " Press CAPS SHIFT + SYMBOL SHIFT and:\n"\
// 	" [1]       Main menu\n"\
//     " [NLANGS]       Load (SNA,Z80,P)\n"\
//     " [3-4]     Load / Save snapshot\n"\
//     " [5]       Select tape file\n"\
//     " [6]       Play/Stop tape\n"\
//     " [7]       Tape browser\n"\
//     " [8]       CPU / Tape load stats\n"\
//     " [9-0]     Volume down-up\n"\
//     " [Q]       Hard reset\n"\
//     " [W]       Reset ESP32\n"\
//     " [R]       Reset to TR-DOS\n"\
//     " [I]       Hardware info\n"\
//     " [T]       Turbo mode\n"\
//     " [Z,X,C,V] Center CRT Screen\n"\
//     " [O]       Input poke\n"\
//     " [N]       NMI\n"\
//     " [P]       Pause\n"\
//     " [S]       BMP capture (folder /.c)\n"

// #define OSD_HELP_ES_ZX \
//     " Presione CAPS SHIFT + SYMBOL SHIFT y:\n"\
//     " [1]       Men\xA3 principal\n"\
//     " [NLANGS]       Cargar (SNA,Z80,P)\n"\
//     " [3-4]     Cargar / Guardar snapshot\n"\
//     " [5]       Elegir archivo de cinta\n"\
//     " [6]       Play/Stop cinta\n"\
//     " [7]       Explorador cinta\n"\
//     " [8]       Status CPU / Carga cinta\n"\
//     " [9-0]     Bajar-Subir volumen\n"\
//     " [Q]       Reset completo\n"\
//     " [W]       Resetear ESP32\n"\
// 	" [R]       Reset a TR-DOS\n"\
// 	" [I]       Info hardware\n"\
//     " [T]       Modo turbo\n"\
//     " [Z,X,C,V] Centrar pantalla CRT\n"\
//     " [O]       Introducir poke\n"\
//     " [N]       NMI\n"\
//     " [P]       Pausa\n"\
//     " [S]       Captura BMP (Carpeta /.c)\n"

static const char *StartMsg[NLANGS] = {
    "Hi! Thanks for choosing \nA1ESP\nF1eccy!\r"\
    "\r"\
    "\nA1ESP\nF1eccy is open source software\r"\
    "licensed under GPL v3. You can use,\r"\
    "modify, and share it for free.\r"\
    "\r"\
    "\r"\
    "\r"\
    "\r"\
    "This project is powered by \nA1ESP\nF1ectrum\r"\
    "(https://zxespectrum.speccy.org/)\r"
	,
	"\xAD" "Hola! \xADGracias por elegir \nA1ESP\nF1eccy!\r"\
	"\r"\
	"\nA1ESP\nF1eccy es software de c\xA2" "digo abier-\r"\
	"to bajo licencia GPL v3, puedes usarlo\r"\
	"modificarlo y compartirlo gratis.\r"\
	"\r"\
    "\r"\
    "\r"\
    "\r"\
	"Proyecto impulsado por \nA1ESP\nF1ectrum\r"\
    "(https://zxespectrum.speccy.org/)\r"
	,
	"Oi! Obrigado por escolher \nA1ESP\nF1eccy!\r"\
	"\r"\
	"\nA1ESP\nF1eccy \x82 software de c" "\xA2" "digo aberto\r"\
	"sob licen\x87" "a GPL v3, voc\x88 pode us\xa0-lo,\r"\
	"modific\xa0-lo e compartilh\xa0-lo gr\xa0tis.\r"\
	"\r"\
    "\r"\
    "\r"\
    "\r"\
	"Projeto impulsionado pelo ESPectrum\n"\
    "(https://zxespectrum.speccy.org/)\r"
};

static const char *STARTMSG_CLOSE[NLANGS] = { "This message will close in %02ds",
											  "Este mensaje se cerrar\xA0 en %02ds",
											  "Esta mensagem ser\xA0 fechada em %02ds"
											};

#include "images.h"

#endif // ESPECTRUM_MESSAGES_h
