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


#ifndef ESPECCY_MESSAGES_PT_h
#define ESPECCY_MESSAGES_PT_h

#define ERR_FS_EXT_FAIL_PT "\xAD" "Armazenamento externo n\x84o dispon\xA1vel!"

#define OSD_MSGDIALOG_YES_PT " Sim  "
#define OSD_MSGDIALOG_NO_PT " N\x84o  "

#define OSD_PAUSE_PT "--=[PAUSADO]=--"

#define POKE_ERR_ADDR1_PT "O endere\x87o deve estar entre 16384 e 65535"
#define POKE_ERR_ADDR2_PT "O endere\x87o deve ser inferior a 16384"
#define POKE_ERR_VALUE_PT "O valor deve ser inferior a 256"

#define RESET_REQUIERED_PT "Rein\xA1" "cio necess\xA0rio. Reinicie agora?"

#define OSD_INVALIDCHAR_PT "Caractere inv\xA0lido"

#define ERR_INVALID_OPERATION_PT "Opera\x87\x84o inv\xA0lida"

#define OSD_TAPE_SAVE_PT "Comando SAVE"

#define OSD_TAPE_SAVE_EXIST_PT "O arquivo j\xA0 existe. Substituir?"

#define OSD_PSNA_SAVE_PT "Salvar snapshot"

#define OSD_PSNA_EXISTS_PT "Substituir slot?"

#define OSD_TAPE_SELECT_ERR_PT "Arquivo de fita n\x84o selecionado"

#define OSD_TAPE_PLAY_PT "--- Reproduzir fita ---"

#define OSD_TAPE_STOP_PT "--- Deter fita ---"

#define OSD_FILE_INDEXING_PT "Indexa\x87\x84o"

#define OSD_FILE_INDEXING_PT_1 "   Ordena\x87\x84o   "

#define OSD_FILE_INDEXING_PT_2 "Salvando indice"

#define OSD_FILE_INDEXING_PT_3 "   Limpando    "

#define OSD_FIRMW_UPDATE_PT "Atualizar firmware "

#define OSD_DLG_SURE_PT "Quer continuar?"

#define OSD_DLG_JOYSAVE_PT "Salvar altera\x87\x94" "es?"

#define OSD_DLG_JOYDISCARD_PT "Descartar altera\x87\x94" "es?"

#define OSD_DLG_SETJOYMAPDEFAULTS_PT "Carregar mapeamento padr\x84o?"

#define OSD_FIRMW_PT "Atualizando firmware"

#define OSD_FIRMW_BEGIN_PT "Apagando parti\x87\x84o de destino."

#define OSD_FIRMW_WRITE_PT "   Gravando novo firmware.   "

#define OSD_FIRMW_END_PT   "Processo feito. Reiniciando. "

#define OSD_NOFIRMW_ERR_PT "Firmware n\x84o encontrado."

#define OSD_FIRMW_ERR_PT "Erro de atualiza\x87\x84o."

#define OSD_ROM_ERR_PT "Erro de grava\x87\x84o."

#define OSD_NOROMFILE_ERR_PT "Custom ROM n\x84o encontrada."

#define OSD_FLASH_BEGIN_PT "     Preparando espa\x87o.     "

#define OSD_ROM_PT "Gravar ROM Custom"

#define OSD_ROM_WRITE_PT "    Gravando ROM custom.    "

#define OSD_KBD_LAYOUT_PT "Gravar Custom Layout"

#define OSD_KBD_LAYOUT_WRITE_PT "   Gravando custom layout.  "

#define MENU_UPG_TITLE_PT "Escolha firmware"

#define MENU_ROM_TITLE_PT "Escolha ROM"

#define MENU_FILE_OPEN_TITLE_PT "Escolha o arquivo"

#define MENU_SAVE_SNA_TITLE_PT "Salvar snapshot"

#define MENU_TAP_TITLE_PT "Escolha o Fita"

#define MENU_DSK_TITLE_PT "Escolha o disco"

#define MENU_CHT_TITLE_PT "Escolha o arq. de cheats"

#define MENU_SCR_TITLE_PT "Escolha o arq. .SCR"

#define MENU_DELETE_TAP_BLOCKS_PT "Excluir sele\x87\x84o"

#define MENU_DELETE_CURRENT_TAP_BLOCK_PT "Excluir bloco"

#define OSD_BLOCK_SELECT_ERR_PT "Nenhum bloco selecionado"

#define OSD_BLOCK_TYPE_ERR_PT "Tipo de bloco inv\xA0lido"

#define MENU_DELETE_CURRENT_FILE_PT "Excluir arquivo"

#define OSD_READONLY_FILE_WARN_PT "Arquivo de somente leitura"

#define OSD_TAPE_FLASHLOAD_PT "Carregamento rapido de fita"
#define OSD_TAPE_INSERT_PT "Fita inserida"
#define OSD_TAPE_EJECT_PT "Fita ejetada"

#define OSD_ROM_INSERTED_PT "Cartucho inserido"
#define OSD_ROM_EJECT_PT "Cartucho ejetado"
#define OSD_ROM_INSERT_ERR_PT "Cartucho n\x84o inserido"

#define MENU_DELETE_SNA_PT "Escolha snapshot"

#define TRDOS_RESET_ERR_PT "TR-DOS n\x84o presente. Ative o Betadisk."

#define OSD_DISK_INSERTED_PT "Disco inserido"
#define OSD_DISK_EJECTED_PT "Disco ejetado"
#define OSD_DISK_ERR_PT "Disco inv\xA0lido"

#define MENU_STATE_PT \
    "Estados\n"\
    "Carregar\t>\n"\
    "Salvar\t>\n"

#define MENU_REALTAPE_PT "Carga de fita real\n"\
    "Auto\t[0]\n"\
    "For\x87ar LOAD\t[1]\n"\
    "For\x87ar SAVE\t[2]\n"

#define MENU_BETADISK_PT \
    "Escolha drive\n"\
    "Drive A\n"\
    "Drive B\n"\
    "Drive C\n"\
    "Drive D\n"

#define MENU_FILE_PT \
    "Arquivo\n"\
    "Navegador arquivo\t>\n"\
    "Navegador fita\t>\n"\
    "Ejetar\t>\n"

#define MENU_FILE_CLOSE_PT \
    "Ejetar\n"\
    "Fita\n"\
    "Disco\t>\n"\
    "Cartucho\n"

#define MENU_MAIN_PT \
    "Arquivo\t>\n"\
    "Estados\t>\n"\
    "Hardware\t>\n"\
    "Reiniciar\t>\n"\
    "Op\x87\x94" "es\t>\n"\
    "Atualizar\t>\n"\
    "Ajuda do teclado\t \n"\
    "Sobre\n"

#define MENU_OPTIONS_PT \
    "Op\x87\x94" "es\n"\
    "Armazenamento\t>\n"\
    "Hardware favorito\t>\n"\
    "ROM favorita\t>\n"\
    "Joystick\t>\n"\
    "Emula\x87\x84o do joystick\t>\n"\
    "Tela\t>\n"\
    "Mouse\t>\n"\
    "Som\t>\n"\
    "Outros\t>\n"\
    "Idioma\t>\n"\
    "Interface do Usu\xA0rio\t>\n"\
    "EAR config\t>\n"

#define MENU_UPDATE_PT \
    "Atualizar\n"\
	"Firmware\n"\
	"ROM Custom 48K\n"\
	"ROM Custom 128k\n"\
    "ROM Custom +2A\n"\
	"ROM Custom TK\n"\
    "Custom KBD Layout\n"

#define MENU_VIDEO_PT \
    "Tela\n"\
    "Tipo de renderiza\x87\x84o\t>\n"\
	"Resolu\x87\x84o\t>\n"\
    "Scanlines\t>\n"

#define MENU_RENDER_PT \
    "Tipo renderiza\x87\x84o\n"\
    "Padr\x84o\t[S]\n"\
    "Efeito de neve\t[A]\n"

#define MENU_ASPECT_PT \
    "Resolu\x87\x84o\n"\
    "320x240 (4:3)\t[4]\n"\
    "360x200 (16:9)\t[1]\n"

#define MENU_RESET_PT \
    "Reiniciar\n"\
    "Reinicializa\x87\x84o suave\n"\
	"Reiniciar para TR-DOS \t(CF11)\n"\
    "Reinicializa\x87\x84o total\t(F11)\n"\
    "Reiniciar ESP32\t(F12)\n"

#define MENU_STATE_SAVE_PT \
    "Salvar estado\n"

#define MENU_STATE_LOAD_PT \
    "Carregar estado\n"

#define MENU_STORAGE_PT "Armazenamento\n"\
    "Betadisk\t>\n"\
    "Auto carregamento de fita\t>\n"\
    "Carregamento r\xA0pido\t>\n"\
    "Timings ROM R.G.\t>\n"\
    "Monitor LOAD\t>\n"\
    "Fita real\t>\n"

#define MENU_YESNO_PT "Sim\t[Y]\n"\
    "N\x84o\t[N]\n"

#define MENU_MOUSE_PT "Mouse\n"\
    "Taxa de amostragem\t>\n"\
    "Resolu\x87\x84o\t>\n"\
    "Escala\t>\n"

#define MENU_SOUND_PT "Som\n"\
    "AY em 48K\t>\n"\
    "Covox\t>\n"

#define MENU_OTHER_PT "Outros\n"\
    "Timing ULA\t>\n"\
    "48K Issue 2\t>\n"\
    "ULA TK\t>\n"\
    "Segundo disp. PS/2\t>\n"\
    "Custom KDB layout\t>\n"

#define MENU_KBD2NDPS2_PT "Dispositivo\n"\
    "Nada\t[N]\n"\
    "Teclado / Adapt. DB9\t[K]\n"\
    "Mouse\t[M]\n"

#define MENU_ALUTIMING_PT "Timing ULA\n"\
    "Early\t[E]\n"\
    "Late\t[L]\n"

#define MENU_ARCH_PT "Escolha hardware\n"

#define MENU_ROMS48_PT "Escolha ROM\n"\
	"48K\n"\
    "48K Espanhol\n"\
    "Custom\n"

#define MENU_ROMS128_PT "Escolha ROM\n"\
	"128K\n"\
    "128K Espanhol\n"\
	"+2\n"\
    "+2 Espanhol\n"\
    "+2 Franc\x88s\n"\
    "ZX81+\n"\
    "Custom\n"

#define MENU_ROMS2A_3_PT "Escolha ROM\n"\
    "v4.0 +2a/+3\n"\
    "v4.0 +2a/+3 Espanhol\n"\
    "v4.1 +2a/+3\n"\
    "v4.1 +2a/+3 Espanhol\n"\
    "Custom\n"

#define MENU_ROMSTK_PT "Escolha ROM\n"\
    "v1 Espanhol\n"\
    "v1 Portugu\x88s\n"\
    "v2 Espanhol\n"\
    "v2 Portugu\x88s\n"\
    "v3 Espanhol (R.G.)\n"\
    "v3 Portugu\x88s (R.G.)\n"\
    "v3 Ingl\x88s (R.G.)\n"\
    "Custom\n"

#define MENU_ROMSTK95_PT "Escolha ROM\n"\
    "Espanhol\n"\
    "Portugu\x88s\n"

#define MENU_ROMS48_PREF_PT	"48K\t[48K]\n"\
    "48K Espanhol\t[48Kes]\n"\
    "Custom\t[48Kcs]\n"

#define MENU_ROMSTK90X_PREF_PT "v1 Espanhol\t[v1es ]\n"\
    "v1 Portugu\x88s\t[v1pt ]\n"\
    "v2 Espanhol\t[v2es ]\n"\
    "v2 Portugu\x88s\t[v2pt ]\n"\
    "v3 Espanhol (R.G.)\t[v3es ]\n"\
    "v3 Portugu\x88s (R.G.)\t[v3pt ]\n"\
    "v3 Ingl\x88s (R.G.)\t[v3en ]\n"\
    "Custom\t[TKcs ]\n"

#define MENU_ROMSTK95_PREF_PT "Espanhol\t[95es ]\n"\
    "Portugu\x88s\t[95pt ]\n"

#define MENU_ROMS128_PREF_PT "128K\t[128K]\n"\
    "128K Espanhol\t[128Kes]\n"\
	"+2\t[+2]\n"\
    "+2 Espanhol\t[+2es]\n"\
    "+2 Franc\x88s\t[+2fr]\n"\
    "ZX81+\t[ZX81+]\n"\
    "Custom\t[128Kcs]\n"

#define MENU_ROMS2A_3_PREF_PT "v4.0 +2a/+3\t[+2A]\n"\
    "v4.0 +2a/+3 Espanhol\t[+2Aes]\n"\
    "v4.1 +2a/+3\t[+2A41]\n"\
    "v4.1 +2a/+3 Espanhol\t[+2A41es]\n"\
    "Custom\t[+2Acs]\n"

#define MENU_INTERFACE_LANG_PT "Idioma\n"\
    "Ingl\x88s\t[E]\n"\
    "Espanhol\t[S]\n"\
	"Portugu\x88s\t[P]\n"

#define MENU_JOY_PT "Joystick\n"

#define MENU_DEFJOY_PT "Definir\n"

#define MENU_JOYPS2_PT "Emula\x87\x84o Joystick\n"\
    "Tipo joystick\t>\n"\
    "Joy nas teclas do cursor\t>\n"\
    "TAB como fire 1\t>\n"

#define DLG_TITLE_INPUTPOK_PT "Adicionar Poke"

#define MENU_UI_PT "Interface do Usu\xA0rio\n"\
    "Menu com setas\t>\n"\
    "Rota\x87\x84o do texto\t>\n"\
    "Miniaturas\t>\n"\
    "Tela instant\x83nea\t>\n"

#endif // ESPECCY_MESSAGES_PT_h
