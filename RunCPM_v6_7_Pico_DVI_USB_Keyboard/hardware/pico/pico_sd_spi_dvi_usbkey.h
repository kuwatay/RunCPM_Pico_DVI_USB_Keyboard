// SPDX-FileCopyrightText: 2023 Mockba the Borg
// SPDX-FileCopyrightText: 2023 Jeff Epler for Adafruit Industries
// SPDX-FileCopyrightText: 2024 Hisashi Kato
//
// SPDX-License-Identifier: MIT

#include <SdFat.h>
#include <PicoDVI.h>
#include <Adafruit_TinyUSB.h>
#include "../../console.h"
#include "../../arduino_hooks.h"

#include "keymapperUS.h"

#define USE_DISPLAY (1)
#define USE_KEYBOARD (1)


#ifndef USE_DISPLAY
#define USE_DISPLAY (0)
#endif

#ifndef USE_KEYBOARD
#define USE_KEYBOARD (0)
#endif

#ifndef USE_MSC
#define USE_MSC (0)
#endif


uint8_t getch_serial1(void) {
    while(true) {
        int r = Serial1.read();
        if(r != -1) {
            return r;
        }
    }
}

bool kbhit_serial1(void) {
    return Serial1.available();
}


#define USBH_KEY_BUFFER_SIZE 64

uint8_t usbhkbuf[USBH_KEY_BUFFER_SIZE];
uint8_t usbhkbufnum = 0;

bool usbhkbd_write(uint8_t code) {
    if (usbhkbufnum > USBH_KEY_BUFFER_SIZE) {
        return false;
    } else {
        usbhkbuf[usbhkbufnum] = code;
        usbhkbufnum++;
        return true;
    }
}

uint8_t usbhkbd_available(void) {
    if (usbhkbufnum == 0) return 0;
    else return usbhkbufnum;
}

int usbhkbd_read(void) {
    if (usbhkbufnum == 0) {
        return (-1);
    } else {
        usbhkbufnum-- ;
        uint8_t code = usbhkbuf[usbhkbufnum];
        return code;
    }
}

uint8_t getch_usbh(void) {
    while(true) {
        int r = usbhkbd_read();
        if(r != -1) {
            return r;
        }
    }
}

bool kbhit_usbh(void) {
    return usbhkbd_available();
}



// USB Keyboard
#if USE_KEYBOARD

#ifndef USE_TINYUSB_HOST
#error This sketch requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

#define KBD_INT_TIME 100 // USB HOST processing interval us

static repeating_timer_t rtimer;

#define LANGUAGE_ID 0x0409 // Language ID: English
Adafruit_USBH_Host USBHost; // USB Host object

#define MAX_REPORT  4

static struct {  // Each HID instance can has multiple reports
  uint8_t report_count;
  tuh_hid_report_info_t report_info[MAX_REPORT];
}hid_info[CFG_TUH_HID];

static bool keyboard_mounted = false;
static uint8_t keyboard_dev_addr = 0;
static uint8_t keyboard_idx = 0;
static uint8_t keyboard_leds = 0;
static bool keyboard_leds_changed = false;

int old_ascii = -1;
uint32_t repeat_timeout;
// this matches Linux default of 500ms to first repeat, 1/20s thereafter
const uint32_t default_repeat_time = 50;
const uint32_t initial_repeat_time = 500;

void send_ascii(uint8_t code, uint32_t repeat_time=default_repeat_time) {
  old_ascii = code;
  repeat_timeout = millis() + repeat_time;
  usbhkbd_write(code);
}

void usb_host_task(void) {
  USBHost.task();
  uint32_t now = millis();
  uint32_t deadline = repeat_timeout - now;
  if (old_ascii >= 0 && deadline > INT32_MAX) {
    send_ascii(old_ascii);
    deadline = repeat_timeout - now;
  } else if (old_ascii < 0) {
    deadline = UINT32_MAX;
  }
  if (keyboard_leds_changed) {
    tuh_hid_set_report(keyboard_dev_addr, keyboard_idx, 0/*report_id*/, HID_REPORT_TYPE_OUTPUT, &keyboard_leds, sizeof(keyboard_leds));
  }
}

bool timer_callback(repeating_timer_t *rtimer) { // USB Host is executed by timer interrupt.
  usb_host_task();
  return true;
}

#endif


/*
#define SPI_CLOCK (20'000'000)
#define SD_CS_PIN (17)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
DedicatedSpiCard blockdevice;
*/

//FatFileSystem SD;
// =========================================================================================
// Define SdFat as alias for SD
// =========================================================================================
SdFat32 SD;

// =========================================================================================
// Define Board-Data
// GP25 green onboard LED
// =========================================================================================
#define LED 25  // GPIO25
#define LEDinv 0
#define board_pico
#define board_analog_io
#define board_digital_io
#define BOARD "Raspberry Pi Pico"

// =========================================================================================
// Pin Documentation
// =========================================================================================
// Normal RPi Pico 
// MISO - Pin 21 - GPIO 16
// MOSI - Pin 25 - GPIO 19
// CS   - Pin 22 - GPIO 17
// SCK  - Pin 24 - GPIO 18
 
// MicroSD Pin Definition for RC2040 board
// Pin 4 - GPIO 2 Clock (SCK)
// Pin 5 - GPIO 3 MOSI
// Pin 6 - GPIO 4 MISO
// Pin 7 - GPIO 5 Chip/Card-Select (CS / SS)

// FUNCTIONS REQUIRED FOR USB MASS STORAGE ---------------------------------

#if USE_MSC
Adafruit_USBD_MSC usb_msc; // USB mass storage object
static bool msc_changed = true; // Is set true on filesystem changes

// Callback on READ10 command.
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  return blockdevice.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 command.
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  digitalWrite(LED_BUILTIN, HIGH);
  return blockdevice.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 completion.
void msc_flush_cb(void) {
  blockdevice.syncBlocks();   // Sync with blockdevice
  SD.cacheClear(); // Clear filesystem cache to force refresh
  digitalWrite(LED_BUILTIN, LOW);
  msc_changed = true;
}
#endif

// =========================================================================================

//
// DVI Display
//  + VT100 support
//

#if USE_DISPLAY
//DVItext1 display(DVI_RES_640x240p60, pimoroni_demo_hdmi_cfg);
DVItext1 display(DVI_RES_640x240p60, pico_sock_cfg);
//DVItext1 display(DVI_RES_800x240p30, pimoroni_demo_hdmi_cfg);

// VT100 controle is based on the following..
//
// https://github.com/dhansel/TerminalUSB/blob/master/firmware/src/vt100.c
// vt100.c
//	vt100 decoder for the VT100 Terminal program
//	Copyright (C) 2014 Geoff Graham (projects@geoffg.net)
//	All rights reserved.
	  
#define VT100   1
#define VT52    2
#define BOTH    3

extern void putch_display(uint8_t ch);
extern void ClearScreen(void);
extern void normalCh(), invCh(), write_normalCh(), write_invCh();

#define VTBUF_SIZE  40
uint8_t vtbuf[VTBUF_SIZE + 1]; // buffer for chars waiting to be decoded
int vtcnt;		      // vtcnt of the number of chars in vtbuf

int arg[8], argc;		// arguments to a command

struct s_cmdtbl {		// structure of the command table
    char *name;			// the string
    int mode;			// 1 = ANSI only, 2 = VT52, 3 = both
    void (*fptr)(void);	// pointer to the function that will interpret that command
};
int CmdTblSize;
extern const struct s_cmdtbl cmdtbl[];

int mode;			// current mode.  can be VT100 or VT52

int AttribUL, AttribRV, AttribInvis; // attributes that can be turned on/off
int SaveX, SaveY, SaveUL, SaveRV, SaveInvis, SaveFontNbr; // saved attributes that can be restored

#define ShowCursor(x) ;

void VT100Putc(uint8_t c) {
    int cmd, i, j, partial;

    if(vtcnt >= VTBUF_SIZE) return;
    vtbuf[vtcnt++] = c;
    partial = false;

    while(vtcnt) {
      for(cmd = 0; cmd < CmdTblSize; cmd++) {
	if((cmdtbl[cmd].mode & mode) && *cmdtbl[cmd].name == *vtbuf) {   // check the mode then a quick check of the first char
	  arg[0] = argc = 0;
	  for(j = i = 1; cmdtbl[cmd].name[i] && j < vtcnt; i++, j++) {
	    if(cmdtbl[cmd].name[i] == '^') {
	      arg[argc++] = vtbuf[j] - 31;
	    } else if(cmdtbl[cmd].name[i] == '@') {
	      arg[argc] = 0;
	      while(isdigit(vtbuf[j]))
		arg[argc] = arg[argc] * 10 + (vtbuf[j++] - '0');
	      j--; // correct for the overshoot, so that the next loop looks at the next char
	      argc++;
	    } else if(cmdtbl[cmd].name[i] != vtbuf[j])
	      goto nextcmd;                           // compare failed, try the next command
	  }
	  if(cmdtbl[cmd].name[i] == 0) { // compare succeded, we have found the command
	    vtcnt = 0;		// clear all chars in the queue
	    cmdtbl[cmd].fptr();	// and execute the command
	    return;
	  } else
	    partial = true; // partial compare so set a flag to indicate this
	}
      nextcmd:
	continue;
      } //for 
      if(!partial) { // we have searched the table and have not even found a partial match
	putch_display(*vtbuf);	// so send the oldest char off
	memcpy(vtbuf, vtbuf + 1, vtcnt--); // and shift down the table
      } else 
	return; // have a partial match so keep the characters in the buffer
    }		// keep looping until the buffer is empty
}

void VideoPrintString(char *p) {
  while(*p)
    display.write(*p++);
}

// utility function to move the cursor
void CursorPosition(int x, int y) {
  //ShowCursor(false); // turn off the cursor to prevent it from getting confused

  display.setCursor(x, y);
  /* CursorX = (fontWidth * fontScale) * (x - 1); */
  /* if(CursorX < 0) CursorX = 0; */
  /* if(CursorX > HRes - (fontWidth * fontScale)) CursorX = HRes - (fontWidth * fontScale); */
  
  /* CursorY = (fontHeight * fontScale) * (y - 1); */
  /* if(CursorY < 0) CursorY = 0; */
  /* if(CursorY > VRes - (fontHeight * fontScale)) CursorY = VRes - (fontHeight * fontScale); */

  /* CharPosX = (CursorX / (fontWidth * fontScale)) + 1;				// update the horizontal character position */
  /* CharPosY = (CursorY / (fontHeight * fontScale)) + 1;			// update the horizontal character position */

}

// cursor up one or more lines
void cmd_CurUp(void) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  normalCh();
  if(argc == 0 || arg[0] == 0) arg[0] = 1;
  display.setCursor(x, y - arg[0]-1);
  invCh();
}

// cursor down one or more lines
void cmd_CurDown(void) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  normalCh();
  if(argc == 0 || arg[0] == 0) arg[0] = 1;
  display.setCursor(x, y + arg[0]-1);
  invCh();
}



// cursor left one or more chars
void cmd_CurLeft(void) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  normalCh();
  if(argc == 0 || arg[0] == 0) arg[0] = 1;
  display.setCursor(x - arg[0]-1, y);
  invCh();
}


// cursor right one or more chars
void cmd_CurRight(void) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  normalCh();
  if(argc == 0 || arg[0] == 0) arg[0] = 1;
  display.setCursor(x + arg[0]-1, y);
  invCh();
}

// cursor home
void cmd_CurHome(void) {
  normalCh();
  display.setCursor(0,0);// internal coordinate start 0..
  invCh();
}


// position cursor
void cmd_CurPosition(void) {
  normalCh();
  if(argc < 1 || arg[0] == 0) arg[0] = 1;
  if(argc < 2 || arg[1] == 0) arg[1] = 1;
  display.setCursor(arg[1]-1, arg[0]-1);  // note that the argument order is Y, X
  invCh();
}

// enter VT52 mode
void cmd_VT52mode(void) {
  mode = VT52;
}


// enter VT100 mode
void cmd_VT100mode(void) {
  mode = VT100;
}

// clear to end of line
void cmd_ClearEOL(void) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  //    ShowCursor(false); // turn off the cursor to prevent it from getting confused
  /* for(y = CursorY; y < CursorY + (fontHeight * fontScale); y++) */
  /*     for(x = CursorX; x < HRes; x++) */
  /*         plot(x, y, 0); */
}


// clear to end of screen
void cmd_ClearEOS(void) {
  int y, n;
  cmd_ClearEOL();
  //    y = CursorY + (fontHeight * fontScale);
  //    n = VBuf*(HBuf/8) - (y * (HBuf/8));
  //    if(y < VBuf) memset((char *)VideoBuf + (y * (HBuf/8)), 0, n);
}

// clear from the beginning of the line to the cursor
void cmd_ClearBOL(void) {
  int x, y;
  ShowCursor(false);                                              // turn off the cursor to prevent it from getting confused
  /* for(y = CursorY; y < CursorY + (fontHeight * fontScale); y++) */
  /*     for(x = 0; x < CursorX; x++) */
  /*         plot(x, y, 0); */
}


// clear from home to the cursor
void cmd_ClearBOS(void) {
  int x, y;
  cmd_ClearBOL();
  /* for(y = CursorY - 1; y >= 0; y--) */
  /*     for(x = 0; x < HRes; x++) */
  /*         plot(x, y, 0); */
}

// turn the cursor off
void cmd_CursorOff(void) {
  //CursorOff = true;
}


// turn the cursor on
void cmd_CursorOn(void) {
  //CursorOff = false;
}

// save the current attributes
void cmd_ClearLine(void) {
  cmd_ClearBOL();
  cmd_ClearEOL();
}


// save the current attributes
void cmd_CurSave(void) {
  /* SaveX = CharPosX; */
  /* SaveY = CharPosY; */
  /* SaveUL = AttribUL; */
  /* SaveRV = AttribRV; */
  /* SaveInvis = AttribInvis; */
  /* SaveFontNbr = fontNbr; */
}

// restore the saved attributes
void cmd_CurRestore(void) {
  /* if(SaveFontNbr == -1) return; */
  /* CursorPosition(SaveX, SaveY); */
  /* AttribUL = SaveUL; */
  /* AttribRV = SaveRV; */
  /* AttribInvis = SaveInvis; */
  /* initFont(SaveFontNbr); */
}


// set the keyboard to numbers mode
void cmd_SetNumLock(void) {
  //NumLock = false;
  //setLEDs(CapsLock,  false, 0);
}

// set the keyboard to arrows mode
void cmd_ExitNumLock(void) {
  //NumLock = true;
  //setLEDs(CapsLock,  true, 0);
}


// respond as a VT52
void cmd_VT52ID(void) {
  //putSerialString("\033[/Z");
}

// respond as a VT100 thats OK
void cmd_VT100OK(void) {
  //putSerialString("\033[0n");
}


// respond as a VT100 with no optiond
void cmd_VT100ID(void) {
  //putSerialString("\033[?1;0c");    // vt100 with no options
}

// reset the terminal
void cmd_Reset(void) {
  mode = VT100;
  //    initFont(1);
  //ConfigBuffers(Option[O_LINES24]);
  //ShowCursor(false); // turn off the cursor to prevent it from getting confused
  ClearScreen();
  //CursorPosition(1, 1);
  //AttribUL = 0;
  //AttribRV = 0;
  //AttribInvis = 0;
  //SaveFontNbr = -1;
}


// control the keyboard leds
void cmd_LEDs(void) {
  /* static int led1 = 0; */
  /* static int led2 = 0; */
  /* static int led3 = 0; */

  /* if(arg[0] == 0) led1 = led2 = led3 = 0; */
  /* if(arg[0] == 2) led1 = 1; */
  /* if(arg[0] == 1) led2 = 1; */
  /* if(arg[0] == 3) led3 = 1; */
  /* setLEDs(led1, led2, led3); */
}

// set attributes
void cmd_Attributes(void) {
  /* ShowCursor(false); // turn off the cursor to prevent it from getting confused */
  /* if(arg[0] == 0) { */
  /*     AttribUL = AttribRV = AttribInvis = 0; */
  /*     initFont(1); */
  /* } */
  /* if(arg[0] == 4) AttribUL = 1; */
  /* if(arg[0] == 3) initFont(2); */
  /* if(arg[0] == 7) AttribRV = 1; */
  /* if(arg[0] == 6) initFont(3); */
  /* if(arg[0] == 8) AttribInvis = 1; */
}


// respond with the current cursor position
void cmd_ReportPosition(void) {
  //    char s[20];
  //    sprintf(s, "\033[%d;%dR", CharPosY, CharPosX);
  //    putSerialString(s);
}

// do a line feed
void cmd_Lf(void) {
  normalCh();
  display.write('\n');
  invCh();
}


// do a line feed
void cmd_LineFeed(void) {
  normalCh();
  display.write('\n');
  invCh();
}


// do an upwards line feed with a reverse scroll
void cmd_ReverseLineFeed(void) {
  //    ShowCursor(false); // turn off the cursor to prevent it from getting confused
  //if(CharPosY > 1)
  //CursorPosition(CharPosX, CharPosY - 1);
  //else
  //ScrollDown();
}


// turn on automatic line wrap, etc
void cmd_SetMode(void) {
  /* if(arg[0] == 7) AutoLineWrap = true; */
  /* if(arg[0] == 9 && vga) { */
  /*     cmd_CurHome(); */
  /*     ConfigBuffers(true); */
  /*     ShowCursor(false); // turn off the cursor to prevent it from getting confused */
  /*     ClearScreen(); */
  /*     CursorPosition(1, 1); */
  /* } */
}


// turn off automatic line wrap, etc
void cmd_ResetMode(void) {
  /* if(arg[0] == 7) AutoLineWrap = false; */
  /* if(arg[0] == 9 && vga) { */
  /*     ConfigBuffers(false); */
  /*     ShowCursor(false); // turn off the cursor to prevent it from getting confused */
  /*     ClearScreen(); */
  /*     CursorPosition(1, 1); */
  /* } */
}



// draw graphics : do nothing
void cmd_Draw(void) {
//    if(Display24Lines) {
//        arg[2] = (arg[2]/3)*2;
//        arg[4] = (arg[4]/3)*2;
//    }
    /* if(arg[0] == 1) DrawLine(arg[1], arg[2], arg[3], arg[4], 1); */
    /* if(arg[0] == 2) DrawBox(arg[1], arg[2], arg[3], arg[4], 0, 1); */
    /* if(arg[0] == 3) DrawBox(arg[1], arg[2], arg[3], arg[4], 1, 1); */
    /* if(arg[0] == 4) DrawCircle(arg[1], arg[2], arg[3], 0, 1, vga ? 1.14 : 1.0); */
    /* if(arg[0] == 5) DrawCircle(arg[1], arg[2], arg[3], 1, 1, vga ? 1.14 : 1.0); */
}


// do nothing for escape sequences that are not implemented
void cmd_NULL(void) {}

void ClearScreen() {
  display.fillScreen(0);
  display.setCursor(0, 0);
  invCh();			// set cursor
}

/***************************************************************************************************************************************
 The command table
 This is scanned from top to bottom by VT100Putc()
 Characters are matched exactly (ie, case sensitive) except for the @ character which is a wild card character representing
 zero or more decimal characters of an argument and ^ which represents a single char and the value returned is the char - 31
 **************************************************************************************************************************************/

const struct s_cmdtbl cmdtbl[]  = {
  { "\033A",          VT52,       cmd_CurUp },
  { "\033B",          VT52,       cmd_CurDown },
  { "\033C",          VT52,       cmd_CurRight },
  { "\033D",          VT52,       cmd_CurLeft },
  { "\033H",          VT52,       cmd_CurHome },
  { "\033I",          VT52,       cmd_ReverseLineFeed },
  { "\033[^^",        VT52,       cmd_CurPosition },

  { "\033J",          VT52,       cmd_ClearEOS },
  { "\033K",          VT52,       cmd_ClearEOL },

  { "\033>",          VT52,       cmd_SetNumLock },
  { "\033=",          VT52,       cmd_ExitNumLock },

  { "\033[Z",         VT52,       cmd_VT52ID },
  { "\033<",          VT52,       cmd_VT100mode },
  { "\033F",          VT52,       cmd_NULL },
  { "\033G",          VT52,       cmd_NULL },
    
  { "\033[K",         VT100,      cmd_ClearEOL },
  { "\033[J",         VT100,      cmd_ClearEOS },
  { "\033[0K",        VT100,      cmd_ClearEOL },
  { "\033[1K",        VT100,      cmd_ClearBOL },
  { "\033[2K",        VT100,      cmd_ClearLine },
  { "\033[0J",        VT100,      cmd_ClearEOS },
  { "\033[1J",        VT100,      cmd_ClearBOS },
 
  { "\033[2J",        VT100,      ClearScreen },

  { "\033[?25l",      VT100,      cmd_CursorOff },
  { "\033[?25h",      VT100,      cmd_CursorOn },

  { "\033[@A",        VT100,      cmd_CurUp },
  { "\033[@B",        VT100,      cmd_CurDown },
  { "\033[@C",        VT100,      cmd_CurRight },
  { "\033[@D",        VT100,      cmd_CurLeft },
  { "\033[@;@H",      VT100,      cmd_CurPosition },
  { "\033[@;@f",      VT100,      cmd_CurPosition },
  { "\033[H",         VT100,      cmd_CurHome },
  { "\033[f",         VT100,      cmd_CurHome },
  { "\0337",          VT100,      cmd_CurSave },
  { "\0338",          VT100,      cmd_CurRestore },
  { "\0336n",         VT100,      cmd_ReportPosition },
    

  { "\033D",          VT100,      cmd_LineFeed },
  { "\033M",          VT100,      cmd_ReverseLineFeed },
  { "\033E",          VT100,      cmd_Lf },
  { "\033[?@h",       VT100,      cmd_SetMode },
  { "\033[?@l",       VT100,      cmd_ResetMode },

  { "\033[>",         VT100,      cmd_SetNumLock },
  { "\033[=",         VT100,      cmd_ExitNumLock },
    
  { "\033[Z@;@;@;@;@Z",VT100,     cmd_Draw },
  { "\033[Z@;@;@;@Z",  VT100,     cmd_Draw },

  { "\033[@m" ,       VT100,      cmd_Attributes },
  { "\033[@q" ,       VT100,      cmd_LEDs },
  { "\033c" ,         BOTH,       cmd_Reset },
  { "\033[c" ,        VT100,      cmd_VT100ID },
  { "\033[0c" ,       VT100,      cmd_VT100ID },
  { "\033[5n" ,       VT100,      cmd_VT100OK },
  { "\033[?2l",       VT100,      cmd_VT52mode },
};          

void initVT100(void) {
  mode = VT100;
  vtcnt = 0;
  CmdTblSize =  (sizeof(cmdtbl)/sizeof(struct s_cmdtbl));
  //SaveFontNbr = -1;
}

#define H_TAB 8
#define V_TAB 1

uint16_t underCursor = ' ';

void normalCh() {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  uint16_t *row = display.getBuffer() + y * display.width()+ x;
  *row &= 0x0ff; 		// turn off mask bit
}

void invCh() {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  uint16_t *row = display.getBuffer() + y * display.width()+ x;
  *row |= 0x0ff00; 		// turn on mask bit
}

void write_invCh(uint8_t ch) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  uint16_t *row = display.getBuffer() + y * display.width()+ x;
  *row = 0x0ff00 + ch;
  //display.setCursor(x+1, y);
}  

void write_normalCh(uint8_t ch) {
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  uint16_t *row = display.getBuffer() + y * display.width()+ x;
  *row =  ch;
  //display.setCursor(x+1, y);
}  


void putch_display(uint8_t ch) {
  normalCh();  // rewrite to normal charactor at current position
  auto x = display.getCursorX();
  auto y = display.getCursorY();
  if(((ch >= 0x20) && (ch <= 0x7E)) || ((ch >= 0x80) && (ch <= 0xFF))) { //ASCII Character
    //display.drawPixel(x, y, ch);
    write_normalCh(ch);
    display.setCursor(x+1, y); // move to next position
  } else {
    switch(ch) {
    case 0x08: //Backspace
      if(x > 0) {
	display.setCursor(--x, y);
	display.drawPixel(x, y, ' ');
      }
      break;
      
    case 0x09: //HTab
      if(x >= 0) {
	int n = x % H_TAB;
	for (int i = 0; i < (H_TAB - n); ++i) {
	  display.setCursor(++x, y);
	  display.drawPixel(x, y, ' ');
	}
      }
      break;

    case 0x0A: //LF
      display.write('\n');
      break;

    case 0x0B: //VTab
      for (int i = 0; i < V_TAB; ++i) {
	display.write('\r');
	display.write('\n');
      }
      y = display.getCursorY();
      for (int i = 0; i <= x; ++i) {
	display.setCursor(i, y);
	display.drawPixel(i, y, ' ');
      }
      break;

    case 0x0D: //CR
      display.write('\r');
      break;

    }
  }
  invCh();  // for cursor 
}

#endif // USE_DISPLAY

bool port_init_early() {
#if USE_DISPLAY
//vreg_set_voltage(VREG_VOLTAGE_1_20);
//delay(10);
  if (!display.begin()) {
    return false;
  }
  _putch_hook = VT100Putc;;
  initVT100();  // initialize state
  
#endif

#if USE_KEYBOARD
  USBHost.begin(0);
  // USB Host is executed by timer interrupt.
  add_repeating_timer_us( KBD_INT_TIME/*us*/, timer_callback, NULL, &rtimer );

  _getch_hook = getch_usbh;
  _kbhit_hook = kbhit_usbh;
#endif


  // USB mass storage / filesystem setup (do BEFORE Serial init)
/*
  if (!blockdevice.begin(SD_CONFIG)) { _puts("Failed to initialize SD card"); return false; }
#if USE_MSC
  // Set disk vendor id, product id and revision
  usb_msc.setID("Adafruit", "Internal Flash", "1.0");
  // Set disk size, block size is 512 regardless of blockdevice page size
  usb_msc.setCapacity(blockdevice.sectorCount(), 512);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setUnitReady(true); // MSC is ready for read/write
  if (!usb_msc.begin()) {
      _puts("Failed to initialize USB MSC"); return false;
  }
#endif
*/
  return true;
}

/*
bool port_flash_begin() {
  if (!SD.begin(&blockdevice, true, 1)) { // Start filesystem on the blockdevice
      _puts("!SD.begin()"); return false;
  }
  return true;
}
*/


#if defined(__cplusplus)
extern "C"
{
#endif


#if USE_KEYBOARD

hid_keyboard_report_t old_report;

bool report_contains(const hid_keyboard_report_t &report, uint8_t key) {
  for (int i = 0; i < 6; i++) {
    if (report.keycode[i] == key) return true;
  }
  return false;
}

void process_boot_kbd_report(uint8_t dev_addr, uint8_t idx, const hid_keyboard_report_t &report) {

  bool alt = report.modifier & 0x44;
  bool shift = report.modifier & 0x22;
  bool ctrl = report.modifier & 0x11;

  bool num = old_report.reserved & 1;
  bool caps = old_report.reserved & 2;

  uint8_t code = 0;

  if (report.keycode[0] == 1 && report.keycode[1] == 1) {
    // keyboard says it has exceeded max kro
    return;
  }

  // something was pressed or release, so cancel any key repeat
  old_ascii = -1;

  for (auto keycode : report.keycode) {
    if (keycode == 0) continue;
    if (report_contains(old_report, keycode)) continue;

    /* key is newly pressed */
    if (keycode == HID_KEY_NUM_LOCK) {
      num = !num;
#ifdef USE_JP_KEYBOARD
    } else if ((keycode == HID_KEY_CAPS_LOCK) && shift) {
#else
    } else if (keycode == HID_KEY_CAPS_LOCK) {
#endif
      caps = !caps;
    } else {
      for (const auto &mapper : keycode_to_ascii) {
        if (!(keycode >= mapper.first && keycode <= mapper.last))
          continue;
        if (mapper.flags & FLAG_SHIFT && !shift)
          continue;
        if (mapper.flags & FLAG_NUMLOCK && !num)
          continue;
        if (mapper.flags & FLAG_CTRL && !ctrl)
          continue;
        if (mapper.flags & FLAG_LUT) {
          code = lut[mapper.code][keycode - mapper.first];
        } else {
          code = keycode - mapper.first + mapper.code;
        }
        if (mapper.flags & FLAG_ALPHABETIC) {
          if (shift ^ caps) {
            code ^= ('a' ^ 'A');
          }
        }
        if (ctrl) code &= 0x1f;
        if (alt) code ^= 0x80;
        send_ascii(code, initial_repeat_time); // send code
        break;
      }
    }
  }

//uint8_t leds = (caps | (num << 1));
  keyboard_leds = (num | (caps << 1));
  if (keyboard_leds != old_report.reserved) {
    keyboard_leds_changed = true;
    // no worky
    //auto r = tuh_hid_set_report(dev_addr, idx/*idx*/, 0/*report_id*/, HID_REPORT_TYPE_OUTPUT/*report_type*/, &leds, sizeof(leds));
    //tuh_hid_set_report(dev_addr, idx/*idx*/, 0/*report_id*/, HID_REPORT_TYPE_OUTPUT/*report_type*/, &leds, sizeof(leds));
  } else {
    keyboard_leds_changed = false;
  }
  old_report = report;
  old_report.reserved = keyboard_leds;
}

/*
//--------------------------------------------------------------------+
// Generic Report
//--------------------------------------------------------------------+
void process_generic_report(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len)
{
  uint8_t const rpt_count = hid_info[idx].report_count;
  tuh_hid_report_info_t* rpt_info_arr = hid_info[idx].report_info;
  tuh_hid_report_info_t* rpt_info = NULL;

  if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0)
  {
    // Simple report without report ID as 1st byte
    rpt_info = &rpt_info_arr[0];
  }else
  {
    // Composite report, 1st byte is report ID, data starts from 2nd byte
    uint8_t const rpt_id = report[0];

    // Find report id in the array
    for(uint8_t i=0; i<rpt_count; i++)
    {
      if (rpt_id == rpt_info_arr[i].report_id )
      {
        rpt_info = &rpt_info_arr[i];
        break;
      }
    }

    report++;
    len--;
  }

  // For complete list of Usage Page & Usage checkout src/class/hid/hid.h. For examples:
  // - Keyboard                     : Desktop, Keyboard
  // - Mouse                        : Desktop, Mouse
  // - Gamepad                      : Desktop, Gamepad
  // - Consumer Control (Media Key) : Consumer, Consumer Control
  // - System Control (Power key)   : Desktop, System Control
  // - Generic (vendor)             : 0xFFxx, xx

  if ( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP )
  {
    switch (rpt_info->usage)
    {
      case HID_USAGE_DESKTOP_KEYBOARD:
        {
          // Assume keyboard follow boot report layout
          process_boot_kbd_report(dev_addr, idx, *(hid_keyboard_report_t const*) report );
        }
        break;

      case HID_USAGE_DESKTOP_MOUSE:
        {
          // Assume mouse follow boot report layout
        }
        break;

      default:
        break;
    }
  }
  else //Other than HID_USAGE_PAGE_DESKTOP
  {
  }
}
*/

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t dev_addr)
{
}

// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t dev_addr)
{
}

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* desc_report, uint16_t desc_len)
{
  // By default host stack will use activate boot protocol on supported interface.
  // Therefore for this simple example, we only need to parse generic report descriptor (with built-in parser)
  hid_info[idx].report_count = tuh_hid_parse_report_descriptor(hid_info[idx].report_info, MAX_REPORT, desc_report, desc_len);
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, idx);

  switch(itf_protocol){
    case HID_ITF_PROTOCOL_NONE: //HID_PROTOCOL_BOOT:NONE
      break;

    case HID_ITF_PROTOCOL_KEYBOARD: //HID_PROTOCOL_BOOT:KEYBOARD
      if (keyboard_mounted != true) {
        keyboard_dev_addr = dev_addr;
        keyboard_idx = idx;
        keyboard_mounted = true;
      }
      break;

    case HID_ITF_PROTOCOL_MOUSE: //HID_PROTOCOL_BOOT:MOUSE
      break;
  }

  if ( !tuh_hid_receive_report(dev_addr, idx) )
  {
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx)
{
  if (dev_addr == keyboard_dev_addr && idx == keyboard_idx) {
    keyboard_mounted = false;
    keyboard_dev_addr = 0;
    keyboard_idx = 0;
    keyboard_leds = 0;
    old_report = {0};
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len)
{
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, idx);

  switch (itf_protocol)
  {
    case HID_ITF_PROTOCOL_KEYBOARD:
      if (keyboard_mounted == true) {
        process_boot_kbd_report(dev_addr, idx, *(hid_keyboard_report_t const*) report );
      }
      break;

    case HID_ITF_PROTOCOL_MOUSE:
      break;

    default:
      //process_generic_report(dev_addr, idx, report, len);
      break;
  }
  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, idx) )
  {
  }
}

#endif

#if defined(__cplusplus)
}
#endif
