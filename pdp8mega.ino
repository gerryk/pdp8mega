/* PDP8 Mega
 * PDP8 Emulator for Arduino Mega 2560
 * Based on original emulator code by Doug Jones, dept. of Comp Sci. UIowa.
 * with modifications, additions and deletions by Ricardo Guerreiro
 * and Gerry Kavanagh
 */

/*************************************************************************/

/* File: bus.h
   Author: Douglas Jones, Dept. of Comp. Sci., U. of Iowa, Iowa City, IA 52242.
   Date: July 26, 1995
   Language: C (UNIX)
   Purpose: Declarations of bus lines shared by PDP8/E and peripherals.
            This is not, strictly speaking, either an omnibus or a positive
            I/O bus, but rather, it is a set of declarations driven by the
	    needs of system emulation.
   Constraints: When included in the main program, MAIN must be defined.
            When included elsewhere, MAIN must not be defined.

   Based on the description in the PDP-8/E Small Computer Handbook,
   Digital Equipment Corporation, 1971.
*/

/***************************************************************
   August 2013
	heavily modified to be used un arduino mega 2560
	Ricardo Das Neves Guerreiro
**************************************************************/
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*               Includes needed by arduino                                */
#include <LiquidCrystal.h>        //lcd library
#include <Wire.h>                 //I2C library
/*              creates lcd instance                                        */
LiquidCrystal lcd_R(6,7,8,9,10,11,12);
byte i8;                            //because my lcd is only 40 wide
#define MAXMEM 4096
// de kk8e
int enab_del; /* secondary enable flipflop, used to delay enable operations */
long int c2;  // to debug
int slow_Button = 15;
// changed to 'true' for test (gerryk)
boolean slow = true;


/* Author = Ricardo Das Neves Guerreiro

            to read focal from eeprom */

// LED TO GENERAL USE
// Was 4... changed to 13 for test (gerryk)
int led_slow = 13;
byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress )
{
    pinMode(led_slow, OUTPUT);
    digitalWrite(led_slow, HIGH);
    byte rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
    digitalWrite(led_slow, LOW);
}

#define STARTADDR ((uint8_t *)0x2200)

void write_SR(byte data,word address)
{
    uint8_t *addr;
    addr= STARTADDR+address;
    *addr = data;
}

byte read_SR(word address)
{
    byte r_data;
    uint8_t *addr;
    addr= STARTADDR+address;
    r_data = *addr;
    return r_data;
}

int from_memory(int rmem)
{
    byte lw,hw;
    if((rmem>=0) && (rmem<=MAXMEM))  {
        hw = read_SR((rmem*2));
        lw = read_SR((rmem*2)+1);
        return word(hw,lw);
    }
}

// here is something wrong
void to_memory(int rval, int rmem)
{
    byte lw,hw;
    if((rmem>=0) &&  (rmem<=MAXMEM))  {
        lw = rmem;
        hw = rmem >> 8;
        write_SR(hw,(word)(rval*2));
        write_SR(lw,(word)(rval*2)+1);
    }  else {
        // noop
    }
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void leeprom(void)
{
    word i;
    byte data1,data2;
    boolean ok;
    lcd_R.setCursor(0,0);
    lcd_R.print("leo eeprom  8192            ");
    Serial.println("Loading EEPROM 8K...");

    lcd_R.setCursor(0,1);
    lcd_R.print("                                     ");
    for (i=0; i<=8191; ++i) {
        data1 = i2c_eeprom_read_byte(0x50, i);      //take from external eeprom
        write_SR(data1,i);                          // into external sram
    }
    lcd_R.setCursor(0,1);
    lcd_R.print("verifico 1      ");
    Serial.println("Verifying...");
    ok = true;                                      // to verify
    for (i=0; i<=8191; ++i) {
        data1 = i2c_eeprom_read_byte(0x50, i);
        data2 = read_SR(i);
        if (data1 != data2) ok = false;
    }
    lcd_R.setCursor(0,1);
    if (ok == true) {
        Serial.println("EPROM & SRAM OK...");
        lcd_R.print("eeprom y Sram OK                  ");
    } else {
        Serial.println("EPROM & SRAM ERROR...");
        lcd_R.print("eeprom y Sram ERROR               ");
    };
    delay(1000);
    lcd_R.setCursor(0,0);
    lcd_R.print("prog cargado              ");
}// fin leeprom

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void iniRAM(void)
{
    /* Enable XMEM interface:
    SRE    (7)   : Set to 1 to enable XMEM interface
    SRL2-0 (6-4) : Set to 00x for wait state sector config: Low=N/A, High=0x2200-0xFFFF
    SRW11:0 (3-2) : Set to 00 for no wait states in upper sector
    SRW01:0 (1-0) : Set to 00 for no wait states in lower sector
    */
    Serial.println("Init RAM interface...");

    XMCRA = 0x80;
    // ver de que se trata
    // Bus keeper, lower 7 bits of Port C used for upper address bits, bit 7 is under PIO control
    XMCRB = _BV(XMBK) | _BV(XMM0);
    DDRC |= _BV(7);
    PORTC = 0x7F; // Enable pullups on port C, select bank 0
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* File: bus.h
   Author: Douglas Jones, Dept. of Comp. Sci., U. of Iowa, Iowa City, IA 52242.
   Date: July 26, 1995
   Language: C (UNIX)
   Purpose: Declarations of bus lines shared by PDP8/E and peripherals.
            This is not, strictly speaking, either an omnibus or a positive
            I/O bus, but rather, it is a set of declarations driven by the
	    needs of system emulation.
   Constraints: When included in the main program, MAIN must be defined.
            When included elsewhere, MAIN must not be defined.
   Based on the description in the PDP-8/E Small Computer Handbook,
   Digital Equipment Corporation, 1971.
*/
/* The following trick puts extern on definitions if not in the main program
*/

/**********/
/* Memory */
/**********/

/* This emulator does not allow for non-contiguous memory fields.
   Checking of memory addressing errors is not done, so all addressable
   memory must be available.  Thus, the memory size depends on whether
   the KM8E option is present.
*/

/*******************************/
/* Generally visible registers */
/*******************************/

/* All of the following are visible outside the CPU in some context or
   another, either to some I/O device or to the front panel.
*/

int ac;  /* the accumulator, 12 bits */
int pc;  /* the program counter, 12 bits */
int mq;  /* the multiplier quotient, 12 bits */
int sr;  /* the switch register */
int cpma;/* the memory address register */
int mb;  /* the memory buffer register */

int link;/* the link bit, 1 bit, in position 010000 of the word */
int run; /* the run flipflop, 0 = halt, 1 = running */

int enab;/* interrupt enable bit, 0 = disable, 1=enable */
int enab_rtf; /* secodary enable needed for RTF deferred enable */
int irq; /* the interrupt request line, 0 = no request, >0 = request */

int sw;  /* the switch, 1 bit */

long int countdown;

/* Note that any positive value of irq indicates a request!  Requests are
   posted by incrementing irq, and withdrawn by decrementing irq.
*/

/*****************************************************************************/

/* realtime.h

   Author: Douglas Jones, Dept. of Comp. Sci., U. of Iowa, Iowa City, IA 52242.
   Date: Nov. 13, 1995
   Language: C (UNIX)
   Purpose: interface to simulation core routines for real-time behavior
            of devices attached to instruction-set level computer emulators.
*/

/* The following trick puts extern on definitions if not in the main program */

/**********************************************/
/* Times are measure in 200 nanosecond ticks. */
/**********************************************/

#define microsecond 1l
#define millisecond 1000l
#define second 1000000l


/**************************************************************************/
/* Using 32 bit signed integers, the maximum time is 2**31 - 1 ticks,     */
/* the conversion to seconds is (ns/tick)(sec/ns)(ticks) = sec,           */
/* so, a 32 bit clock can hold (200)(10**-9)(2**31-1) sec = 7.158 min     */
/*                                                                        */
/* This is not much running time, so scheduled events are always recorded */
/* as delays from the present, and no peripheral activity may be set to   */
/* happen more than 7 minutes into the future (this is plenty long for    */
/* such things as rewinding a reel of tape, one of the slowest actions.   */
/**************************************************************************/

/**************************************************************************/
/* For each device for which there may be asynchronous activity, a timer  */
/* needs to be allocated.  This timer is set whenever the activity is     */
/* caused, and when the timer expires, it calls the appropriate service   */
/* to simulate the activity in question                                   */
/**************************************************************************/

struct timer {
    long int delay;      /* the delay until the timer is to fire */
    void (* action)();   /* the function to call when the timer fires */
    int param;           /* parameter to action */
    struct timer * next; /* the next timer to worry about after this */
};

/* each timer must be initialized before the first time it is scheduled */
#define init_timer(t) t.delay = -1

/**************************************************************************/
/* It is up to the emulator to decrement countdown appropriately as each  */
/* machine instruction is interpreted, thus recording the passage of time */
/* The emulator is also responsible for calling fire_timer if countdown   */
/* ever reaches zero.                                                     */
/**************************************************************************/


/*****************************************************************************/

/* File: realtime.c

   Author: Douglas Jones, Dept. of Comp. Sci., U. of Iowa, Iowa City, IA 52242.
   Date: July 26, 1995
   Language: C (UNIX)
   Purpose: simulation core routines for real-time behavior of devices
            attached to instruction-set level computer emulators.
*/

/************************************************/
/* Read the comments in the include file first! */
/************************************************/

/**************************************************************************/
/* Timers that have been scheduled are stored in a linked list, ordered   */
/* by their delay from the present.  The delay in each timer record after */
/* the first is the delay between the preceeding event and that event.    */
/* The delay of the first scheduled timer is not recorded in that timer,  */
/* but is stored in countdown, which is periodically decremented.         */
/**************************************************************************/

#define niltimer (struct timer *)0

static struct timer * head; /* points to the next timer in the queue */

void init_timers()  /* initialize timer subsystem */
{
    head = niltimer; /* there is no pending timer initially */
    countdown = 0x7FFFFFFF; /* so put off next firing a long time */
}

void schedule(struct timer * t,long int d,void (* a)())  /* schedule timer t after a delay of d ticks */
{
    t->action = a;
    if (t->delay >= 0) { /* timer is already scheduled */
        /* don't mess up the queue, schedule becomes no-op */
    } else if (head == niltimer) { /* nothing is scheduled yet */
        countdown = d;
        t->delay = d;
        t->next = niltimer;
        head = t;
    } else { /* there is an existing schedule */
        if (countdown > d) {    /* t goes before old head */
            head->delay = countdown - d;
            countdown = d;    /* set timer until t */
            t->next = head;   /* link t ahead of old head */
            t->delay = d;     /* mark timer as queued */
            head = t;	  /* reset list head */
        } else { /* new event goes down into queue */
            struct timer * i;
            i = head;
            d = d - countdown; /* delay relative to head */
            while (i->next != niltimer) { /* scan list for place */
                if (d < i->next->delay) {
                    i->next->delay = i->next->delay - d;
                    break;
                }
                i = i->next;
                d = d - i->delay; /* adjust delay */
            }
            t->next = i->next; /* link new timer into queue */
            t->delay = d;
            i->next = t;
        }
    }
}

long int query_timer(struct timer * t)  /* find how much time remains on t */
{
    if (t->delay < 0) { /* timer is not in queue */
        return -1;
    } else {
        long int query = countdown;
        struct timer * i = head;
        while (i != t) { /* scan list for t */
            i = i->next;
            query = query + i->delay; /* accumulate delays */
        }
        return query;
    }
}


void fire_timer()  /* cause timer to fire at current time */
{
    if (head == niltimer) { /* no pending timer */
        /* put off next firing as long as possible */
        countdown = 0x7FFFFFFF;
    } else {
        void (* a)();
        /* save action for use after dequeue */
        a = head->action;
        /* mark head timer as idle */
        head->delay = -1;
        /* schedule next event */
        head = head->next;
        if (head == niltimer) { /* no new timer */
            /* put off next firing as long as possible */
            countdown = 0x7FFFFFFF;
        } else {
            /* figure delay till next timer */
            countdown = countdown + head->delay;
            /* note: this could have been countdown = head->delay,
               except that countdown could have gone slightly
               negative, and we want to make delays add nicely
               for such devices as line frequency clocks */
        }
        /* fire action on head timer */
        (* a)();
    }
}

/*****************************************************************************/

/* File: kl8e.c

   Author: Douglas Jones, Dept. of Comp. Sci., U. of Iowa, Iowa City, IA 52242.
   Date: Feb. 29, 1996
   Language: C (UNIX)
   Purpose: DEC KL8/E console teletype emulator, for the console device only!
            Emulators for other asynchronous interfaces must be constructed
            separately.

   Based on the description in the PDP-8/E Small Computer Handbook,
   Digital Equipment Corporation, 1971.

   Uses nonblocking polling to grab characters from the keyboard.  This is
   a bit too synchronous a style of reading, but until ttyaccess.c can be
   made to use interrupts, it will do.  As a result, this should look to
   software as if it is reading from a synchronous data line interface
   that automatically discards pad characters.  Users who type too fast
   will note that their input-overruns lead to lost data.
*/

/*********************************************************************/
/* options:  The user may change the speed of the simulated teletype */
/* to any positive value.  DEC sold the KL8E in the following        */
/* standard versions with an RS232 interface:                        */
/*                                                                   */
/*   KL8E option:           A     B     C     D     E     F     G    */
/*   M8650 version:         -     YA    YA    YA    YA    YA    YA   */
/*   Transmit baud rate:   110   150   300   600  1200  1200  2400   */
/*   Receive baud rate:    110   150   300   600  1200   150   150   */
/*   Receive baud rate:    110   150   300   600  1200   150   150   */
/*                                                                   */
/* In fact, the M8650 YA board could be jumpered to handle 2400,     */
/* 4800 and 9600 baud on both input and output, but DEC didn't tell  */
/* people about this, at least not in the manual.                    */
/*********************************************************************/

/* translations from baud rate to time per character */

#define baud110  (100    * millisecond)
#define baud150  ( 66667 * microsecond)
#define baud300  ( 33333 * microsecond)
#define baud600  ( 16667 * microsecond)
#define baud1200 (  8333 * microsecond)
#define baud2400 (  4166 * microsecond)
#define baud4800 (  2083 * microsecond)
#define baud9600 (  1041 * microsecond)

/* select the baud rate here (note: if the emulator runs at 1/10 the speed
   of the real PDP-8, 1200 baud simulation will look like 110 to a person! */

#define IOFUDGE 1
#define print_time (baud9600 / IOFUDGE)
#define read_time (baud9600 / IOFUDGE)

/****************************************************************/
/* restrictions:  This version of the console has no low-speed  */
/* paper-tape reader or punch -- call it a KSR teletype instead */
/* of the usual ASR teletype.                                   */
/****************************************************************/

/*********************************************************/
/* Interface between device implementation and "console" */
/*********************************************************/

/* timers used to simulate delays between I/O initiation and completion */
/*
	struct timer {
		long int delay;       the delay until the timer is to fire
		void (* action)();    the function to call when the timer fires
		int param;            parameter to action
		struct timer * next;  the next timer to worry about after this
	};
*/

static struct timer print_delay;
static struct timer read_delay;

/*************************************/
/* "officially visible" device state */
/*************************************/

static int keyboard_flag;
static int keyboard_buffer;
static int interrupt_enable;
static int print_flag;
static int print_buffer;

/*************************/
/* Device Implementation */
/*************************/

void ttyputc(char ch) /* put character to console */
{
    Serial.write(ch);
}

static void keyboard_event() /* called to poll for keyboard input */
{
    int poll;
    if (Serial.available() > 0) {
        keyboard_buffer = ( Serial.read() | 0200 );
        /* my debug code below... gerryk */
        ttyputc(keyboard_buffer);  // echo
        /* report the keypress to the rest of the emulator */
        /*RIC ver como keyboard_flag puede ser distinto a 1 */
        if (keyboard_flag != 1) { /* no overrun condition */
            keyboard_flag = 1;
            if (interrupt_enable == 1) {
                irq = irq + 1;
            }
        }
    }
    /*RIC regenera la llamada */
    schedule( &read_delay, read_time, keyboard_event );
}

static void print_event()
{
    /* called from timer when a byte has been successfully printed */
    /* this code allows for the DEC convention of setting the high bit */
    ttyputc( print_buffer & 0177 );
    print_flag = 1;
    if (interrupt_enable == 1) {
        irq = irq + 1;
    }
}

static void print_character()
{
    /* schedule the completion of a print "print_time" in the future */
    schedule( &print_delay, print_time, print_event);
}

/******************/
/* Initialization */
/******************/

void kl8epower() /* global initialize */
{
    /* set up timers used to delay I/O activity */
    init_timer(print_delay);
    init_timer(read_delay);
    /* the following makes the reader run forever (probably wrong) */
    schedule( &read_delay, read_time, keyboard_event );
}

void kl8einit() /* console reset */
{
    keyboard_flag = 0;
    print_flag = 0;
    interrupt_enable = 1;
}



/********************/
/* IOT Instructions */
/********************/

void kl8edev3(int op) /* keyboard*/
{
    switch (op) {
        /* (pdp8e) keyboard flag is cleared */
    case 00: /* KCF */
        if (interrupt_enable == 1) {
            irq = irq - keyboard_flag;
        }
        keyboard_flag = 0;
        break;
        /* KSF skip next instruction when keyboard buffer register is loaded
           with an ASCII symbol (causing keyboard flag raised) */
    case 01: /* KSF */
        if (keyboard_flag == 1) {
            pc = (pc + 1) & 07777;
        }
        break;
        /* clear AC, clear keyboard flag */
    case 02: /* KCC */
        if (interrupt_enable == 1) {
            irq = irq - keyboard_flag;
        }
        keyboard_flag = 0;
        ac = 00000;
        break;
    case 03: /* no operation! */
        break;
        /* transfer the content of keyboard buffer to AC */
    case 04: /* KRS */
        ac = ac | keyboard_buffer;
        break;
        /* (pdp8e) acumulator loaded into device control register */
    case 05: /* KIE != KSF KRS */
        if ((ac & 00001) == 0) { /* disable interrupts */
            if (interrupt_enable == 1) {
                interrupt_enable = 0;
                irq = irq - (keyboard_flag + print_flag);
            }
        } else { /* enable interrupts */
            if (interrupt_enable == 0) {
                interrupt_enable = 1;
                irq = irq + (keyboard_flag + print_flag);
            }
        }
        break;
        /* transfer contents keyboard buffer to AC and clear keyboard flag*/
    case 06: /* KRB = KCC KRS */
        if (interrupt_enable == 1) {
            irq = irq - keyboard_flag;
        }
        keyboard_flag = 0;
        ac = keyboard_buffer;
        break;
    case 07: /* no operation! */
        break;
    }
}

void kl8edev4(int op) /*printer*/
{
    switch (op) {
        /* (pdp8e) printer flag, signaling output completed is set */
    case 00: /* TFL */
        if (print_flag == 0) {
            print_flag = 1;
            if (interrupt_enable == 1) {
                irq = irq + 1;
            }
        }
        break;
        /* skip next instruction if printer flag=1 */
    case 01: /* TSF */
        if (print_flag == 1) {
            pc = (pc + 1) & 07777;
        }
        break;
        /* clear printer flag */
    case 02: /* TCF */
        if (interrupt_enable == 1) {
            irq = irq - print_flag;
        }
        print_flag = 0;
        break;
    case 03: /* no operation! */
        break;
        /* load printer buffer with contents AC, select and print character
           flag is raised when action completed  */
    case 04: /* TPC */
        print_buffer = ac & 00377;
        print_character();
        break;
        /* (pdp8e) skip next instruction if if printer or keyboards flags are set */
    case 05: /* TSK != TSF TPC */
        if ((print_flag == 1) || (keyboard_flag == 1)) {
            pc = (pc + 1) & 07777;
        }
        break;
        /* clear printer flag, transfer contents AC to printer buffer register
           select and print character. flag is raised when action completed */
    case 06: /* TLS = TCF TPC */
        if (interrupt_enable == 1) {
            irq = irq - print_flag;
        }
        print_flag = 0;
        print_buffer = ac & 00377;
        print_character();
        break;
    case 07: /* no operation! */
        break;
    }
}

/*****************************************************************************/

/* Some parts of File: kk8e.c
   The ones that are not in the main loop

   Modify: Ricardo Guerreiro 2011. Cleaning unused definitions (debug ke8e etc)
           Preparing for pdp8r
	basado en kk8er que esta testeado que anda ok
	esta limpio para un straight
	lee focal de debug.txt
	no usa devices para nada
	ANDA SIN  front panel

   Author: Douglas Jones, Dept. of Comp. Sci., U. of Iowa, Iowa City, IA 52242.
   Date: Mar. 6, 1996
   Language: C (UNIX)
   Purpose: DEC PDP-8/e emulator
   Based on the PDP-8/E Small Computer Handbook,
   Digital Equipment Corporation, 1971, 1973,
   and the PDP-8/E/F/M Maintenance Manual, volumes 1 and 2,
   Digital Equipment Corporation, 1975.
*/

/******************/
/* Initialization */
/******************/

/* Both the reset key on the console and the CAF instruction call this */

void clearflags() {
    /* device specific effects of the reset operation */
    kl8einit(); /* console TTY */
    link = 000000;
    ac = 00000;
    irq = 0;
    enab = 0;
    enab_del = 0;
}


void powerup() {
/* called only once for power on */

/* first, see if there is a specified core image save file,
   since PDP-8/E machines usually have core memory and tend
   to remember what was in them as of the previous shutdown

   A parameter of the form <filename> with no leading dash
   is interpreted as the name of the core file.  If there is
   no core file specified, core comes up uninitialized.
*/

  run = 0; /* by default, the machine comes up halted */

  /* initialize the real-time simulation mechanisms */
  
  init_timers();

  /* initialize all devices to their power-on state */
  /* core must be registered first because it's going to be
   on the console device window at console-power up */

  kl8epower(); /* console TTY */
  clearflags();
}


void powerdown() {  /* called only once from the console to exit the emulator */
    exit(0);
}


/************************/
/* Instruction decoding */
/************************/

/*
/			       _____ _____ _____ _____
/  instruction word format: = |_|_|_|_|_|_|_|_|_|_|_|_|
/ 			      | op  |i|z|     adr     |
/
/  Instructions will be decoded using a 5 bit opcode-mode combination,
/  so the following opcode definitions are shifted 2 places left
*/

#define opAND (0 << 2)
#define opTAD (1 << 2)
#define opISZ (2 << 2)
#define opDCA (3 << 2)
#define opJMS (4 << 2)
#define opJMP (5 << 2)
#define opIOT (6 << 2)
#define opOPR (7 << 2)

/* The following definitions give the addressing modes as a 2 bit field
*/

#define DIRECT 0
#define DEFER 2
#define ZERO 0
#define CURRENT 1

/* The following definitions give instruction times in terms of 200 ns ticks
*/

#define shortcycle 6
#define longcycle 7

/* The following definitions give widely used code for addressing
*/

/*RIC ojo aca viene algo si mas memoria */

#define PAGE_ZERO cpma = (mb & 0177)

#define CURRENT_PAGE cpma = (mb & 0177) | (pc & 07600)

#define DEFER_CYCLE {                                      \
  if ((cpma & 07770) == 00010) {  /* indexed */            \
      to_memory(cpma,((from_memory(cpma) + 1) & 07777));   \
      cpma = from_memory(cpma);                            \
      countdown -= longcycle;                              \
  } else {  /* normal */                                   \
      cpma = from_memory(cpma);                            \
      countdown -= shortcycle;                             \
  }                                                        \
}                                                          \

/****************************************************************************/

/* Actual Arduino stuff follows...
 */

void setup()
{
    int c1;
    int data2;
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.println("Welcome to PDP8/e Emulator for ArduinoMEGA");
    pinMode(led_slow, OUTPUT);
    digitalWrite(led_slow, HIGH);
    Wire.begin();                 // initialise the connection
    lcd_R.print("Inicio               ");
    Serial.println("Setup Started...");
    iniRAM();
    leeprom();
    lcd_R.setCursor(0,0);
    /*
    left from debugging
    for (c1 = 0; c1 < 4096; c1++) {
       data2 = from_memory(c1);
           Serial.print(c1);
           Serial.print(" ");
           Serial.println(data2);
    }

    */
    powerup();
    run=1;
    pc=128;
    lcd_R.setCursor(0,0);
    lcd_R.print("Ejecutando     ");
    c2=0;
    digitalWrite(led_slow, LOW);
    // first row of leds
    DDRF = 0b11111111;    //All pins in PORTF are outputs
    DDRK = 0b11111111;    //All pins in PORTK are outputs
    DDRL = 0b11111111;    //All pins in PORTK are outputs
    pinMode(slow_Button, INPUT);
    Serial.println("Setup Complete...");
    Serial.flush();
}



void loop()
{
    /* to display leds */
    byte l_low1;
    byte l_high1;
    byte l_low2;
    byte l_high2;
    byte mix;
    int buttonState = digitalRead(slow_Button);
    if (buttonState == 1) {
        slow = true;
        digitalWrite(led_slow, HIGH);
    }
    if (slow) delay(10);
    l_low1 = lowByte(pc);
    l_high1 = highByte(pc);
    l_low2 = lowByte(ac);
    l_high2 = highByte(ac);
    mix = l_high1 & 0b00001111;
    l_high2 = l_high2 << 4;
    // is this neccessary?
    l_high2 = l_high2 & 0b11110000;
    mix = l_high2 | l_high1;
    PORTK = mix;
    PORTF = l_low1;
    PORTL = l_low2;
    /* setup to fetch from pc */
    
    /*  This block was commented out... uncommenting for interest  */
    /*
    Serial.print(c2-1);
    Serial.print(' ');
    Serial.print(pc,OCT);
    Serial.print(' ');
    Serial.print(ac,OCT);
    Serial.print(' ');
    Serial.print(countdown);
    Serial.print(' ');
    Serial.print(irq);
    Serial.print(' ');
    Serial.print(enab_del);
    Serial.print(' ');
    Serial.print(enab_rtf);
    Serial.print(' ');
    Serial.println(cpma,OCT);
    */
    /*   */
    cpma = pc;
    /* I/O and console activity happens with CPMA holding PC */
    while (countdown <= 0) { /* handle pending device activity */
        fire_timer();
    }
    /* If an interrupt was requested, PC will change! */
    if ((irq > 0) && (enab_del != 0) && (enab_rtf != 0)) {
        /* an interrupt occurs */
        to_memory(0,pc);
        pc = 1;
        cpma = pc;
        countdown -= longcycle;
        enab = 0;
    }
    /* this line handles 1 instr delay of interrupt enable */
    enab_del = enab;
    /* the actual instruction fetch is here */
    mb = from_memory(cpma);
    countdown -= shortcycle;
    switch (mb >> 7) { /* note that we decode i and z here */
    case opAND | DIRECT | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        ac = ac & from_memory(cpma);
        countdown -= longcycle;
        break;
    case opAND | DIRECT | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        ac = ac & from_memory(cpma);
        countdown -= longcycle;
        break;
    case opAND | DEFER | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        ac = ac & from_memory(cpma);
        countdown -= longcycle;
        break;
    case opAND | DEFER | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        ac = ac & from_memory(cpma);
        countdown -= longcycle;
        break;
    case opTAD | DIRECT | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        ac = (ac | link) + from_memory(cpma);
        link = ac & 010000;
        ac = ac & 007777;
        countdown -= longcycle;
        break;
    case opTAD | DIRECT | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        ac = (ac | link) + from_memory(cpma);
        link = ac & 010000;
        ac = ac & 007777;
        countdown -= longcycle;
        break;
    case opTAD | DEFER | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        ac = (ac | link) + from_memory(cpma);
        link = ac & 010000;
        ac = ac & 007777;
        countdown -= longcycle;
        break;
    case opTAD | DEFER | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        ac = (ac | link) + from_memory(cpma);
        link = ac & 010000;
        ac = ac & 007777;
        countdown -= longcycle;
        break;
    case opISZ | DIRECT | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        to_memory(cpma,((from_memory(cpma) + 1) & 07777));
        mb = from_memory(cpma);
        if (mb == 0) {
            pc = (pc + 1) & 07777;
        }
        countdown -= longcycle;
        break;
    case opISZ | DIRECT | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        to_memory(cpma,((from_memory(cpma) + 1) & 07777));
        mb = from_memory(cpma);
        if (mb == 0) {
            pc = (pc + 1) & 07777;
        }
        countdown -= longcycle;
        break;
    case opISZ | DEFER | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        to_memory(cpma,((from_memory(cpma) + 1) & 07777));
        mb = from_memory(cpma);
        if (mb == 0) {
            pc = (pc + 1) & 07777;
        }
        countdown -= longcycle;
        break;
    case opISZ | DEFER | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        to_memory(cpma,((from_memory(cpma) + 1) & 07777));
        mb = from_memory(cpma);
        if (mb == 0) {
            pc = (pc + 1) & 07777;
        }
        countdown -= longcycle;
        break;
    case opDCA | DIRECT | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        to_memory(cpma,ac);
        ac = 00000;
        countdown -= longcycle;
        break;
    case opDCA | DIRECT | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        to_memory(cpma,ac);
        ac = 00000;
        countdown -= longcycle;
        break;
    case opDCA | DEFER | ZERO:
        PAGE_ZERO;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        to_memory(cpma,ac);
        ac = 00000;
        countdown -= longcycle;
        break;
    case opDCA | DEFER | CURRENT:
        CURRENT_PAGE;
        pc = (pc + 1) & 07777;
        DEFER_CYCLE;
        to_memory(cpma,ac);
        ac = 00000;
        countdown -= longcycle;
        break;
        /* force indirect branching to use the instruction field */
// ric creo que se puede sacar #define dfr ifr
    case opJMS | DIRECT | ZERO:
        PAGE_ZERO;
        enab_rtf = 1;
        to_memory(cpma,((pc + 1) & 07777));
        countdown -= longcycle;
        pc = (cpma + 1) & 07777;
        break;
    case opJMS | DIRECT | CURRENT:
        CURRENT_PAGE;
        enab_rtf = 1;
        to_memory(cpma,((pc + 1) & 07777));
        countdown -= longcycle;
        pc = (cpma + 1) & 07777;
        break;
    case opJMS | DEFER | ZERO:
        PAGE_ZERO;
        DEFER_CYCLE;
        enab_rtf = 1;
        to_memory(cpma,((pc + 1) & 07777));
        countdown -= longcycle;
        pc = (cpma + 1) & 07777;
        break;
    case opJMS | DEFER | CURRENT:
        CURRENT_PAGE;
        DEFER_CYCLE;
        enab_rtf = 1;
        to_memory(cpma,((pc + 1) & 07777));
        countdown -= longcycle;
        pc = (cpma + 1) & 07777;
        break;
    case opJMP | DIRECT | ZERO:
        PAGE_ZERO;
        enab_rtf = 1;
        pc = cpma & 07777;
        break;
    case opJMP | DIRECT | CURRENT:
        CURRENT_PAGE;
        enab_rtf = 1;
        pc = cpma & 07777;
        break;
    case opJMP | DEFER | ZERO:
        PAGE_ZERO;
        DEFER_CYCLE;
        enab_rtf = 1;
        pc = cpma & 07777;
        break;
    case opJMP | DEFER | CURRENT:
        CURRENT_PAGE;
        DEFER_CYCLE;
        enab_rtf = 1;
        pc = cpma & 07777;
        break;
        /* undo kluge to force branches to use instruction field */
// creo se puede sacar #undef dfr
    case opIOT | DIRECT | ZERO:
    case opIOT | DIRECT | CURRENT:
    case opIOT | DEFER | ZERO:
    case opIOT | DEFER | CURRENT:
        pc = (pc + 1) & 07777;
        switch ((mb >> 3) & 077) { /* decode device address */
        case 000:
            switch (mb & 07) { /* decode CPU IOTs */
            case 00: /* SKON */
                if (enab != 0) {
                    pc = (pc + 1) & 07777;
                }
                enab = 0;
                enab_del = 0;
                break;
            case 01: /* ION */
                enab = 1;
                break;
            case 02: /* IOF */
                enab = 0;
                enab_del = 0;
                break;
            case 03: /* SRQ */
                if (irq > 0) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 04: /* GTF */
                ac = (link >> 1)       /* bit 0 */
                     | ((irq > 0) << 9)  /* bit 2 */
                     | (enab << 7)       /* bit 4 */
                     ;
                break;
            case 05: /* RTF */
                link = (ac<<1)& 010000; /* bit 0 */
                /* nothing */		/* bit 2 */
                /* nothing */		/* bit 3 */
                enab = 1;		/* bit 4 */
                /* disable interrupts until branch */
                enab_rtf = 0;
                break;
            case 06: /* SGT */
                break;
            case 07: /* CAF */
                clearflags();
                break;
            }
            break;
        case 003:
            kl8edev3(mb & 07);
            break;
        case 004:
            kl8edev4(mb & 07);
            break;
        default: /* non existant device */
            break;
        }
        break;
    case opOPR | DIRECT | ZERO:	/* group 1, CLA = 0 */
    case opOPR | DIRECT | CURRENT:	/* group 1, CLA = 1 */
        pc = (pc + 1) & 07777;
        switch ((mb >> 4) & 017) { /* decode CLA ... CML here */
        case 000:	/* NOP */
            break;
        case 001:	/*             CML */
            link = link ^ 010000;
            break;
        case 002:	/*         CMA     */
            ac = ac ^ 007777;
            break;
        case 003:	/*         CMA CML */
            ac = ac ^ 007777;
            link = link ^ 010000;
            break;
        case 004:	/*     CLL         */
            link = 000000;
            break;
        case 005:	/*     CLL     CML */
            link = 010000;
            break;
        case 006:	/*     CLL CMA     */
            ac = ac ^ 007777;
            link = 000000;
            break;
        case 007:	/*     CLL CMA CML */
            ac = ac ^ 007777;
            link = 010000;
            break;
        case 010:	/* CLA             */
            ac = 00000;
            break;
        case 011:	/* CLA         CML */
            ac = 00000;
            link = link ^ 010000;
            break;
        case 012:	/* CLA     CMA     */
            ac = 07777;
            break;
        case 013:	/* CLA     CMA CML */
            ac = 07777;
            link = link ^ 010000;
            break;
        case 014:	/* CLA CLL         */
            ac = 00000;
            link = 000000;
            break;
        case 015:	/* CLA CLL     CML */
            ac = 00000;
            link = 010000;
            break;
        case 016:	/* CLA CLL CMA     */
            ac = 07777;
            link = 000000;
            break;
        case 017:	/* CLA CLL CMA CML */
            ac = 07777;
            link = 010000;
            break;
        }
        if (mb & 00001) { /* IAC */
            ac = (ac | link) + 1;
            link = ac & 010000;
            ac = ac & 007777;
        }
        switch ((mb >> 1) & 07) { /* decode RAR,RAL,TWO */
        case 00:	/* NOP */
            break;
        case 01:	/*         TWO -- BSW */
            ac = ((ac & 07700) >> 6)
                 | ((ac & 00077) << 6);
            break;
        case 02:	/*     RAL     */
            ac = (ac << 1) | (link >> 12);
            link = ac & 010000;
            ac = ac & 007777;
            break;
        case 03:	/*     RAL TWO */
            ac = (ac << 2) | ((ac | link) >> 11);
            link = ac & 010000;
            ac = ac & 007777;
            break;
        case 04:	/* RAR         */
            ac = ((ac | link) >> 1) | (ac << 12);
            link = ac & 010000;
            ac = ac & 007777;
            break;
        case 05:	/* RAR     TWO */
            ac = ((ac | link) >> 2) | (ac << 11);
            link = ac & 010000;
            ac = ac & 007777;
            break;
        case 06:	/* RAR RAL     */
            /* this uses a data path meant for AND */
            ac = ac & mb;
            break;
        case 07:	/* RAR RAL TWO */
            /* this uses an addressing data path */
            ac = ((pc - 1) & 07600) | (mb & 00177);
            break;
        }
        break;
    case opOPR | DEFER | ZERO:	/* group 2,3 CLA = 0 */
    case opOPR | DEFER | CURRENT:   /* group 2,3 CLA = 1 */
        if ((mb & 00001) == 0) { /* GROUP 2 */
            pc = (pc + 1) & 07777;
            switch ((mb & 00170) >> 3) { /* SMA ... REV */
            case 000: /* NOP */
                break;
            case 001: /*             REV */
                pc = (pc + 1) & 07777;
                break;
            case 002: /*         SNL     */
                if (link) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 003: /*         SNL REV */
                if (link == 0) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 004: /*     SZA         */
                if (ac == 0) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 005: /*     SZA     REV */
                if (ac) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 006: /*     SZA SNL     */
                if ((ac == 0) || link) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 007: /*     SZA SNL REV */
                if (ac && (link == 0)) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 010: /* SMA             */
                if (ac & 04000) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 011: /* SMA         REV */
                if ((ac & 04000) == 0) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 012: /* SMA     SNL     */
                if ((ac | link) & 014000) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 013: /* SMA     SNL REV */
                if (((ac | link) & 014000) == 0) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 014: /* SMA SZA         */
                if ((ac & 04000) || (ac == 0)) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 015: /* SMA SZA     REV */
                if (((ac & 004000) == 0) && ac) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 016: /* SMA SZA SNL     */
                if (((ac | link) & 014000)
                        || (ac == 0)) {
                    pc = (pc + 1) & 07777;
                }
                break;
            case 017: /* SMA SZA SNL REV */
                if ((((ac | link) & 014000) == 0)
                        && (ac)) {
                    pc = (pc + 1) & 07777;
                }
                break;
            }
            if (mb & 00200) { /* CLA */
                ac = 00000;
            }
            if (mb & 00004) { /* OSR */
                ac = ac | sr;
            }
            if (mb & 00002) { /* HLT */
                countdown = 0;
                run = 0;
            }
        } else { /* GROUP 3 */
            pc = (pc + 1) & 07777;
            if (mb & 00200) { /* CLA */
                ac = 00000;
            }
            if ((mb & 00120) == 00100) { /* MQA */
                ac = mq | ac;
            } else if ((mb & 00120) == 00020) { /* MQL */
                mq = ac;
                ac = 00000;
            } else if ((mb & 00120) == 00120) { /*MQA,MQL*/
                int temp;
                temp = mq;
                mq = ac;
                ac = temp;
            }
        }
    }
}
