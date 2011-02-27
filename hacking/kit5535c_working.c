/************************************************************************/
/*                                                                      */
/* Program PCDEP.C for reading of pressure, temperature and             */
/* calibration data of MS5535 (12 bar module). Displays compensated     */
/* Temperature and Pressure only (no depth display in first version)    */
/* 									*/
/* Date: 14.3.98 First version, with linear temperature compensation	*/
/*                                                                      */
/************************************************************************/

/*

This program runs under MSDOS or Windows NT4.0 (in MSDOS Window)
The frequency at SCLK is defined to approx 1ms in the subroutine
del(int time). To have it faster delete the line delay(1) and
use the for(..) loop. Adjust with the constants del_hi and del_lo.

*/


#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <stdlib.h>

#define  L 1500

typedef unsigned int WORD;
int base = 956;
int datin[L];
int sclk = 0;
int sdi = 0;
int rest = 0xfc;
long dut,c1,c2,c3,c4,c5,c6,d1,d2;


const del_hi = 120/30;
const del_lo = 320/30;

del(int time);
oport(void);
int  iport(void);
sclklo(void);
sclkhi(void);
sdilo(void);
sdihi(void);
send_hi(void);
send_lo(void);
reset(void);
calib_req(long word);
d1_req(void);
d2_req(void);
long adc_read(void);
long d1_read(void);
long d2_read(void);
long cali_read(long word);
coeff(void);
long press(void);
long temp(void);


/*** Unterprogramm(e) ***************************************************/

del(int time)
  {
  int i;
  delay(1);
  /*
  for (i = 0; i < (time << 5); i++)
    i = i;
  */
  };

oport()
  {
  int outb;
  outb = sclk + (sdi << 1) + rest;
  outportb(base, outb);
  };

int iport()
  {
  return((inportb(base + 1) & 32) >> 5);
  };


sdihi()
{
  sdi = 1;	oport();	del(del_hi);
}

sdilo()
{
  sdi = 0;	oport();	del(del_lo);
}
sclkhi()
{
  sclk = 1;	oport();	del(del_hi);
}

sclklo()
{
  sclk = 0;	oport();	del(del_lo);
}

send_hi()
  {
  sdihi();
  sclkhi();
  sdilo();
  sclklo();
  };

send_lo()
  {
  sclk = 1;	oport();	del(del_hi);
  sclk = 0;	oport();	del(del_lo);
  };

reset()
  {
  int i = 0;
  for(i = 0; i < 9; i++)
    {
    send_hi();
    send_lo();
    };
  for(i = 0; i < 3; i++)
    {
    send_lo();
    };
  };



long adc_read()
  {
  int i = 0;
  long di = 0;
  long adc_val = 0;

  send_lo();
  delay(1);
  for(i = 0; i < 16; i++)
    {
    di = (long) iport();
    di = di << (15 - i);
    adc_val += di;
    send_lo();
    delay(1);
    };
  return(adc_val);
  };

long d1_read()
  {
  d1_req();
  delay(1);
  do
   if(kbhit()) break;
  while ( iport() != 0);
  return(adc_read());
  };

long d2_read()
  {
  d2_req();
  delay(1);
  do
   if(kbhit()) break;
  while ( iport() != 0);

  return(adc_read());
  };





/************************************************************************/
/* (Anschluss an LPT1 port des Laptops)!                                */
/* Pin 1  - ws		open						*/
/* Pin 2  - gb  - SCLK	data 1	                                        */
/* Pin 3  - org - SDI	data 2	                                        */
/* Pin 4  - gn  - VDD	data 4						*/
/* Pin 10 - rt  - open	data 64						*/
/* Pin 11 - br  - open  data x128					*/
/* Pin 12 - bl 	- SDO  	data 32						*/
/* Pin 25 - sw - Masse                                                  */
/************************************************************************/

main()
{

/* Variablen und Constanten */

int i, j, k;
long p,t;
float p_fil,t_fil;
FILE *dat_pointer;


base = *(WORD far *) MK_FP(0x0040, 8);

/*** Hauptprogramm ******************************************************/

reset();
coeff();
clrscr();

reset();
p = press();
t = temp();


p_fil = p;

gotoxy(10,8);
printf("---------------------------------------------------------");
gotoxy(10,14);
printf("---------------------------------------------------------");

for(;;)
  {

  p = press();
  t = temp();

  if (p>300)
  {
  p_fil = (4*p_fil+(p-p_fil))/4;
  t_fil = t;
  }
  gotoxy(12,10);
  printf("Temperature %.2f øC    Pressure %.0f mbar\n", t_fil/10,p_fil);
/*  printf("                 D1 %ld \n", d1);  */
/*  printf("                 D2 %ld \n", d2);  */
  printf("\n");
  if (kbhit()) break;
  };
}

