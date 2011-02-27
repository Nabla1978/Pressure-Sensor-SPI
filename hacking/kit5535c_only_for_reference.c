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

calib_req(long word)
  {
  int i;
  for(i = 0; i < 3; i++) send_hi();
  send_lo();	send_hi();
  if (word == 1)
  {
     send_lo();  send_hi();  send_lo();  send_hi();
  }
  if (word == 2)
  {
     send_lo();  send_hi();  send_hi();  send_lo();
  }
  if (word == 3)
  {
     send_hi();  send_lo();  send_lo();  send_hi();
  }
  if (word == 4)
  {
     send_hi();  send_lo();  send_hi();  send_lo();
  }

  for(i = 0; i < 4; i++) send_lo();

  };

d1_req()
  {
  int i;
  for(i = 0; i < 3; i++) send_hi();
  send_hi();	send_lo();
  send_hi();	send_lo();
  for(i = 0; i < 5; i++) send_lo();
  };

d2_req()
  {
  int i;
  for(i = 0; i < 3; i++) send_hi();
  send_hi();	send_lo();
  send_lo();	send_hi();
  for(i = 0; i < 5; i++) send_lo();
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
long cali_read(long word)
  {
  calib_req(word);
  delay(1);
  return(adc_read());
  };

coeff()
  {
  long word1,word2,word3,word4;

  word1=cali_read(1);
  word2=cali_read(2);
  word3=cali_read(3);
  word4=cali_read(4);


  c1 = word1>>3;
  c2 = word1<<10;
  c2 &= 0x1c00;
  c2 += word2>>6;
  c3 = word3>>6;
  c4 = word4>>7;
  c5 = word2 & 0x3f;
  c5 <<= 6;
  c5 |= (word3 & 0x3f);
  c6 = word4 & 0x7f;
}

long press()
  {
     long ut20,off,sens,p;  /*d1, d2*/

     d1 = d1_read();
     d2 = d2_read();
     ut20  = 8*c5+10000;
     dut  = d2-ut20;

     off  = c2+(c4-250)*dut/4096+10000;
     sens = c1/2+(c3+200)*dut/8192+3000;
     p    = sens*(d1-off)/4096+1000;
     return(p);
  }

long temp()
  {
  long temp, dut2;

  /* The following term dut2 is the optional 2nd order temperature  */
  /* calculation for a most accurate temperature reading            */

  if (dut<0) dut2 = dut - (dut/128*dut/128)/4;
   else dut2 = dut - (dut/128*dut/128)/8;

  temp = 200+dut2*(c6+100)/2048;

  return(temp);
  }


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