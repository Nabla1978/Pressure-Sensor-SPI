#include "utils.h"



void CSensor::Init()
   {
   m_nSmpCnt = 0;
   m_State = SENSOR_ST_IDLE;
   m_nKLPFA = 4;
   m_nKLPFC = SENSOR_DFL_KLPFC;
   m_nPZTblSel = SENSOR_DFL_PZTBL_SEL;
   m_nAltM = m_nAltCm = m_nAltCmx16 = 0;
   m_nCmPerSec = m_nCmPerSecx16 = 0;
   m_nWindowSecs = SENSOR_DFL_WINDOW_SECS;
   m_nWindowSamples = m_nWindowSecs * SENSOR_SAMPLES_PER_SEC;
   m_nTempCorrInx = SENSOR_DFL_TEMP_CORR_INX;

   // Set  GPIO_MS5534B_SCLK and GPIO_MS5534B_DIN pins as outputs, GPIO_MS5534B_DOUT as input
   XPORT_GPIO_DIR |= (GPIO_MS5534B_SCLK | GPIO_MS5534B_DIN);
   XPORT_GPIO_DIR &= (u16)~GPIO_MS5534B_DOUT;

   Reset();
   ReadCoeffs();
   }


void CSensor::Reset()
   {
   int cnt;

   cnt = 8;
   while (cnt--)
      {
      WRITE_1();
      WRITE_0();
      }

   cnt = 5;
   while (cnt--)
      {
      WRITE_0();
      }
   }




void CSensor::ReadCoeffs()
   {
   int clk;

   // CW 1 //
   START();

   WRITE_0();
   WRITE_1();
   WRITE_0();
   WRITE_1();
   WRITE_0();
   WRITE_1();

   STOP();

   CLK();

   m_CoeffWord1 = 0;
   clk = 16;
   while(clk--)
      {
      ACCUM(m_CoeffWord1,clk);
      }

   CLK();

   // CW 2  //
   START();

   WRITE_0();
   WRITE_1();
   WRITE_0();
   WRITE_1();
   WRITE_1();
   WRITE_0();

   STOP();

   CLK();

   m_CoeffWord2 = 0;
   clk = 16;
   while(clk--)
      {
      ACCUM(m_CoeffWord2,clk);
      }

   CLK();

   // CW 3 //
   START();

   WRITE_0();
   WRITE_1();
   WRITE_1();
   WRITE_0();
   WRITE_0();
   WRITE_1();

   STOP();

   CLK();
   
   m_CoeffWord3 = 0;
   clk = 16;
   while(clk--)
      {
      ACCUM(m_CoeffWord3,clk);
      }

   CLK();

   // CW 4 //
   START();

   WRITE_0();
   WRITE_1();
   WRITE_1();
   WRITE_0();
   WRITE_1();
   WRITE_0();

   STOP();

   CLK();

   m_CoeffWord4 = 0;
   clk = 16;
   while(clk--)
      {
      ACCUM(m_CoeffWord4,clk);
      }

   CLK();

   m_C1 = int((m_CoeffWord1 >> 1) & 0x7FFF);
   m_C5 = int(((m_CoeffWord1 & 0x1)<<10) | ((m_CoeffWord2 >> 6) & 0x3FF));
   m_C6 = int(m_CoeffWord2 & 0x3F);
   m_C4 = int((m_CoeffWord3>>6) & 0x3FF);
   m_C2 = int(((m_CoeffWord3 & 0x3F) << 6) | (m_CoeffWord4 & 0x3F));
   m_C3 =  int((m_CoeffWord4 >> 6) & 0x3FF);
   }



void CSensor::TriggerPressureSample()
   {
   START();

   WRITE_1();
   WRITE_0();
   WRITE_1();
   WRITE_0();

   STOP();

   CLK();
   CLK();
   }


u16 CSensor::ReadPressure()
   {
   u16 D1;
   int clk;
   clk = 16;
   D1 = 0;
   while (clk--)
      {
      ACCUM(D1,clk);
      }
   CLK();
   return D1;
   }


void CSensor::TriggerTemperatureSample()
   {
   START();

   WRITE_1();
   WRITE_0();
   WRITE_0();
   WRITE_1();

   STOP();

   CLK();
   CLK();
   }


u16 CSensor::ReadTemperature()
   {
   u16 D2;
   int clk;

   clk = 16;
   D2 = 0;
   while (clk--)
      {
      ACCUM(D2,clk);
      }
   CLK();
   return D2;
   }


//int sTick;

void CSensor::ConvertPressureTemperature(int pzInx, int tcInx)
    {
    int UT1,dT,OFF,SENS,X,P,TEMP,P2,T2;

   // verbatim from data sheet
    UT1  = (m_C5<<3) + 20224;
    dT   = (int)m_D2 - UT1;
    TEMP = 200 + ((dT*(m_C6+50))>>10);

    OFF  = (m_C2<<2) + (((m_C4-512)*dT)>>12);
    SENS = m_C1 + ((m_C3*dT)>>10)  + 24576;
    X    = ((SENS* ((int)m_D1 - 7168))>>14)  - OFF;
    P    = ((X*10)>>5) + 2500;
 //   P    = ((X*100)>>5) + 25000;

    if (TEMP < 200)
        {
        T2 = (11*(m_C6+24)*(200-TEMP)*(200-TEMP))>>20;
        P2 = (3*T2*(P-3500))>>14;
//        P2 = (3*T2*(P-35000))>>14;
        }
    else
        {
        T2 = 0;
        P2 = 0;
        }

    TEMP = TEMP-T2;
    P = P-P2;

    m_SmpBuf[m_nSmpCnt].tck = (int)gTmr.ticks;
    // ms5534 P reading has 0.1mbar resolution, LUT has 1 pa input resolution.
    // 0.1mbar == 10pa
    m_SmpBuf[m_nSmpCnt].p = m_nPresPaSample = P*10;
//    m_SmpBuf[m_nSmpCnt].p = m_nPresPaSample = P;
    m_SmpBuf[m_nSmpCnt].t = m_nTempCx10Sample = TEMP;
    m_SmpBuf[m_nSmpCnt].z = Pa2Cm(m_nPresPaSample, pzInx, tcInx);

    // increment the pointer, now it points to the oldest sample in the circular buffer
    m_nSmpCnt++;
    if (m_nSmpCnt == m_nWindowSamples)
       {
//       ham_DrawText(0,0,"t%06d",((gTmr.ticks-sTick)*TMR_TICK_MS)/m_nWindowSamples);
       m_nSmpCnt = 0;
//       sTick = gTmr.ticks;
       }
    }


void CSensor::InitData()
   {
   InitWindowBuffers();
   m_State = SENSOR_ST_IDLE;
   }
   
void CSensor::InitWindowBuffers()
   {
   int n,tck;
   AcquireAveragedSample(4, m_nPZTblSel, m_nTempCorrInx);
   tck = int(gTmr.ticks);
   m_nSmpCnt =  m_nWindowSamples-1;
   n = m_nWindowSamples;
   while(n--)
      {
      m_SmpBuf[n].z  = m_nAltCm;
      m_SmpBuf[n].p  = m_nPresPa;
      m_SmpBuf[n].t = m_nTempCx10;
      m_SmpBuf[n].tck = tck - m_nWindowSamples + n;
      }
   }
   

void CSensor::AcquireAveragedSample(int nSamples, int pzInx, int tcInx)
   {
   u32 eTick;
   int pAccum, tAccum, n;

   n = nSamples;
   pAccum = 0;
   tAccum = 0;
   while(n--)
      {
      Reset();
      TriggerTemperatureSample();
      // typically 35mS for a conversion, timeout after 50mS
      eTick = WAIT_TICK(50);
      while((CONV_COMPLETE() == false) && gTmr.ticks < eTick);
      if (gTmr.ticks >= eTick) return;
      m_D2   = ReadTemperature();
      Reset();
      TriggerPressureSample();
      eTick = WAIT_TICK(50);
      while((CONV_COMPLETE() == false) && gTmr.ticks < eTick);
      if (gTmr.ticks >= eTick) return;
      m_D1 = ReadPressure();
      ConvertPressureTemperature(pzInx, tcInx);
      pAccum += m_nPresPaSample;
      tAccum += m_nTempCx10Sample;
      }
   m_nPresPa = pAccum/nSamples;
   m_nTempCx10 = tAccum/nSamples;
   m_nAltCm = Pa2Cm(m_nPresPa,pzInx,tcInx);
   m_nAltCmx16 = m_nAltCm<<4;
   m_nAltM = (m_nAltCm+50)/100;
   m_nCmPerSecx16 = 0;
   m_nCmPerSec = 0;
   m_nSmpCnt = 0;
   }

void CSensor::CalibrateT0()
   {
   int nT0;
   // Do the altitude computation using nominal values
   AcquireAveragedSample(4,SENSOR_DFL_PZTBL_SEL,SENSOR_DFL_TEMP_CORR_INX);
   nT0 = m_nTempCx10/10 + (65*m_nAltM)/10000;
   // Temp Corr LUT has range -10 to 40
   CLAMP(nT0,SENSOR_T0_MIN,SENSOR_T0_MAX);
   m_nTempCorrInx = nT0 + 10; 
   // now redo the altitude computation using the actual values
   AcquireAveragedSample(4,m_nPZTblSel,m_nTempCorrInx);
   }

void CSensor::GetAveragedAltitude(int nSamples)
   {
   AcquireAveragedSample(nSamples,m_nPZTblSel,m_nTempCorrInx);
   }

void CSensor::AveragePressureTemperature()
   {
   int n,pAccum,tAccum;
   tAccum = pAccum = 0;
   for (n = 0; n < m_nWindowSamples; n++)
      {
      pAccum += m_SmpBuf[n].p;
      tAccum += m_SmpBuf[n].t;
      }
   m_nPresPa = (pAccum+m_nWindowSamples/2)/m_nWindowSamples;
   m_nTempCx10 = (tAccum+m_nWindowSamples/2)/m_nWindowSamples;
   }


int CSensor::DeltaCmPerSec()
    {
    int n;
    s64 dZ,sum_zt,sum_z,sum_t,sum_t2,trel,num,den,tref,z;

    // time reference is the oldest sample in window
    tref = s64(m_SmpBuf[m_nSmpCnt].tck);

    sum_zt = sum_z = sum_t = sum_t2 = 0;
    for (n = 0; n < m_nWindowSamples; n++)
        {
        z = m_SmpBuf[n].z;
        trel = (m_SmpBuf[n].tck - tref);
        sum_z += z;
        sum_t += trel;
        sum_t2 += (trel*trel);
        sum_zt += (trel*z);
        }

    num = (m_nWindowSamples*sum_zt) - (sum_z*sum_t);
    den = (m_nWindowSamples*sum_t2) - (sum_t*sum_t);

    dZ = den ? (num*TMR_TICKS_PER_SEC + den/2)/den : 0;
    return int(dZ);
    }

void CSensor::ReadSampleContinuous()
   {
   int altCm,cmPerSec;

   switch (m_State)
      {
      case SENSOR_ST_IDLE :
      default :
      Reset();
      TriggerTemperatureSample();
      m_State = SENSOR_ST_TCONV;
      break;

      case SENSOR_ST_TCONV :
      if (CONV_COMPLETE())
         {
         m_D2 = ReadTemperature();
         Reset();
         TriggerPressureSample();
         m_State = SENSOR_ST_PCONV;
         }
      break;

      case SENSOR_ST_PCONV :
      if (CONV_COMPLETE())
         {
         m_D1 = ReadPressure();

         // trigger the next sample immediately to maximize samples/sec
         Reset();
         TriggerTemperatureSample();
         
         ConvertPressureTemperature(m_nPZTblSel,m_nTempCorrInx);
         AveragePressureTemperature();
         altCm = Pa2Cm(m_nPresPa,m_nPZTblSel,m_nTempCorrInx);
         LPFAltitude(altCm);
         cmPerSec = DeltaCmPerSec();
         LPFClimbRate(cmPerSec);
         m_State = SENSOR_ST_TCONV;
         }
      break;
      }
   }


/*
 * $Header$
 */

