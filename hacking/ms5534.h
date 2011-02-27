#ifndef MS5534_H_
#define MS5534_H_


#define WRITE_1()       {XPORT_GPIO_DATA |= GPIO_MS5534B_DIN; XPORT_GPIO_DATA |= GPIO_MS5534B_SCLK; XPORT_GPIO_DATA &= (u16)~GPIO_MS5534B_SCLK;}
#define WRITE_0()       {XPORT_GPIO_DATA &= (u16)~GPIO_MS5534B_DIN; XPORT_GPIO_DATA |= GPIO_MS5534B_SCLK; XPORT_GPIO_DATA &= (u16)~GPIO_MS5534B_SCLK;}
#define CLK()           {XPORT_GPIO_DATA |= GPIO_MS5534B_SCLK; XPORT_GPIO_DATA &= (u16)~GPIO_MS5534B_SCLK;}
#define ACCUM(w,clk)    {XPORT_GPIO_DATA |= GPIO_MS5534B_SCLK; XPORT_GPIO_DATA &= (u16)~GPIO_MS5534B_SCLK; w |= (((XPORT_GPIO_DATA & GPIO_MS5534B_DOUT)? 1 : 0) << clk);}

#define START()                  {WRITE_1();WRITE_1();WRITE_1();}
#define STOP()                   {WRITE_0();WRITE_0();WRITE_0();}

#define CONV_COMPLETE()          ((XPORT_GPIO_DATA & GPIO_MS5534B_DOUT) == 0)

#define SENSOR_ST_IDLE           11
#define SENSOR_ST_TCONV          22
#define SENSOR_ST_PCONV          33

// approximately 65mS per P+T sample, which is about 15 P+T samples/second
#define SENSOR_SAMPLES_PER_SEC   15
#define SENSOR_MAX_SAMPLES       120

#endif

/*
 * $Header$
 */
