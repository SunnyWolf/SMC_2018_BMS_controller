#include "pec15.h"

int16_t pec15Table[256];
#define CRC15_POLY 0x4599

void PEC15_Init()
{
  int16_t remainder;
  for (int i = 0; i < 256; i++)
  {
    remainder = i << 7;
    for (int bit = 8; bit > 0; --bit)
    {
      if (remainder & 0x4000)
      {
        remainder = ((remainder << 1));
        remainder = (remainder ^ CRC15_POLY);
      }
      else
      {
        remainder = ((remainder << 1));
      }
    }
    pec15Table[i] = remainder&0xFFFF;
  }
}

uint16_t PEC15(uint8_t *data , int len)
{
  int16_t remainder, address;
  remainder = 16;//PEC seed
  for (int i = 0; i < len; i++)
  {
    address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
    remainder = (remainder << 8 ) ^ pec15Table[address];
  }
  return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}
