#include "ntc.h"

#define NTC_B57500M_REF_POINT_TEMP                25.0      // ����������� ��� ������� �������� ������������� ������������
#define NTC_B57500M_REF_POINT_RES                 10000.0   // ������������� ������������ ��� ����������� T_REF_POINT_TEMP

#define NTC_B57500M_COEFF_BP                      3988.0    // ������������ ������������
#define NTC_B57500M_COEFF_BM                      3785.0;//3988.0    // ������������ ������������

// ������ �� -50�
#define NTC_B57500M_MAX_RES                       670100.0  // ������������ ������������� -50C
//  #define NTC_B57500M_MAX_RES                       963000.0  // ������������ ������������� -55C
//  #define NTC_B57500M_MAX_RES                       177000.0  // ������������ ������������� -30C

// ������ �� +100�
//  #define NTC_B57500M_MIN_RES                       165.3     // ����������� ������������� 155C
//  #define NTC_B57500M_MIN_RES                       389.3     // ����������� ������������� 120C
#define NTC_B57500M_MIN_RES                       680.0     // ����������� ������������� 100C
#define NTC_B57500M_MIN_TEMP                      -55       // ����������� �����������
#define NTC_B57500M_MAX_TEMP                      155       // ����������� �����������
/*
int NTC_B57500M_GetTemperatureC(uint32_t Urt_mV, float *ret_temp)
{
  double Rt, Urt;
  *ret_temp = 0;

  if(!is_init) return NTC_B575 00M_ERR_INTERNAL;
  if(!ret_temp) return NTC_B57500M_ERR_INTERNAL;

  Urt = Urt_mV / 1000.0;
  Rt = Urt * r1 / (u0 - Urt);	


  if( Rt > NTC_B57500M_MAX_RES ) return NTC_B57500M_ERR_OVER_TEMP;//Rt = 200000;
  if( Rt < NTC_B57500M_MIN_RES ) return NTC_B57500M_ERR_LOW_TEMP;//Rt = 1000;
  if( Rt < r0 )
    *ret_temp = 1.0f / (1.0f / bp * (float)log(Rt / r0) + 1.0f / (t0 + 273.0f) ) - 273.0f;
  else
    *ret_temp = 1.0f / (1.0f / bm * (float)log(Rt / r0) + 1.0f / (t0 + 273.0f) ) - 273.0f;
  return NTC_B57500M_ERR_OK;
};
*/
