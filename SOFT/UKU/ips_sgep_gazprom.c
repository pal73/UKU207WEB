#include "ips_sgep_gazprom.h"
#include "25lc640.h"
#include "eeprom_map.h"
#include "main.h"

void ipsSgepGazpromEeSets(void)
{
if(lc640_read_int(ADR_EE_BAT_IS_ON[0])!=bisOFF)lc640_write_int(ADR_EE_BAT_IS_ON[0],bisOFF);
if(lc640_read_int(ADR_EE_BAT_IS_ON[1])!=bisOFF)lc640_write_int(ADR_EE_BAT_IS_ON[1],bisOFF);
if(lc640_read_int(EE_HV_VZ_STAT)!=hvsOFF)lc640_write(EE_HV_VZ_STAT,hvsOFF);
if(lc640_read_int(EE_TBAT)!=0)lc640_write_int(EE_TBAT,0);
if(lc640_read_int(EE_TZAS)!=3)lc640_write_int(EE_TZAS,0);
}
