/*
* Copyright (C) 2010 Realtek Semiconductor Corp.
* All Rights Reserved.
*
* This program is the proprietary software of Realtek Semiconductor
* Corporation and/or its licensors, and only be used, duplicated,
* modified or distributed under the authorized license from Realtek.
*
* ANY USE OF THE SOFTWARE OTEHR THAN AS AUTHORIZED UNDER 
* THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
* 
* $Revision: 10083 $ 
* $Date: 2010-06-07 11:18:41 +0800 (星期一, 07 六月 2010) $
*
* Purpose : MDC/MDIO example code
*
* Feature :  This file consists of following modules:
*                
*
*/


#include "rtl8306e_types.h"
#include "mdcmdio.h"      


/* Function Name:
 *      smiRead
 * Description:
 *      Read data from phy register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 * Output:
 *      data    -  Register value 
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *     This function could read register through MDC/MDIO serial 
 *     interface, and it is platform  related. It use two GPIO pins 
 *     to simulate MDC/MDIO timing. MDC is sourced by the Station Management 
 *     entity to the PHY as the timing reference for transfer of information
 *     on the MDIO signal. MDC is an aperiodic signal that has no maximum high 
 *     or low times. The minimum high and low times for MDC shall be 160 ns each, 
 *     and the minimum period for MDC shall be 400 ns. Obeying frame format defined
 *     by IEEE802.3 standard, you could access Phy registers. If you want to 
 *     port it to other CPU, please modify static functions which are called 
 *      by this function.
 */
int32 smiRead(uint32 phyad, uint32 regad, uint32 * data) 
{
    if ((phyad > 31) || (regad > 31) || (data == NULL))  
        return FAILED;
	
    RMII_Read((unsigned long)phyad, (unsigned long)regad, (unsigned long *)data);
        
    return  SUCCESS;
}

/* Function Name:
 *      smiWrite
 * Description:
 *      Write data to Phy register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31)
 *      data    -  Data to be written into Phy register
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *     This function could read register through MDC/MDIO serial 
 *     interface, and it is platform  related. It use two GPIO pins 
 *     to simulate MDC/MDIO timing. MDC is sourced by the Station Management 
 *     entity to the PHY as the timing reference for transfer of information
 *     on the MDIO signal. MDC is an aperiodic signal that has no maximum high 
 *     or low times. The minimum high and low times for MDC shall be 160 ns each, 
 *     and the minimum period for MDC shall be 400 ns. Obeying frame format defined
 *     by IEEE802.3 standard, you could access Phy registers. If you want to 
 *     port it to other CPU, please modify static functions which are called 
*      by this function.
 */

int32 smiWrite(uint32 phyad, uint32 regad, uint32 data)
{            
    if ((phyad > 31) || (regad > 31) || (data > 0xFFFF))  
        return FAILED;
	
    RMII_Write((unsigned long)phyad, (unsigned long)regad, (unsigned long)data);
    
    return SUCCESS; 
}


/* Function Name:
 *      smiInit
 * Description:
 *      Init Rtl8651B smi interface
 * Input:
 *      port        - Specify Rtl8651B GPIO port
 *      pinMDC    - Set which gpio pin as MDC 
 *      pinMDIO   - Set which gpio pin as MDIO
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *      This function is only for Rtl8651B, use it to specify
 *      GPIO pins as MDC/MDIO signal. It should be called at first.
 */
int32 smiInit(uint32 port, uint32 pinMDC, uint32 pinMDIO)
{
    return SUCCESS;
}


/* Function Name:
 *      smiReadBit
 * Description:
 *      Read one bit of PHY register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 *      bit       -  Register bit (0~15)   
 * Output:
 *      pdata    - the pointer of  Register bit value 
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 */

int32 smiReadBit(uint32 phyad, uint32 regad, uint32 bit, uint32 * pdata) 
{
    uint32 regData;

    if ((phyad > 31) || (regad > 31) || (bit > 15) || (pdata == NULL) ) 
        return  FAILED;
    
    if(bit>=16)
        * pdata = 0;
    else 
    {
        smiRead(phyad, regad, &regData);
        if(regData & (1<<bit)) 
            * pdata = 1;
        else
            * pdata = 0;
    }
    
    return SUCCESS;
}

/* Function Name:
 *      smiWriteBit
 * Description:
 *      Write one bit of PHY register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 *      bit       -  Register bit (0~15)   
 *      data     -  Bit value to be written
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 */

int32 smiWriteBit(uint32 phyad, uint32 regad, uint32 bit, uint32 data) 
{
    uint32 regData;
    
    if ((phyad > 31) || (regad > 31) || (bit > 15) || (data > 1) ) 
        return  FAILED;
	
    smiRead(phyad, regad, &regData);
    
    if(data) 
        regData = regData | (1<<bit);
    else
        regData = regData & ~(1<<bit);
	
    smiWrite(phyad, regad, regData);
    
    return SUCCESS;
}




















