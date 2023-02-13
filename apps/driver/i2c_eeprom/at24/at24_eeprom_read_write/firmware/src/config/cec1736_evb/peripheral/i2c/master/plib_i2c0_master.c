/*******************************************************************************
  Inter-Integrated Circuit (I2C) Library
  Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_i2c0_master.c

  Summary:
    I2C PLIB Master Mode Implementation file

  Description:
    This file defines the interface to the I2C peripheral library.
    This library provides access to and control of the associated peripheral
    instance.

*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "device.h"
#include "interrupts.h"
#include "../plib_i2c_smb_common.h"
#include "plib_i2c0_master.h"
#include "../../ecia/plib_ecia.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

#define I2C0_START_CONDITION  (SMB_WCTRL_PIN_Msk | SMB_WCTRL_ESO_Msk | SMB_WCTRL_ENI_Msk | SMB_WCTRL_STA_Msk | SMB_WCTRL_ACK_Msk)

#define I2C0_STOP_CONDITION  (SMB_WCTRL_PIN_Msk | SMB_WCTRL_ESO_Msk | SMB_WCTRL_ENI_Msk | SMB_WCTRL_STO_Msk | SMB_WCTRL_ACK_Msk)

#define I2C0_REPEATED_START_CONDITION  (SMB_WCTRL_ESO_Msk | SMB_WCTRL_ENI_Msk | SMB_WCTRL_STA_Msk | SMB_WCTRL_ACK_Msk)

#define I2C0_NAK  (SMB_WCTRL_ESO_Msk | SMB_WCTRL_ENI_Msk)


static I2C_OBJ i2c0Obj;

void I2C0_Initialize(void)
{
    /* PIN = 1 (Software reset, self-clearing bit) */
    SMB0_REGS->SMB_WCTRL = SMB_WCTRL_PIN_Msk;

    /* Set the port associated with this instance of I2C peripheral */
    SMB0_REGS->SMB_CFG[0] = SMB_CFG_PORT_SEL(6);

    /* Enable I2C peripheral */
    SMB0_REGS->SMB_CFG[0] |= SMB_CFG_EN_Msk;

    /* I2C bus clock frequency */
    SMB0_REGS->SMB_BUSCLK = SMB_BUSCLK_HIGH_PER(19) | SMB_BUSCLK_LOW_PER(19);

    /* Enable Serial output and set ACK bit */
    SMB0_REGS->SMB_WCTRL = (SMB_WCTRL_ESO_Msk | SMB_WCTRL_ACK_Msk);

    /* Repeated start hold time setup */
    SMB0_REGS->SMB_RSHTM = I2C_SMB_RecommendedValues[2][1];

    /* Data timing register */
    SMB0_REGS->SMB_DATATM = I2C_SMB_RecommendedValues[1][1];

    i2c0Obj.callback = NULL;

    i2c0Obj.state = I2C_MASTER_STATE_IDLE;

    i2c0Obj.error = I2C_ERROR_NONE;
}

/* I2C state machine */
static void I2C0_TransferSM(void)
{
    /* check for bus error or arbitration lost error */
    if ((SMB0_REGS->SMB_RSTS & SMB_RSTS_BER_Msk) != 0U)
    {
        i2c0Obj.error = I2C_ERROR_BUS_COLLISION;
    }
    if ((SMB0_REGS->SMB_RSTS & SMB_RSTS_LAB_Msk) != 0U)
    {
        i2c0Obj.error = I2C_ERROR_ARBITRATION_LOST;
    }

    if (i2c0Obj.error != I2C_ERROR_NONE)
    {
        i2c0Obj.state = I2C_MASTER_STATE_IDLE;

        /* Write transfer complete. Give callback. */
        if (i2c0Obj.callback != NULL)
        {
            i2c0Obj.callback(i2c0Obj.context);
        }

        return;
    }

    switch (i2c0Obj.state)
    {
        case I2C_MASTER_STATE_TRANSMIT:
            /* Enter here for write, write-read transfers */
            /* if last received bit is 0 (ack from slave), then transmit more bytes */
            if ((SMB0_REGS->SMB_RSTS & SMB_RSTS_LRB_AD0_Msk) == 0U)
            {
                if (i2c0Obj.writeCount < i2c0Obj.writeSize)
                {
                    SMB0_REGS->SMB_I2CDATA = i2c0Obj.writeBuffer[i2c0Obj.writeCount++];

                    if (i2c0Obj.writeCount >= i2c0Obj.writeSize)
                    {
                        if (i2c0Obj.readCount < i2c0Obj.readSize)
                        {
                            /* It's a I2C Write-Read transfer */
                            i2c0Obj.state = I2C_MASTER_STATE_REPEATED_START;
                        }
                    }
                }
                else
                {
                    /* All bytes transmitted out. Generate stop condition and give a callback. */
                    SMB0_REGS->SMB_WCTRL = I2C0_STOP_CONDITION;
                    i2c0Obj.state = I2C_MASTER_STATE_IDLE;

                    /* Write transfer complete. Give callback. */
                    if (i2c0Obj.callback != NULL)
                    {
                        i2c0Obj.callback(i2c0Obj.context);
                    }
                }
            }
            else
            {
                /* NAK received, generate stop condition and give a callback */
                SMB0_REGS->SMB_WCTRL = I2C0_STOP_CONDITION;
                i2c0Obj.state = I2C_MASTER_STATE_IDLE;

                i2c0Obj.error = I2C_ERROR_NACK;

                /* Write transfer complete. Give callback. */
                if (i2c0Obj.callback != NULL)
                {
                    i2c0Obj.callback(i2c0Obj.context);
                }
            }
            break;

        case I2C_MASTER_STATE_REPEATED_START:
            if ((SMB0_REGS->SMB_RSTS & SMB_RSTS_LRB_AD0_Msk) == 0U)
            {
                SMB0_REGS->SMB_I2CDATA = (uint8_t)((i2c0Obj.address << 1U) | I2C_TRANSFER_TYPE_READ);
                SMB0_REGS->SMB_WCTRL = I2C0_REPEATED_START_CONDITION;
                i2c0Obj.state = I2C_MASTER_STATE_ADDR_ACK;
            }
            else
            {
                /* NAK received, generate stop condition and give a callback */
                SMB0_REGS->SMB_WCTRL = I2C0_STOP_CONDITION;
                i2c0Obj.state = I2C_MASTER_STATE_IDLE;

                i2c0Obj.error = I2C_ERROR_NACK;

                /* Write transfer complete. Give callback. */
                if (i2c0Obj.callback != NULL)
                {
                    i2c0Obj.callback(i2c0Obj.context);
                }
            }
            break;

        case I2C_MASTER_STATE_ADDR_ACK:
            /* Enter here for read only transfers*/
            /* Repeated start + Slave addr sent. Check if slave ack'd the address byte. */
            if ((SMB0_REGS->SMB_RSTS & SMB_RSTS_LRB_AD0_Msk) == 0U)
            {
                /* If only one byte needs to be read, then de-assert ACK bit in preparation for the last byte */
                if ((i2c0Obj.readSize - i2c0Obj.readCount) == 1U)
                {
                    SMB0_REGS->SMB_WCTRL = I2C0_NAK;
                }
                i2c0Obj.state = I2C_MASTER_STATE_RECEIVE;
                uint32_t dummy = SMB0_REGS->SMB_I2CDATA;       //dummy read of data corresponding to the address byte sent.
                dummy = dummy;
            }
            else
            {
                /* NAK received, generate stop condition and give a callback */
                SMB0_REGS->SMB_WCTRL = I2C0_STOP_CONDITION;
                i2c0Obj.state = I2C_MASTER_STATE_IDLE;

                i2c0Obj.error = I2C_ERROR_NACK;

                /* Write transfer complete. Give callback. */
                if (i2c0Obj.callback != NULL)
                {
                    i2c0Obj.callback(i2c0Obj.context);
                }
            }
            break;

        case I2C_MASTER_STATE_RECEIVE:

            if ((i2c0Obj.readSize - i2c0Obj.readCount) > 2U)
            {
                i2c0Obj.readBuffer[i2c0Obj.readCount] = SMB0_REGS->SMB_I2CDATA;
                i2c0Obj.readCount++;
            }
            else if ((i2c0Obj.readSize - i2c0Obj.readCount) == 2U)
            {
                /* De-assert ACK bit in preparation for the last byte */
                SMB0_REGS->SMB_WCTRL = I2C0_NAK;
                i2c0Obj.readBuffer[i2c0Obj.readCount] = SMB0_REGS->SMB_I2CDATA;
                i2c0Obj.readCount++;
            }
            else if ((i2c0Obj.readSize - i2c0Obj.readCount) == 1U)
            {
                SMB0_REGS->SMB_WCTRL = I2C0_STOP_CONDITION;
                i2c0Obj.readBuffer[i2c0Obj.readCount] = SMB0_REGS->SMB_I2CDATA;
                i2c0Obj.readCount++;
                i2c0Obj.state = I2C_MASTER_STATE_IDLE;
                if (i2c0Obj.callback != NULL)
                {
                    i2c0Obj.callback(i2c0Obj.context);
                }
            }
            else
            {
                /* Do nothing */
            }
            break;

        case I2C_MASTER_STATE_TRANSFER_ABORT:
            /* If read is in progress, then read one more byte and generate a NAK. Then generate a STOP condition. */
            if ((i2c0Obj.writeCount == i2c0Obj.writeSize) && ((i2c0Obj.readSize - i2c0Obj.readCount) >= 2U))
            {
                /* De-assert ACK bit to 0 in preparation for the last byte */
                SMB0_REGS->SMB_WCTRL = I2C0_NAK;
                uint32_t dummy = SMB0_REGS->SMB_I2CDATA;       //dummy read of data corresponding to the address
                dummy = dummy;
                i2c0Obj.readCount--;
            }
            else
            {
                /* firmware enters here, if a write was in progress or a NAK has been sent for the last read byte */
                SMB0_REGS->SMB_WCTRL = I2C0_STOP_CONDITION;
                i2c0Obj.state = I2C_MASTER_STATE_IDLE;

                /* Write transfer complete. Give callback. */
                if (i2c0Obj.callback != NULL)
                {
                    i2c0Obj.callback(i2c0Obj.context);
                }
            }
            break;

        default:
            /* Do nothing */
            break;
    }

}

bool I2C0_Read(uint16_t address, uint8_t* rdata, size_t rlength)
{
    /* State machine must be idle and I2C module should not have detected a start bit on the bus */
    if(i2c0Obj.state == I2C_MASTER_STATE_IDLE)
    {
        while((SMB0_REGS->SMB_RSTS & SMB_RSTS_NBB_Msk) == 0U)
        {
            /* Wait for the bus to become idle */
        }

        i2c0Obj.address             = address;
        i2c0Obj.readBuffer          = rdata;
        i2c0Obj.readSize            = rlength;
        i2c0Obj.writeBuffer         = NULL;
        i2c0Obj.writeSize           = 0U;
        i2c0Obj.writeCount          = 0U;
        i2c0Obj.readCount           = 0U;
        i2c0Obj.error               = I2C_ERROR_NONE;
        i2c0Obj.state               = I2C_MASTER_STATE_ADDR_ACK;

        /* Disable MDONE interrupt */
        SMB0_REGS->SMB_CFG[0]                 &= ~SMB_CFG_ENMI_Msk;

        SMB0_REGS->SMB_I2CDATA                 = (uint8_t)((address << 1U) | I2C_TRANSFER_TYPE_READ);
        SMB0_REGS->SMB_WCTRL                   = I2C0_START_CONDITION;
        return true;
    }
    else
    {
        return false;
    }
}


bool I2C0_Write(uint16_t address, uint8_t* wdata, size_t wlength)
{
    /* State machine must be idle and I2C module should not have detected a start bit on the bus */
    if(i2c0Obj.state == I2C_MASTER_STATE_IDLE)
    {
        while((SMB0_REGS->SMB_RSTS & SMB_RSTS_NBB_Msk) == 0U)
        {
            /* Wait for the bus to become idle */
        }


        i2c0Obj.address             = address;
        i2c0Obj.readBuffer          = NULL;
        i2c0Obj.readSize            = 0U;
        i2c0Obj.writeBuffer         = wdata;
        i2c0Obj.writeSize           = wlength;
        i2c0Obj.writeCount          = 0U;
        i2c0Obj.readCount           = 0U;
        i2c0Obj.error               = I2C_ERROR_NONE;
        i2c0Obj.state               = I2C_MASTER_STATE_TRANSMIT;

        /* Disable MDONE interrupt */
        SMB0_REGS->SMB_CFG[0]                 &= ~SMB_CFG_ENMI_Msk;

        SMB0_REGS->SMB_I2CDATA                 = (uint8_t)((address << 1U) | I2C_TRANSFER_TYPE_WRITE);
        SMB0_REGS->SMB_WCTRL                   = I2C0_START_CONDITION;
        return true;
    }
    else
    {
        return false;
    }
}

bool I2C0_WriteRead(uint16_t address, uint8_t* wdata, size_t wlength, uint8_t* rdata, size_t rlength)
{
    if(i2c0Obj.state == I2C_MASTER_STATE_IDLE)
    {
        while((SMB0_REGS->SMB_RSTS & SMB_RSTS_NBB_Msk) == 0U)
        {
            /* Wait for the bus to become idle */
        }

        i2c0Obj.address             = address;
        i2c0Obj.readBuffer          = rdata;
        i2c0Obj.readSize            = rlength;
        i2c0Obj.writeBuffer         = wdata;
        i2c0Obj.writeSize           = wlength;
        i2c0Obj.writeCount          = 0U;
        i2c0Obj.readCount           = 0U;
        i2c0Obj.error               = I2C_ERROR_NONE;
        i2c0Obj.state               = I2C_MASTER_STATE_TRANSMIT;

        /* Disable MDONE interrupt */
        SMB0_REGS->SMB_CFG[0]                 &= ~SMB_CFG_ENMI_Msk;

        SMB0_REGS->SMB_I2CDATA                 = (uint8_t)((address << 1U) | I2C_TRANSFER_TYPE_WRITE);
        SMB0_REGS->SMB_WCTRL                   = I2C0_START_CONDITION;
        return true;
    }
    else
    {
        return false;
    }
}

void I2C0_CallbackRegister(I2C_CALLBACK callback, uintptr_t contextHandle)
{
    if (callback == NULL)
    {
        return;
    }

    i2c0Obj.callback = callback;
    i2c0Obj.context = contextHandle;
}

bool I2C0_IsBusy(void)
{
    if((i2c0Obj.state != I2C_MASTER_STATE_IDLE) || (SMB0_REGS->SMB_RSTS & SMB_RSTS_NBB_Msk) == 0U)
    {
        return true;
    }
    else
    {
        return false;
    }
}

I2C_ERROR I2C0_ErrorGet(void)
{
    I2C_ERROR error;

    error = i2c0Obj.error;
    i2c0Obj.error = I2C_ERROR_NONE;

    return error;
}

bool I2C0_TransferSetup(I2C_TRANSFER_SETUP* setup, uint32_t srcClkFreq )
{
    uint32_t high_low_period;
    uint32_t i2cBusFreq;
    uint32_t timingValuesIndex = 0U;
    float temp;

    if ((setup == NULL) || (i2c0Obj.state != I2C_MASTER_STATE_IDLE))
    {
        return false;
    }

    i2cBusFreq = setup->clkSpeed;

    /* Maximum I2C clock speed cannot be greater than 1 MHz */
    if (i2cBusFreq > 1000000U)
    {
        return false;
    }

    if( srcClkFreq == 0U)
    {
        srcClkFreq = 16000000UL;
    }

    temp = ((((float)srcClkFreq/(float)i2cBusFreq)/2.0f) - 1.0f);

    high_low_period = (uint32_t)temp;

    /* high/low baud period value cannot be greater than 255 */
    if (high_low_period > 255U)
    {
        high_low_period = 255U;
    }

    /* Clear the ESO bit(Serial output) before changing the baud value */
    SMB0_REGS->SMB_WCTRL &= ~SMB_WCTRL_ESO_Msk;

    SMB0_REGS->SMB_BUSCLK = SMB_BUSCLK_HIGH_PER(high_low_period) | SMB_BUSCLK_LOW_PER(high_low_period);

    if (i2cBusFreq < 250000U)
    {
        timingValuesIndex = 0U;
    }
    else if (i2cBusFreq < 750000U)
    {
        timingValuesIndex = 1U;
    }
    else
    {
        timingValuesIndex  = 2U;
    }

    /* Repeated start hold time setup */
    SMB0_REGS->SMB_RSHTM = I2C_SMB_RecommendedValues[2][timingValuesIndex];

    /* Data timing register */
    SMB0_REGS->SMB_DATATM = I2C_SMB_RecommendedValues[1][timingValuesIndex];

    /* Timeout scaling register setup */
    SMB0_REGS->SMB_TMOUTSC = I2C_SMB_RecommendedValues[4][timingValuesIndex];

    /* Set the ESO bit (Serial output) after changing the baud value */
    SMB0_REGS->SMB_WCTRL |= SMB_WCTRL_ESO_Msk;

    return true;
}

void I2C0_TransferAbort( void )
{
    i2c0Obj.error = I2C_ERROR_NONE;

    /* Abort transfer */
    i2c0Obj.state = I2C_MASTER_STATE_TRANSFER_ABORT;
}



static void I2C0_MASTER_InterruptHandler(void)
{
    I2C0_TransferSM();
}

void I2CSMB0_InterruptHandler(void)
{
    if (ECIA_GIRQResultGet(ECIA_DIR_INT_SRC_I2CSMB0))
    {
         I2C0_MASTER_InterruptHandler();
         
        ECIA_GIRQSourceClear(ECIA_DIR_INT_SRC_I2CSMB0);
    }
}