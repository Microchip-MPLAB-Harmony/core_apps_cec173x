/*******************************************************************************
  UART0 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart0.c

  Summary:
    UART0 PLIB Implementation File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#include "interrupts.h"
#include "plib_uart0.h"
#include "../ecia/plib_ecia.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART0 Implementation
// *****************************************************************************
// *****************************************************************************

volatile static UART_OBJECT uart0Obj;

void UART0_Initialize( void )
{
    UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_DLAB_Msk;
    UART0_REGS->DLAB.UART_BAUDRT_MSB = 0x0;
    UART0_REGS->DLAB.UART_BAUDRT_LSB = 0x1;
    UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;
    UART0_REGS->DATA.UART_LCR = 0x3;

    UART0_REGS->DATA.UART_MCR = UART_DATA_MCR_OUT2_Msk;
    /* Initialize instance object */
    uart0Obj.rxBuffer = NULL;
    uart0Obj.rxSize = 0;
    uart0Obj.rxProcessedSize = 0;
    uart0Obj.rxBusyStatus = false;
    uart0Obj.rxCallback = NULL;
    uart0Obj.txBuffer = NULL;
    uart0Obj.txSize = 0;
    uart0Obj.txProcessedSize = 0;
    uart0Obj.txBusyStatus = false;
    uart0Obj.txCallback = NULL;
    uart0Obj.errors = UART_ERROR_NONE;

    /* Turn ON UART0 */
    UART0_REGS->DATA.UART_ACTIVATE = 0x01;
}

bool UART0_SerialSetup(UART_SERIAL_SETUP* setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    uint32_t baud_clk_src = 1843200;
    uint32_t baud_div;

    if (uart0Obj.rxBusyStatus == true)
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }
    if (uart0Obj.txBusyStatus == true)
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }

    if (setup != NULL)
    {
        baud = setup->baudRate;

        /* Set DLAB = 1 */
        UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_DLAB_Msk;

        if ((UART0_REGS->DLAB.UART_BAUDRT_MSB & 0x80U) != 0U)
        {
            baud_clk_src = 48000000;
        }

        baud_div = (baud_clk_src >> 4)/baud;

        if ((baud_div < 1U) || (baud_div > 32767U))
        {
            /* Set DLAB = 0 */
            UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;

            return status;
        }

        UART0_REGS->DLAB.UART_BAUDRT_LSB = (uint8_t)baud_div;
        UART0_REGS->DLAB.UART_BAUDRT_MSB |= (uint8_t)((baud_div & 0x7F00U) >> 8U);

        /* Set DLAB = 0 */
        UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;

        UART0_REGS->DATA.UART_LCR = (UART0_REGS->DATA.UART_LCR & ~(UART_DATA_LCR_PAR_SEL_Msk | UART_DATA_LCR_STOP_BITS_Msk | UART_DATA_LCR_WORD_LEN_Msk)) | ((uint8_t)setup->parity | (uint8_t)setup->stopBits | (uint8_t)setup->dataWidth);

        if (setup->parity == UART_PARITY_NONE)
        {
            UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_EN_PAR_Msk;
        }
        else
        {
            UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_EN_PAR_Msk;
        }

        status = true;
    }

    return status;
}

bool UART0_Read(void* buffer, const size_t size )
{
    bool status = false;
    uint8_t lsr;
    uint8_t* pRxBuffer = (uint8_t* )buffer;

    if(pRxBuffer != NULL)
    {
        /* Check if receive request is in progress */
        if(uart0Obj.rxBusyStatus == false)
        {
            /* Clear error flags that may have been received when no active request was pending */
            lsr = UART0_REGS->DATA.UART_LSR;
            (void)lsr;

            uart0Obj.rxBuffer = pRxBuffer;
            uart0Obj.rxSize = size;
            uart0Obj.rxProcessedSize = 0;
            uart0Obj.rxBusyStatus = true;
            uart0Obj.errors = UART_ERROR_NONE;

            status = true;

            /* Enable UART0 Receive and Line Error Interrupt */
            UART0_REGS->DATA.UART_IEN |= (UART_DATA_IEN_ERDAI_Msk | UART_DATA_IEN_ELSI_Msk);
        }
    }

    return status;
}

bool UART0_Write( void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* pTxBuffer = (uint8_t*)buffer;

    if(pTxBuffer != NULL)
    {
        /* Check if transmit request is in progress */
        if(uart0Obj.txBusyStatus == false)
        {
            uart0Obj.txBuffer = pTxBuffer;
            uart0Obj.txSize = size;
            uart0Obj.txProcessedSize = 0;
            uart0Obj.txBusyStatus = true;
            status = true;

            size_t txProcessedSize = uart0Obj.txProcessedSize;
            size_t txSize = uart0Obj.txSize;

            /* Initiate the transfer by writing as many bytes as we can */
            while(((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_TRANS_EMPTY_Msk) != 0U) && (txSize > txProcessedSize) )
            {
                UART0_REGS->DATA.UART_TX_DAT = pTxBuffer[txProcessedSize];
                txProcessedSize++;
            }

            uart0Obj.txProcessedSize = txProcessedSize;

            /* Enable UART0 Transmit holding register empty Interrupt */
            UART0_REGS->DATA.UART_IEN |= (UART_DATA_IEN_ETHREI_Msk);
        }
    }

    return status;
}

UART_ERROR UART0_ErrorGet( void )
{
    UART_ERROR errors = uart0Obj.errors;

    uart0Obj.errors = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}


bool UART0_TransmitComplete( void )
{
    bool transmitComplete = false;

    if ((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_TRANS_ERR_Msk) != 0U)
    {
        transmitComplete = true;
    }

    return transmitComplete;
}

void UART0_ReadCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart0Obj.rxCallback = callback;

    uart0Obj.rxContext = context;
}

bool UART0_ReadIsBusy( void )
{
    return uart0Obj.rxBusyStatus;
}

size_t UART0_ReadCountGet( void )
{
    return uart0Obj.rxProcessedSize;
}

bool UART0_ReadAbort(void)
{
    if (uart0Obj.rxBusyStatus == true)
    {
        /* Disable the Receive and Line Error interrupt */
        UART0_REGS->DATA.UART_IEN &= ~(UART_DATA_IEN_ERDAI_Msk | UART_DATA_IEN_ELSI_Msk);

        uart0Obj.rxBusyStatus = false;

        /* If required, application should read the num bytes processed prior to calling the read abort API */
        uart0Obj.rxSize = 0;
        uart0Obj.rxProcessedSize = 0;
    }

    return true;
}

void UART0_WriteCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart0Obj.txCallback = callback;

    uart0Obj.txContext = context;
}

bool UART0_WriteIsBusy( void )
{
    return uart0Obj.txBusyStatus;
}

size_t UART0_WriteCountGet( void )
{
    return uart0Obj.txProcessedSize;
}

static void __attribute__((used)) UART0_ERROR_InterruptHandler (void)
{
    uint8_t lsr;
    bool rxBusyStatus = uart0Obj.rxBusyStatus;

    lsr = UART0_REGS->DATA.UART_LSR;

    /* Check for overrun, parity and framing errors */
    lsr = (lsr & (UART_DATA_LSR_OVERRUN_Msk | UART_DATA_LSR_PE_Msk | UART_DATA_LSR_FRAME_ERR_Msk));
    uart0Obj.errors = lsr;

    if (((uint32_t)uart0Obj.errors != 0U) && (rxBusyStatus == true))
    {
        uart0Obj.rxBusyStatus = false;

        /* Disable the Receive and Line Error interrupt */
        UART0_REGS->DATA.UART_IEN &= ~(UART_DATA_IEN_ERDAI_Msk | UART_DATA_IEN_ELSI_Msk);

        if(uart0Obj.rxCallback != NULL)
        {
            uintptr_t rxContext = uart0Obj.rxContext;
            uart0Obj.rxCallback(rxContext);
        }
    }
}

static void __attribute__((used)) UART0_RX_InterruptHandler (void)
{
    uint32_t lsr;

    if(uart0Obj.rxBusyStatus == true)
    {
        size_t rxProcessedSize = uart0Obj.rxProcessedSize;
        size_t rxSize = uart0Obj.rxSize;

        do
        {
            lsr = UART0_REGS->DATA.UART_LSR;

            /* Check for overrun, parity and framing errors */
            uart0Obj.errors = (lsr & (UART_DATA_LSR_OVERRUN_Msk | UART_DATA_LSR_PE_Msk | UART_DATA_LSR_FRAME_ERR_Msk));

            if ((uart0Obj.errors == 0U) && ((lsr & UART_DATA_LSR_DATA_READY_Msk) != 0U))
            {
                uart0Obj.rxBuffer[rxProcessedSize] = UART0_REGS->DATA.UART_RX_DAT;
                rxProcessedSize++;
            }
        }while(((uint32_t)uart0Obj.errors == 0U) && ((lsr & UART_DATA_LSR_DATA_READY_Msk) != 0U) && (rxProcessedSize < rxSize));

        uart0Obj.rxProcessedSize = rxProcessedSize;

        /* Check if the buffer is done */
        if(((uint32_t)uart0Obj.errors != 0U) || (rxProcessedSize >= rxSize))
        {
            uart0Obj.rxBusyStatus = false;

            /* Disable the Receive and Line Error interrupt */
            UART0_REGS->DATA.UART_IEN &= ~(UART_DATA_IEN_ERDAI_Msk | UART_DATA_IEN_ELSI_Msk);

            if(uart0Obj.rxCallback != NULL)
            {
                uintptr_t rxContext = uart0Obj.rxContext;
                uart0Obj.rxCallback(rxContext);
            }
        }
    }
}

static void __attribute__((used)) UART0_TX_InterruptHandler (void)
{
    if(uart0Obj.txBusyStatus == true)
    {
        size_t txProcessedSize = uart0Obj.txProcessedSize;

        UART0_REGS->DATA.UART_TX_DAT = uart0Obj.txBuffer[txProcessedSize];
        txProcessedSize++;

        uart0Obj.txProcessedSize = txProcessedSize;

        /* Check if the buffer is done */
        if(txProcessedSize >= uart0Obj.txSize)
        {
            uart0Obj.txBusyStatus = false;

            /* Disable the transmit interrupt, to avoid calling ISR continuously */
            UART0_REGS->DATA.UART_IEN &= ~(UART_DATA_IEN_ETHREI_Msk);

            if(uart0Obj.txCallback != NULL)
            {
                uintptr_t txContext = uart0Obj.txContext;
                uart0Obj.txCallback(txContext);
            }
        }
    }
}

void __attribute__((used)) UART0_InterruptHandler (void)
{
    uint8_t int_id = 0;

    int_id = ((UART0_REGS->DATA.UART_INT_ID & UART_DATA_INT_ID_INTID_Msk) >> UART_DATA_INT_ID_INTID_Pos);

    ECIA_GIRQSourceClear(ECIA_DIR_INT_SRC_UART0);

    /* Interrupt type is Receiver Line Status */
    if (int_id == 0x03U)
    {
        UART0_ERROR_InterruptHandler();
    }
    /* Interrupt type is Received data available */
    else if (int_id == 0x02U)
    {
        UART0_RX_InterruptHandler();
    }
    /* Interrupt type is Transmit holding register empty */
    else if (int_id == 0x01U)
    {
        UART0_TX_InterruptHandler();
    }
    else
    {
        /* Do nothing */
    }
}

