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

static UART_RING_BUFFER_OBJECT uart0Obj;

#define UART0_READ_BUFFER_SIZE      16
#define UART0_RX_INT_DISABLE()      UART0_REGS->DATA.UART_IEN &= ~(UART_DATA_IEN_ERDAI_Msk | UART_DATA_IEN_ELSI_Msk)
#define UART0_RX_INT_ENABLE()       UART0_REGS->DATA.UART_IEN |= (UART_DATA_IEN_ERDAI_Msk | UART_DATA_IEN_ELSI_Msk)

static uint8_t UART0_ReadBuffer[UART0_READ_BUFFER_SIZE];

#define UART0_WRITE_BUFFER_SIZE     512
#define UART0_TX_INT_DISABLE()      UART0_REGS->DATA.UART_IEN &= ~(UART_DATA_IEN_ETHREI_Msk)
#define UART0_TX_INT_ENABLE()       UART0_REGS->DATA.UART_IEN |= (UART_DATA_IEN_ETHREI_Msk)

static uint8_t UART0_WriteBuffer[UART0_WRITE_BUFFER_SIZE];

void UART0_Initialize( void )
{
    /* Set DLAB = 1 */
    UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_DLAB_Msk;
    UART0_REGS->DLAB.UART_BAUDRT_MSB = 0x0;
    UART0_REGS->DLAB.UART_BAUDRT_LSB = 0x1;
    /* Set DLAB = 0 */
    UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;
    UART0_REGS->DATA.UART_LCR = 0x3;
    UART0_REGS->DATA.UART_MCR = UART_DATA_MCR_OUT2_Msk;

    /* Initialize instance object */
    uart0Obj.rdCallback = NULL;
    uart0Obj.rdInIndex = 0;
    uart0Obj.rdOutIndex = 0;
    uart0Obj.isRdNotificationEnabled = false;
    uart0Obj.isRdNotifyPersistently = false;
    uart0Obj.rdThreshold = 0;

    uart0Obj.wrCallback = NULL;
    uart0Obj.wrInIndex = 0;
    uart0Obj.wrOutIndex = 0;
    uart0Obj.isWrNotificationEnabled = false;
    uart0Obj.isWrNotifyPersistently = false;
    uart0Obj.wrThreshold = 0;

    uart0Obj.errors = UART_ERROR_NONE;

    uart0Obj.rdBufferSize = UART0_READ_BUFFER_SIZE;
    uart0Obj.wrBufferSize = UART0_WRITE_BUFFER_SIZE;


    /* Turn ON UART0 */
    UART0_REGS->DATA.UART_ACTIVATE = 0x01;

    UART0_RX_INT_ENABLE();
}

bool UART0_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    uint32_t baud_clk_src = 1843200;
    uint32_t baud_div;

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

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static inline bool UART0_RxPushByte(uint8_t rdByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart0Obj.rdInIndex + 1U;

    if (tempInIndex >= uart0Obj.rdBufferSize)
    {
        tempInIndex = 0;
    }

    if (tempInIndex == uart0Obj.rdOutIndex)
    {
        /* Queue is full - Report it to the application. Application gets a chance to free up space by reading data out from the RX ring buffer */
        if(uart0Obj.rdCallback != NULL)
        {
            uart0Obj.rdCallback(UART_EVENT_READ_BUFFER_FULL, uart0Obj.rdContext);

            /* Read the indices again in case application has freed up space in RX ring buffer */
            tempInIndex = uart0Obj.rdInIndex + 1U;

            if (tempInIndex >= uart0Obj.rdBufferSize)
            {
                tempInIndex = 0;
            }
        }
    }

    /* Attempt to push the data into the ring buffer */
    if (tempInIndex != uart0Obj.rdOutIndex)
    {
        UART0_ReadBuffer[uart0Obj.rdInIndex] = rdByte;

        uart0Obj.rdInIndex = tempInIndex;

        isSuccess = true;
    }
    else
    {
        /* Queue is full. Data will be lost. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART0_ReadNotificationSend(void)
{
    uint32_t nUnreadBytesAvailable;

    if (uart0Obj.isRdNotificationEnabled == true)
    {
        nUnreadBytesAvailable = UART0_ReadCountGet();

        if(uart0Obj.rdCallback != NULL)
        {
            if (uart0Obj.isRdNotifyPersistently == true)
            {
                if (nUnreadBytesAvailable >= uart0Obj.rdThreshold)
                {
                    uart0Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart0Obj.rdContext);
                }
            }
            else
            {
                if (nUnreadBytesAvailable == uart0Obj.rdThreshold)
                {
                    uart0Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart0Obj.rdContext);
                }
            }
        }
    }
}

size_t UART0_Read(uint8_t* pRdBuffer, const size_t size)
{
    size_t nBytesRead = 0;
    uint32_t rdOutIndex = 0;
    uint32_t rdInIndex = 0;

    /* Take a snapshot of indices to avoid creation of critical section */
    rdOutIndex = uart0Obj.rdOutIndex;
    rdInIndex = uart0Obj.rdInIndex;

    while (nBytesRead < size)
    {
        if (rdOutIndex != rdInIndex)
        {
            pRdBuffer[nBytesRead] = UART0_ReadBuffer[rdOutIndex];
            nBytesRead++;
            rdOutIndex++;

            if (rdOutIndex >= uart0Obj.rdBufferSize)
            {
                rdOutIndex = 0;
            }
        }
        else
        {
            /* No more data available in the RX buffer */
            break;
        }
    }

    uart0Obj.rdOutIndex = rdOutIndex;

    return nBytesRead;
}

size_t UART0_ReadCountGet(void)
{
    size_t nUnreadBytesAvailable;
    uint32_t rdInIndex;
    uint32_t rdOutIndex;

    /* Take a snapshot of indices to avoid processing in critical section */
    rdInIndex = uart0Obj.rdInIndex;
    rdOutIndex = uart0Obj.rdOutIndex;

    if ( rdInIndex >=  rdOutIndex)
    {
        nUnreadBytesAvailable =  rdInIndex -  rdOutIndex;
    }
    else
    {
        nUnreadBytesAvailable =  (uart0Obj.rdBufferSize -  rdOutIndex) + rdInIndex;
    }

    return nUnreadBytesAvailable;
}

size_t UART0_ReadFreeBufferCountGet(void)
{
    return (uart0Obj.rdBufferSize - 1U) - UART0_ReadCountGet();
}

size_t UART0_ReadBufferSizeGet(void)
{
    return (uart0Obj.rdBufferSize - 1U);
}

bool UART0_ReadNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart0Obj.isRdNotificationEnabled;

    uart0Obj.isRdNotificationEnabled = isEnabled;

    uart0Obj.isRdNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART0_ReadThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0U)
    {
        uart0Obj.rdThreshold = nBytesThreshold;
    }
}

void UART0_ReadCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart0Obj.rdCallback = callback;

    uart0Obj.rdContext = context;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static bool UART0_TxPullByte(uint8_t* pWrByte)
{
    bool isSuccess = false;
    uint32_t wrOutIndex = uart0Obj.wrOutIndex;
    uint32_t wrInIndex = uart0Obj.wrInIndex;

    if (wrOutIndex != wrInIndex)
    {
        *pWrByte = UART0_WriteBuffer[wrOutIndex];
        wrOutIndex++;

        if (wrOutIndex >= uart0Obj.wrBufferSize)
        {
            wrOutIndex = 0;
        }

        uart0Obj.wrOutIndex = wrOutIndex;

        isSuccess = true;
    }

    return isSuccess;
}

static inline bool UART0_TxPushByte(uint8_t wrByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    uint32_t wrOutIndex = uart0Obj.wrOutIndex;
    uint32_t wrInIndex = uart0Obj.wrInIndex;

    tempInIndex = wrInIndex + 1U;

    if (tempInIndex >= uart0Obj.wrBufferSize)
    {
        tempInIndex = 0;
    }
    if (tempInIndex != wrOutIndex)
    {
        UART0_WriteBuffer[wrInIndex] = wrByte;

        uart0Obj.wrInIndex = tempInIndex;

        isSuccess = true;
    }
    else
    {
        /* Queue is full. Report Error. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART0_WriteNotificationSend(void)
{
    uint32_t nFreeWrBufferCount;

    if (uart0Obj.isWrNotificationEnabled == true)
    {
        nFreeWrBufferCount = UART0_WriteFreeBufferCountGet();

        if(uart0Obj.wrCallback != NULL)
        {
            if (uart0Obj.isWrNotifyPersistently == true)
            {
                if (nFreeWrBufferCount >= uart0Obj.wrThreshold)
                {
                    uart0Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart0Obj.wrContext);
                }
            }
            else
            {
                if (nFreeWrBufferCount == uart0Obj.wrThreshold)
                {
                    uart0Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart0Obj.wrContext);
                }
            }
        }
    }
}

static size_t UART0_WritePendingBytesGet(void)
{
    size_t nPendingTxBytes;

    /* Take a snapshot of indices to avoid processing in critical section */

    uint32_t wrOutIndex = uart0Obj.wrOutIndex;
    uint32_t wrInIndex = uart0Obj.wrInIndex;

    if ( wrInIndex >=  wrOutIndex)
    {
        nPendingTxBytes =  wrInIndex - wrOutIndex;
    }
    else
    {
        nPendingTxBytes =  (uart0Obj.wrBufferSize -  wrOutIndex) + wrInIndex;
    }

    return nPendingTxBytes;
}

size_t UART0_WriteCountGet(void)
{
    size_t nPendingTxBytes;

    nPendingTxBytes = UART0_WritePendingBytesGet();

    return nPendingTxBytes;
}

size_t UART0_Write(uint8_t* pWrBuffer, const size_t size )
{
    size_t nBytesWritten  = 0;

    while (nBytesWritten < size)
    {
        if (UART0_TxPushByte(pWrBuffer[nBytesWritten]) == true)
        {
            nBytesWritten++;
        }
        else
        {
            /* Queue is full, exit the loop */
            break;
        }
    }

    /* Check if any data is pending for transmission */
    if (UART0_WritePendingBytesGet() > 0U)
    {
        /* Enable TX interrupt as data is pending for transmission */
        UART0_TX_INT_ENABLE();
    }

    return nBytesWritten;
}

size_t UART0_WriteFreeBufferCountGet(void)
{
    return (uart0Obj.wrBufferSize - 1U) - UART0_WriteCountGet();
}

size_t UART0_WriteBufferSizeGet(void)
{
    return (uart0Obj.wrBufferSize - 1U);
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

bool UART0_WriteNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart0Obj.isWrNotificationEnabled;

    uart0Obj.isWrNotificationEnabled = isEnabled;

    uart0Obj.isWrNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART0_WriteThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0U)
    {
        uart0Obj.wrThreshold = nBytesThreshold;
    }
}

void UART0_WriteCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart0Obj.wrCallback = callback;

    uart0Obj.wrContext = context;
}

UART_ERROR UART0_ErrorGet( void )
{
    UART_ERROR errors = uart0Obj.errors;

    uart0Obj.errors = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}

static void UART0_ERROR_InterruptHandler (void)
{
    uint8_t lsr;

    lsr = UART0_REGS->DATA.UART_LSR;
    lsr = (lsr & (UART_DATA_LSR_OVERRUN_Msk | UART_DATA_LSR_PE_Msk | UART_DATA_LSR_FRAME_ERR_Msk));

    /* Check for overrun, parity and framing errors */
    uart0Obj.errors = lsr;

    /* Client must call UARTx_ErrorGet() function to clear the errors */
    if( uart0Obj.rdCallback != NULL )
    {
        uart0Obj.rdCallback(UART_EVENT_READ_ERROR, uart0Obj.rdContext);
    }
}

static void UART0_RX_InterruptHandler (void)
{
    if (UART0_RxPushByte( UART0_REGS->DATA.UART_RX_DAT ) == true)
    {
        UART0_ReadNotificationSend();
    }
    else
    {
        /* UART RX buffer is full */
    }
}

static void UART0_TX_InterruptHandler (void)
{
    uint8_t wrByte;

    if (UART0_TxPullByte(&wrByte) == true)
    {
        UART0_REGS->DATA.UART_TX_DAT = wrByte;

        /* Send notification */
        UART0_WriteNotificationSend();
    }
    else
    {
        /* Nothing to transmit. Disable the data register empty interrupt. */
        UART0_TX_INT_DISABLE();
    }
}

void UART0_InterruptHandler (void)
{
    uint8_t int_id = 0;

    int_id = ((UART0_REGS->DLAB.UART_INT_ID & UART_DLAB_INT_ID_INTID_Msk) >> UART_DLAB_INT_ID_INTID_Pos);

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
