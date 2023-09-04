/*******************************************************************************
  QMSPI0 SPI Peripheral Library Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_qmspi0_spi.c

  Summary
    QMSPI0 SPI mode peripheral library interface.

  Description

  Remarks:

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
#include "interrupts.h"
#include "plib_qmspi0_spi.h"
#include "../ecia/plib_ecia.h"

typedef struct
{
    uint8_t*                pTxBuffer;
    uint8_t*                pRxBuffer;
    size_t                  txSize;
    size_t                  rxSize;
    size_t                  rxCount;
    size_t                  txCount;
    size_t                  rxPending;
    size_t                  txPending;
    uint32_t                txDummyData;
    uint32_t                rxDummyData;
    bool                    transferIsBusy;
    QMSPI_SPI_CALLBACK      callback;
    uintptr_t               context;

} QMSPI_SPI_OBJECT ;


/* Global object to save SPI Exchange related data */
volatile static QMSPI_SPI_OBJECT qmspi0Obj;

// *****************************************************************************
// *****************************************************************************
// QMSPI0 PLIB Interface Routines
// *****************************************************************************
// *****************************************************************************


void QMSPI0_SPI_Initialize(void)
{
    /* Reset the QMSPI Block */
    QMSPI0_REGS->QMSPI_MODE = QMSPI_MODE_SOFT_RESET_Msk;

    QMSPI0_REGS->QMSPI_MODE = QMSPI_MODE_CLK_DIV(8U) | QMSPI_MODE_CHPA_MOSI(0) | QMSPI_MODE_CHPA_MISO(0) | QMSPI_MODE_CS(0);

    /* Activate the QMSPI Block */
    QMSPI0_REGS->QMSPI_MODE |= QMSPI_MODE_ACT_Msk;
}

bool QMSPI0_SPI_TransferSetup (QMSPI_SPI_TRANSFER_SETUP *setup)
{
    uint32_t clock_divide;
    bool setupStatus = false;

    if (setup != NULL)
    {
        clock_divide = 96000000U / setup->clockFrequency;

        if (clock_divide >= 256U)
        {
            clock_divide = 0;
        }

        QMSPI0_REGS->QMSPI_MODE =
        ( (QMSPI0_REGS->QMSPI_MODE & ~(QMSPI_MODE_CLK_DIV_Msk | QMSPI_MODE_CHPA_MOSI_Msk | QMSPI_MODE_CHPA_MISO_Msk | QMSPI_MODE_CPOL_Msk)) | (QMSPI_MODE_CLK_DIV(clock_divide) | (uint32_t)setup->clockPhase | (uint32_t)setup->clockPolarity));

        setupStatus = true;
    }
    return setupStatus;
}

bool QMSPI0_SPI_Write(void* pTransmitData, size_t txSize)
{
    return(QMSPI0_SPI_WriteRead(pTransmitData, txSize, NULL, 0));
}

bool QMSPI0_SPI_Read(void* pReceiveData, size_t rxSize)
{
    return(QMSPI0_SPI_WriteRead(NULL, 0, pReceiveData, rxSize));
}


void QMSPI0_SPI_CallbackRegister(QMSPI_SPI_CALLBACK callback, uintptr_t context)
{
    qmspi0Obj.callback = callback;
    qmspi0Obj.context = context;
}

bool QMSPI0_SPI_IsBusy (void)
{
    bool transferIsBusy = qmspi0Obj.transferIsBusy;

    return transferIsBusy;
}


bool QMSPI0_SPI_IsTransmitterBusy(void)
{
    return ((QMSPI0_REGS->QMSPI_STS & QMSPI_STS_TRANS_ACTIV_Msk) != 0U);
}


bool QMSPI0_SPI_WriteRead (void* pTransmitData, size_t txSize, void* pReceiveData, size_t rxSize)
{
    bool rxAddrInc = false;
    bool txAddrInc = false;
    bool isSuccess = false;
    size_t transferSize;

    /* Verify the request */
    if(((txSize > 0U) && (pTransmitData != NULL)) || ((rxSize > 0U) && (pReceiveData != NULL)))
    {
        if(pTransmitData == NULL)
        {
            txSize = 0U;
        }

        if(pReceiveData == NULL)
        {
            rxSize = 0U;
        }

        qmspi0Obj.txSize = txSize;
        qmspi0Obj.rxSize = rxSize;
        qmspi0Obj.pTxBuffer = pTransmitData;
        qmspi0Obj.pRxBuffer = pReceiveData;
        qmspi0Obj.rxPending = 0;
        qmspi0Obj.txPending = 0;
        qmspi0Obj.txDummyData = 0xFFFFFFFFU;

        if ((rxSize == txSize) || (rxSize == 0U) || (txSize == 0U))
        {
            // Transfer size will be the max of qmspi0Obj.rxSize and qmspi0Obj.txSize
            transferSize = rxSize > txSize ? rxSize : txSize;
        }
        else
        {
            // Transfer size will be min of qmspi0Obj.rxSize and qmspi0Obj.txSize
            transferSize = rxSize > txSize ? txSize : rxSize;

            if (rxSize > txSize)
            {
                qmspi0Obj.rxPending = (rxSize - txSize);
            }
            else
            {
                qmspi0Obj.txPending = (txSize - rxSize);
            }
        }

        rxAddrInc = qmspi0Obj.rxSize == 0U ? false:true;

        txAddrInc = qmspi0Obj.txSize == 0U ? false:true;

        QMSPI0_REGS->QMSPI_MODE |= QMSPI_MODE_LDMA_RXEN_Msk | QMSPI_MODE_LDMA_TXEN_Msk;

        /* Flush out any unread data in SPI DATA Register from the previous transfer */
        QMSPI0_REGS->QMSPI_EXE = QMSPI_EXE_CLR_DAT_BUFF_Msk;

        QMSPI0_REGS->QMSPI_CTRL = QMSPI_CTRL_TRANS_UNITS(0x01) | QMSPI_CTRL_TRANS_LEN(transferSize) | QMSPI_CTRL_TX_TRANS_EN(0x01) | QMSPI_CTRL_TX_DMA_EN(1) | QMSPI_CTRL_RX_TRANS_EN_Msk | QMSPI_CTRL_RX_DMA_EN(1);

        if (qmspi0Obj.txPending == 0U)
        {
            if (qmspi0Obj.rxPending == 0U)
            {
                QMSPI0_REGS->QMSPI_CTRL |= QMSPI_CTRL_CLOSE_TRANS_EN_Msk;
            }
        }

        if (txAddrInc)
        {
            QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXCTRL = QMSPI_LDMA_TXCTRL_CH_EN_Msk | QMSPI_LDMA_TXCTRL_INC_ADDR_EN_Msk;
            QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXSTRT_ADDR = (uint32_t)qmspi0Obj.pTxBuffer;
        }
        else
        {
            QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXCTRL = QMSPI_LDMA_TXCTRL_CH_EN_Msk;
            QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXSTRT_ADDR = (uint32_t)&qmspi0Obj.txDummyData;
        }

        QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TX_LEN = transferSize;

        if (rxAddrInc)
        {
            QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXCTRL = QMSPI_LDMA_RXCTRL_CH_EN_Msk | QMSPI_LDMA_RXCTRL_INC_ADDR_EN_Msk;
            QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXSTRT_ADDR = (uint32_t)qmspi0Obj.pRxBuffer;
        }
        else
        {
            QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXCTRL = QMSPI_LDMA_RXCTRL_CH_EN_Msk;
            QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXSTRT_ADDR = (uint32_t)&qmspi0Obj.rxDummyData;
        }

        qmspi0Obj.transferIsBusy = true;

        QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RX_LEN = transferSize;

        QMSPI0_REGS->QMSPI_IEN = QMSPI_IEN_TRANS_COMPL_EN_Msk;
        QMSPI0_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;

        isSuccess = true;
    }

    return isSuccess;
}

void __attribute__((used)) QMSPI0_InterruptHandler(void)
{
    bool txAddrInc = false;
    bool rxAddrInc = false;
    size_t transferSize;
    size_t txPending = qmspi0Obj.txPending;
    size_t rxPending = qmspi0Obj.rxPending;
    size_t txSize = qmspi0Obj.txSize;
    size_t rxSize = qmspi0Obj.rxSize;

    if (ECIA_GIRQResultGet(ECIA_DIR_INT_SRC_QMSPI0) != 0U)
    {
        ECIA_GIRQSourceClear(ECIA_DIR_INT_SRC_QMSPI0);

        if ((QMSPI0_REGS->QMSPI_STS & QMSPI_STS_TRANS_COMPL_Msk) != 0U)
        {
            QMSPI0_REGS->QMSPI_STS |= QMSPI_STS_TRANS_COMPL_Msk;

            if ( ((QMSPI0_REGS->QMSPI_CTRL & QMSPI_CTRL_DESCR_BUFF_EN_Msk) == 0U) && ((txPending > 0U) || (rxPending > 0U)) )
            {
                txAddrInc = qmspi0Obj.txPending > 0U? true : false;
                rxAddrInc = qmspi0Obj.rxPending > 0U? true : false;

                transferSize = txPending > rxPending? txPending : rxPending;

                QMSPI0_REGS->QMSPI_CTRL = QMSPI_CTRL_TRANS_UNITS(0x01) | QMSPI_CTRL_TRANS_LEN(transferSize) | QMSPI_CTRL_TX_TRANS_EN(0x01) | QMSPI_CTRL_TX_DMA_EN(1) | QMSPI_CTRL_RX_TRANS_EN_Msk | QMSPI_CTRL_RX_DMA_EN(1)  | QMSPI_CTRL_CLOSE_TRANS_EN_Msk ;

                if (txAddrInc)
                {
                    QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXCTRL = QMSPI_LDMA_TXCTRL_CH_EN_Msk | QMSPI_LDMA_TXCTRL_INC_ADDR_EN_Msk;
                    QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXSTRT_ADDR = (uint32_t)&qmspi0Obj.pTxBuffer[txSize - txPending];
                }
                else
                {
                    QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXCTRL = QMSPI_LDMA_TXCTRL_CH_EN_Msk;
                    QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TXSTRT_ADDR = (uint32_t)&qmspi0Obj.txDummyData;
                }

                QMSPI0_REGS->LDMA_TX[0].QMSPI_LDMA_TX_LEN = transferSize;

                if (rxAddrInc)
                {
                    QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXCTRL = QMSPI_LDMA_RXCTRL_CH_EN_Msk | QMSPI_LDMA_RXCTRL_INC_ADDR_EN_Msk;
                    QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXSTRT_ADDR = (uint32_t)&qmspi0Obj.pRxBuffer[rxSize - rxPending];
                }
                else
                {
                    QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXCTRL = QMSPI_LDMA_RXCTRL_CH_EN_Msk;
                    QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RXSTRT_ADDR = (uint32_t)&qmspi0Obj.rxDummyData;
                }

                qmspi0Obj.txPending = qmspi0Obj.rxPending = 0;

                QMSPI0_REGS->LDMA_RX[0].QMSPI_LDMA_RX_LEN = transferSize;

                QMSPI0_REGS->QMSPI_IEN = QMSPI_IEN_TRANS_COMPL_EN_Msk;
                QMSPI0_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
            }
            else
            {
                QMSPI0_REGS->QMSPI_IEN &= ~QMSPI_IEN_TRANS_COMPL_EN_Msk;

                qmspi0Obj.transferIsBusy = false;

                if (qmspi0Obj.callback != NULL)
                {
                    uintptr_t context = qmspi0Obj.context;

                    qmspi0Obj.callback(context);
                }
            }
        }
    }
}







/*******************************************************************************
 End of File
*/
