/*******************************************************************************
* Copyright (C) 2023 Microchip Technology Inc. and its subsidiaries.
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

/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "definitions.h"
#include <string.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

/* Application's state machine enum */
typedef enum
{
    APP_W25_STATE_DRIVER_SETUP,
	APP_W25_STATE_INITIALIZE,
    APP_W25_STATE_WAIT_MIN_POWER_UP_TIME,
    APP_W25_STATE_RESET,
    APP_W25_STATE_GLOBAL_BLK_PROTECTION_UNLOCK,
    APP_W25_STATE_JEDEC_ID_READ,            
    APP_W25_STATE_SECTOR_ERASE,
    APP_W25_STATE_READ_STATUS,    
    APP_W25_STATE_PAGE_PROGRAM,
    APP_W25_STATE_MEMORY_READ,
    APP_W25_STATE_VERIFY,
    APP_W25_STATE_XFER_SUCCESSFUL,
    APP_W25_STATE_XFER_ERROR,    
    APP_W25_STATE_IDLE,    
} APP_W25_STATES;

/* W25 Flash Commands */
#define APP_W25_CMD_ENABLE_RESET                      0x66
#define APP_W25_CMD_MEMORY_RESET                      0x99
#define APP_W25_CMD_STATUS_REG_READ                   0x05
#define APP_W25_CMD_CONFIG_REG_READ                   0x35
#define APP_W25_CMD_MEMORY_READ                       0x03
#define APP_W25_CMD_MEMORY_HIGH_SPEED_READ            0x0B
#define APP_W25_CMD_ENABLE_WRITE                      0x06
#define APP_W25_CMD_DISABLE_WRITE                     0x04
#define APP_W25_CMD_4KB_SECTOR_ERASE                  0x20
#define APP_W25_CMD_BLOCK_ERASE                       0xD8
#define APP_W25_CMD_CHIP_ERASE                        0xC7
#define APP_W25_CMD_PAGE_PROGRAM                      0x02
#define APP_W25_CMD_JEDEC_ID_READ                     0x9F
#define APP_W25_CMD_GLOBAL_BLOCK_PROTECTION_UNLOCK    0x98

#define APP_W25_STATUS_BIT_BUSY                       (0x01 << 0)
#define APP_W25_STATUS_BIT_WEL                        (0x01 << 1)

#define APP_W25_PAGE_PROGRAM_SIZE_BYTES               256                      

#define APP_W25_MEM_ADDR                              0x10000
#define LED_On()                                      LED5_On()
#define LED_Off()                                     LED5_Off()


typedef struct
{
    APP_W25_STATES              state;
    APP_W25_STATES              nextState;
    uint8_t                     transmitBuffer[APP_W25_PAGE_PROGRAM_SIZE_BYTES + 5];    
    uint8_t                     manufacturerID;
    uint16_t                    deviceID;
    DRV_HANDLE                  drvSPIHandle;
    DRV_SPI_TRANSFER_HANDLE     transferHandle;
    DRV_SPI_TRANSFER_SETUP      setup;
    volatile bool               isTransferDone;
}APP_W25_DATA;


static APP_W25_DATA             appW25Data;
static uint8_t                  writeDataBuffer[APP_W25_PAGE_PROGRAM_SIZE_BYTES];
static uint8_t                  readDataBuffer[APP_W25_PAGE_PROGRAM_SIZE_BYTES + 5];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* This function will be called by SPI PLIB when transfer is completed */
void APP_W25_SPIEventHandler(
    DRV_SPI_TRANSFER_EVENT event, 
    DRV_SPI_TRANSFER_HANDLE transferHandle, 
    uintptr_t context
)
{    
    switch(event)
    {
        case DRV_SPI_TRANSFER_EVENT_COMPLETE:            
            break;
        case DRV_SPI_TRANSFER_EVENT_ERROR:            
        default:
            appW25Data.state = APP_W25_STATE_XFER_ERROR;
            break;
    }

    appW25Data.isTransferDone = true;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************



void APP_W25_Reset(void)
{    
    appW25Data.isTransferDone = false; 
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_ENABLE_RESET;
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, &appW25Data.transferHandle);   
    
    while (appW25Data.isTransferDone == false);  
    
    appW25Data.isTransferDone = false;        
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_MEMORY_RESET;
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, &appW25Data.transferHandle); 
    
    while (appW25Data.isTransferDone == false);  
}

void APP_W25_WriteEnable(void)
{
    appW25Data.isTransferDone = false;    
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_ENABLE_WRITE;
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, &appW25Data.transferHandle);    
    
    while (appW25Data.isTransferDone == false);  
}

void APP_W25_WriteDisable(void)
{
    appW25Data.isTransferDone = false;    
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_DISABLE_WRITE;
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, &appW25Data.transferHandle);    
    
    while (appW25Data.isTransferDone == false);  
}

void APP_W25_SectorErase(uint32_t address)
{       
    APP_W25_WriteEnable();
    
    appW25Data.isTransferDone = false;    
    
    /* The address bits from A11:A0 are don't care and must be Vih or Vil */
    address = address & 0xFFFFF000;
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_4KB_SECTOR_ERASE;
    appW25Data.transmitBuffer[1] = (address >> 16);
    appW25Data.transmitBuffer[2] = (address >> 8);
    appW25Data.transmitBuffer[3] = address;
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 4, &appW25Data.transferHandle);    
    
    while (appW25Data.isTransferDone == false);  
}

void APP_W25_ChipErase(void)
{       
    APP_W25_WriteEnable();
    
    appW25Data.isTransferDone = false;            
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_CHIP_ERASE;    
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, &appW25Data.transferHandle);    
    
    while (appW25Data.isTransferDone == false);  
}

void APP_W25_PageProgram(uint32_t address, uint8_t* pPageData)
{        
    uint32_t i;
    
    APP_W25_WriteEnable();
    
    appW25Data.isTransferDone = false;                   
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_PAGE_PROGRAM;
    appW25Data.transmitBuffer[1] = (address >> 16);
    appW25Data.transmitBuffer[2] = (address >> 8);
    appW25Data.transmitBuffer[3] = address;
    
    for (i = 0; i < APP_W25_PAGE_PROGRAM_SIZE_BYTES; i++)
    {
        appW25Data.transmitBuffer[4 + i] = pPageData[i];
    }
    
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, (4 + APP_W25_PAGE_PROGRAM_SIZE_BYTES), &appW25Data.transferHandle);    
        
    while (appW25Data.isTransferDone == false);  
}

void APP_W25_MemoryRead(uint32_t address, uint8_t* pReadBuffer, uint32_t nBytes, bool isHighSpeedRead)
{                        
    uint8_t nTxBytes;
    
    appW25Data.isTransferDone = false;                  
        
    appW25Data.transmitBuffer[1] = (address >> 16);
    appW25Data.transmitBuffer[2] = (address >> 8);
    appW25Data.transmitBuffer[3] = address;        
    
    if (isHighSpeedRead == true)
    {
        appW25Data.transmitBuffer[0] = APP_W25_CMD_MEMORY_HIGH_SPEED_READ;
        /* For high speed read, perform a dummy write */
        appW25Data.transmitBuffer[4] = 0xFF;  
        nTxBytes = 5;
    }
    else
    {
        appW25Data.transmitBuffer[0] = APP_W25_CMD_MEMORY_READ;
        nTxBytes = 4;
    }
    
    appW25Data.isTransferDone = false;                 
    
    DRV_SPI_WriteReadTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, nTxBytes, pReadBuffer, nBytes + nTxBytes, &appW25Data.transferHandle);
        
    while (appW25Data.isTransferDone == false);  
}

uint8_t APP_W25_StatusRead(void)
{
    uint8_t status;
    appW25Data.isTransferDone = false;    
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_STATUS_REG_READ;
      
    DRV_SPI_WriteReadTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, appW25Data.transmitBuffer, (1+1), &appW25Data.transferHandle);    
        
    while (appW25Data.isTransferDone == false); 
    
    status = appW25Data.transmitBuffer[1];
    
    return status;
}

uint8_t APP_W25_ConfigRegisterRead(void)
{
    uint8_t config_reg;
    
    appW25Data.isTransferDone = false;    
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_CONFIG_REG_READ;
    
    DRV_SPI_WriteReadTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, appW25Data.transmitBuffer, (1+1), &appW25Data.transferHandle);    
        
    while (appW25Data.isTransferDone == false);  
    
    config_reg = appW25Data.transmitBuffer[1];
    
    return config_reg;
}

void APP_W25_JEDEC_ID_Read(uint8_t* manufacturerID, uint16_t* deviceID)
{
    appW25Data.isTransferDone = false;    
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_JEDEC_ID_READ;
         
    DRV_SPI_WriteReadTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, appW25Data.transmitBuffer, (1+3), &appW25Data.transferHandle);    
        
    while (appW25Data.isTransferDone == false);  
    
    *manufacturerID = appW25Data.transmitBuffer[1];
    *deviceID = (appW25Data.transmitBuffer[2] << 8UL) | appW25Data.transmitBuffer[3];    
}

void APP_W25_GlobalWriteProtectionUnlock(void)
{
    APP_W25_WriteEnable();
    
    appW25Data.isTransferDone = false;  
    
    appW25Data.transmitBuffer[0] = APP_W25_CMD_GLOBAL_BLOCK_PROTECTION_UNLOCK;
         
    DRV_SPI_WriteTransferAdd(appW25Data.drvSPIHandle, appW25Data.transmitBuffer, 1, &appW25Data.transferHandle);    
        
    while (appW25Data.isTransferDone == false);          
}

void APP_W25_MinPowerOnDelay(void)
{
    uint32_t i;        
    
    /* Cheap delay. 
     * Based on the CPU frequency, ensure the delay is at-least 100 microseconds. 
     */
    for (i = 0; i < 100000; i++)
    {
        asm("NOP");
    }        
}



void APP_W25_Initialize (void)
{
    uint32_t i;
        
    LED_Off();
    
    appW25Data.state = APP_W25_STATE_INITIALIZE;
    
    /* Fill up the test data */
    for (i = 0; i < APP_W25_PAGE_PROGRAM_SIZE_BYTES; i++)
    {
        writeDataBuffer[i] = i;
    }            
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************


void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appW25Data.state = APP_W25_STATE_DRIVER_SETUP;

}

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************


void APP_Tasks ( void )
{    
    uint8_t status;
        
    /* Check the application's current state. */
    switch (appW25Data.state)
    {
        case APP_W25_STATE_DRIVER_SETUP:

            /* Open the SPI Driver instance 1 */
            appW25Data.drvSPIHandle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appW25Data.drvSPIHandle != DRV_HANDLE_INVALID)
            {   
                appW25Data.setup.baudRateInHz    = 1000000;
                appW25Data.setup.clockPhase      = DRV_SPI_CLOCK_PHASE_VALID_LEADING_EDGE;
                appW25Data.setup.clockPolarity   = DRV_SPI_CLOCK_POLARITY_IDLE_LOW;
                appW25Data.setup.dataBits        = DRV_SPI_DATA_BITS_8;
                appW25Data.setup.chipSelect      = SYS_PORT_PIN_NONE;
                appW25Data.setup.csPolarity      = DRV_SPI_CS_POLARITY_ACTIVE_LOW;

                if(DRV_SPI_TransferSetup(appW25Data.drvSPIHandle, &appW25Data.setup) == true)
                {
                    DRV_SPI_TransferEventHandlerSet(appW25Data.drvSPIHandle, APP_W25_SPIEventHandler, (uintptr_t) 0);
                    appW25Data.state = APP_W25_STATE_INITIALIZE;
                }
                else
                {
                    appW25Data.state = APP_W25_STATE_XFER_ERROR;
                }
            }
            else
            {
                appW25Data.state = APP_W25_STATE_XFER_ERROR;
            }
        break;

        case APP_W25_STATE_INITIALIZE:
            APP_W25_Initialize();                
            appW25Data.state = APP_W25_STATE_WAIT_MIN_POWER_UP_TIME;
            break;

        case APP_W25_STATE_WAIT_MIN_POWER_UP_TIME:
            APP_W25_MinPowerOnDelay();
            appW25Data.state = APP_W25_STATE_RESET;                                 
            break;

        case APP_W25_STATE_RESET:
            APP_W25_Reset();
            appW25Data.state = APP_W25_STATE_GLOBAL_BLK_PROTECTION_UNLOCK;
            break;

        case APP_W25_STATE_GLOBAL_BLK_PROTECTION_UNLOCK:
            APP_W25_GlobalWriteProtectionUnlock();                
            appW25Data.state = APP_W25_STATE_JEDEC_ID_READ;
            break;

        case APP_W25_STATE_JEDEC_ID_READ:
            APP_W25_JEDEC_ID_Read(&appW25Data.manufacturerID, &appW25Data.deviceID);
            appW25Data.state = APP_W25_STATE_SECTOR_ERASE;
            break;                                

        case APP_W25_STATE_SECTOR_ERASE:
            APP_W25_SectorErase(APP_W25_MEM_ADDR);                
            appW25Data.state = APP_W25_STATE_READ_STATUS;
            appW25Data.nextState = APP_W25_STATE_PAGE_PROGRAM;
            break;

        case APP_W25_STATE_READ_STATUS:
            status = APP_W25_StatusRead();
            if ((status & APP_W25_STATUS_BIT_BUSY) == 0)                                             
            {
                appW25Data.state = appW25Data.nextState;
            }
            break;

        case APP_W25_STATE_PAGE_PROGRAM:
            APP_W25_PageProgram(APP_W25_MEM_ADDR, &writeDataBuffer[0]);
            appW25Data.state = APP_W25_STATE_READ_STATUS;
            appW25Data.nextState = APP_W25_STATE_MEMORY_READ;
            break;

        case APP_W25_STATE_MEMORY_READ:
            APP_W25_MemoryRead(APP_W25_MEM_ADDR, readDataBuffer, APP_W25_PAGE_PROGRAM_SIZE_BYTES, false);
            appW25Data.state = APP_W25_STATE_VERIFY;                
            break;

        case APP_W25_STATE_VERIFY:
            if (memcmp(writeDataBuffer, &readDataBuffer[4], APP_W25_PAGE_PROGRAM_SIZE_BYTES) == 0)
            {
                appW25Data.state = APP_W25_STATE_XFER_SUCCESSFUL;
            }
            else
            {
                appW25Data.state = APP_W25_STATE_XFER_ERROR;
            }
            break;

        case APP_W25_STATE_XFER_SUCCESSFUL:
            LED_On();
            appW25Data.state = APP_W25_STATE_IDLE;
            break;

        case APP_W25_STATE_XFER_ERROR:
            LED_Off();
            appW25Data.state = APP_W25_STATE_IDLE;
            break;

        case APP_W25_STATE_IDLE:
            break;

        default:
            break;
    }
}


/*******************************************************************************
 End of File
 */
