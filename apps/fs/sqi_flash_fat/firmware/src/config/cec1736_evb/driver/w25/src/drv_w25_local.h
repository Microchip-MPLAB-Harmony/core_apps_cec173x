/*******************************************************************************
  W25 Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_w25_local.h

  Summary:
    W25 driver local declarations and definitions

  Description:
    This file contains the W25 driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _DRV_W25_LOCAL_H
#define _DRV_W25_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include <string.h>
#include "configuration.h"
#include "driver/w25/drv_w25.h"

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* W25 Command set

  Summary:
    Enumeration listing the W25 commands.

  Description:
    This enumeration defines the commands used to interact with the W25
    series of devices.

  Remarks:
    None
*/

typedef enum
{
    /* Reset enable command. */
    W25_CMD_FLASH_RESET_ENABLE      = 0x66,

    /* Command to reset the flash. */
    W25_CMD_FLASH_RESET             = 0x99,

    /* Command to read JEDEC-ID of the flash device. */
    W25_CMD_JEDEC_ID_READ           = 0x9F,

    /* Command to perform Read data */
    W25_CMD_READ_DATA               = 0x03,

    /* Command to perform Fast Read quad IO */
    W25_CMD_FAST_READ_QUAD_IO       = 0xEB,

    /* Write enable command. */
    W25_CMD_WRITE_ENABLE            = 0x06,

    /* Write disable command. */
    W25_CMD_WRITE_DISABLE           = 0x04,

    /* Page Program command. */
    W25_CMD_PAGE_PROGRAM            = 0x02,

    /* Quad Input Page Program command. */
    W25_CMD_QUAD_INPUT_PAGE_PROGRAM = 0x32,

    /* Command to read the Flash status register. */
    W25_CMD_READ_STATUS_REG         = 0x05,

    /* Command to read the Flash configuration register. */
    W25_CMD_READ_CONFIG_REG         = 0x35,

    /* Command to perform sector erase */
    W25_CMD_SECTOR_ERASE            = 0x20,

    /* Command to perform Block erase */
    W25_CMD_BLOCK_ERASE_64K         = 0xD8,

    /* Command to perform Chip erase */
    W25_CMD_CHIP_ERASE              = 0xC7,

    /* Command to unlock the flash device. */
    W25_CMD_UNPROTECT_GLOBAL        = 0x98

} W25_CMD;

// *****************************************************************************
/* W25 Driver operations.

  Summary:
    Enumeration listing the W25 driver operations.

  Description:
    This enumeration defines the possible W25 driver operations.

  Remarks:
    None
*/

typedef enum
{
    /* Request is a command operation */
    DRV_W25_OPERATION_TYPE_CMD = 0,

    /* Request is read operation. */
    DRV_W25_OPERATION_TYPE_READ,

    /* Request is write operation. */
    DRV_W25_OPERATION_TYPE_WRITE,

    /* Request is erase operation. */
    DRV_W25_OPERATION_TYPE_ERASE,

} DRV_W25_OPERATION_TYPE;

/**************************************
 * W25 Driver Hardware Instance Object
 **************************************/
typedef struct
{
    /* Flag to indicate in use  */
    bool inUse;

    /* Flag to indicate status of transfer */
    volatile bool isTransferDone;

    /* The status of the driver */
    SYS_STATUS status;

    /* Intent of opening the driver */
    DRV_IO_INTENT ioIntent;

    /* Indicates the number of clients that have opened this driver */
    size_t nClients;

    /* Current Operation */
    DRV_W25_OPERATION_TYPE curOpType;

    /* PLIB API list that will be used by the driver to access the hardware */
    const DRV_W25_PLIB_INTERFACE *w25Plib;

} DRV_W25_OBJECT;



#endif //#ifndef _DRV_W25_LOCAL_H

/*******************************************************************************
 End of File
*/

