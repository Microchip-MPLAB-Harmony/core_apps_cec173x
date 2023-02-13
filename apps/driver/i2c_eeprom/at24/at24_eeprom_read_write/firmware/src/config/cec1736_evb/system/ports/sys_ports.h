/*******************************************************************************
  PORTS Service

  Company:
    Microchip Technology Inc.

  File Name:
    sys_ports.h

  Summary:
    PORTS Service Header File

  Description:
    This library provides an interface to control and interact with PORTS
    System Service.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef SYS_PORTS_H
#define SYS_PORTS_H

#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Sys Port

  Summary:
    Identifies the available Port Channels.

  Description:
    This enumeration identifies the available Port Channels.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    SYS_PORT_0 = 0,
    SYS_PORT_1 = 1,
    SYS_PORT_2 = 2,
    SYS_PORT_3 = 3,
    SYS_PORT_4 = 4,
    SYS_PORT_5 = 5,
} SYS_PORT;


// *****************************************************************************
/* Sys Port Pins

  Summary:
    Identifies the available port pins.

  Description:
    This enumeration identifies the available port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    /* GPIO000 pin */
    SYS_PORT_PIN_GPIO000 = 0U,
    /* GPIO002 pin */
    SYS_PORT_PIN_GPIO002 = 2U,
    /* GPIO003 pin */
    SYS_PORT_PIN_GPIO003 = 3U,
    /* GPIO004 pin */
    SYS_PORT_PIN_GPIO004 = 4U,
    /* GPIO012 pin */
    SYS_PORT_PIN_GPIO012 = 10U,
    /* GPIO013 pin */
    SYS_PORT_PIN_GPIO013 = 11U,
    /* GPIO015 pin */
    SYS_PORT_PIN_GPIO015 = 13U,
    /* GPIO016 pin */
    SYS_PORT_PIN_GPIO016 = 14U,
    /* GPIO020 pin */
    SYS_PORT_PIN_GPIO020 = 16U,
    /* GPIO021 pin */
    SYS_PORT_PIN_GPIO021 = 17U,
    /* GPIO022 pin */
    SYS_PORT_PIN_GPIO022 = 18U,
    /* GPIO023 pin */
    SYS_PORT_PIN_GPIO023 = 19U,
    /* GPIO024 pin */
    SYS_PORT_PIN_GPIO024 = 20U,
    /* GPIO026 pin */
    SYS_PORT_PIN_GPIO026 = 22U,
    /* GPIO027 pin */
    SYS_PORT_PIN_GPIO027 = 23U,
    /* GPIO030 pin */
    SYS_PORT_PIN_GPIO030 = 24U,
    /* GPIO031 pin */
    SYS_PORT_PIN_GPIO031 = 25U,
    /* GPIO032 pin */
    SYS_PORT_PIN_GPIO032 = 26U,
    /* GPIO033 pin */
    SYS_PORT_PIN_GPIO033 = 27U,
    /* GPIO034 pin */
    SYS_PORT_PIN_GPIO034 = 28U,
    /* GPIO045 pin */
    SYS_PORT_PIN_GPIO045 = 37U,
    /* GPIO046 pin */
    SYS_PORT_PIN_GPIO046 = 38U,
    /* GPIO047 pin */
    SYS_PORT_PIN_GPIO047 = 39U,
    /* GPIO050 pin */
    SYS_PORT_PIN_GPIO050 = 40U,
    /* GPIO053 pin */
    SYS_PORT_PIN_GPIO053 = 43U,
    /* GPIO055 pin */
    SYS_PORT_PIN_GPIO055 = 45U,
    /* GPIO056 pin */
    SYS_PORT_PIN_GPIO056 = 46U,
    /* GPIO057 pin */
    SYS_PORT_PIN_GPIO057 = 47U,
    /* GPIO063 pin */
    SYS_PORT_PIN_GPIO063 = 51U,
    /* GPIO070 pin */
    SYS_PORT_PIN_GPIO070 = 56U,
    /* GPIO071 pin */
    SYS_PORT_PIN_GPIO071 = 57U,
    /* GPIO104 pin */
    SYS_PORT_PIN_GPIO104 = 68U,
    /* GPIO105 pin */
    SYS_PORT_PIN_GPIO105 = 69U,
    /* GPIO106 pin */
    SYS_PORT_PIN_GPIO106 = 70U,
    /* GPIO107 pin */
    SYS_PORT_PIN_GPIO107 = 71U,
    /* GPIO112 pin */
    SYS_PORT_PIN_GPIO112 = 74U,
    /* GPIO113 pin */
    SYS_PORT_PIN_GPIO113 = 75U,
    /* GPIO120 pin */
    SYS_PORT_PIN_GPIO120 = 80U,
    /* GPIO121 pin */
    SYS_PORT_PIN_GPIO121 = 81U,
    /* GPIO122 pin */
    SYS_PORT_PIN_GPIO122 = 82U,
    /* GPIO123 pin */
    SYS_PORT_PIN_GPIO123 = 83U,
    /* GPIO124 pin */
    SYS_PORT_PIN_GPIO124 = 84U,
    /* GPIO125 pin */
    SYS_PORT_PIN_GPIO125 = 85U,
    /* GPIO126 pin */
    SYS_PORT_PIN_GPIO126 = 86U,
    /* GPIO127 pin */
    SYS_PORT_PIN_GPIO127 = 87U,
    /* GPIO130 pin */
    SYS_PORT_PIN_GPIO130 = 88U,
    /* GPIO131 pin */
    SYS_PORT_PIN_GPIO131 = 89U,
    /* GPIO132 pin */
    SYS_PORT_PIN_GPIO132 = 90U,
    /* GPIO140 pin */
    SYS_PORT_PIN_GPIO140 = 96U,
    /* GPIO143 pin */
    SYS_PORT_PIN_GPIO143 = 99U,
    /* GPIO144 pin */
    SYS_PORT_PIN_GPIO144 = 100U,
    /* GPIO145 pin */
    SYS_PORT_PIN_GPIO145 = 101U,
    /* GPIO146 pin */
    SYS_PORT_PIN_GPIO146 = 102U,
    /* GPIO147 pin */
    SYS_PORT_PIN_GPIO147 = 103U,
    /* GPIO150 pin */
    SYS_PORT_PIN_GPIO150 = 104U,
    /* GPIO156 pin */
    SYS_PORT_PIN_GPIO156 = 110U,
    /* GPIO157 pin */
    SYS_PORT_PIN_GPIO157 = 111U,
    /* GPIO163 pin */
    SYS_PORT_PIN_GPIO163 = 115U,
    /* GPIO165 pin */
    SYS_PORT_PIN_GPIO165 = 117U,
    /* GPIO170 pin */
    SYS_PORT_PIN_GPIO170 = 120U,
    /* GPIO171 pin */
    SYS_PORT_PIN_GPIO171 = 121U,
    /* GPIO200 pin */
    SYS_PORT_PIN_GPIO200 = 128U,
    /* GPIO201 pin */
    SYS_PORT_PIN_GPIO201 = 129U,
    /* GPIO202 pin */
    SYS_PORT_PIN_GPIO202 = 130U,
    /* GPIO203 pin */
    SYS_PORT_PIN_GPIO203 = 131U,
    /* GPIO204 pin */
    SYS_PORT_PIN_GPIO204 = 132U,
    /* GPIO223 pin */
    SYS_PORT_PIN_GPIO223 = 147U,
    /* GPIO224 pin */
    SYS_PORT_PIN_GPIO224 = 148U,
    /* GPIO227 pin */
    SYS_PORT_PIN_GPIO227 = 151U,
    /* GPIO250 pin */
    SYS_PORT_PIN_GPIO250 = 168U,
    /* GPIO253 pin */
    SYS_PORT_PIN_GPIO253 = 171U,

    /* This element should not be used in any of the PORTS APIs.
       It will be used by other modules or application to denote that none of the PORT Pin is used */
    SYS_PORT_PIN_NONE = -1

} SYS_PORT_PIN;


// *****************************************************************************
// *****************************************************************************
// Section: SYS PORT Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void SYS_PORT_PinWrite(SYS_PORT_PIN pin, bool value)

  Summary:
    Writes to the selected pin.

  Description:
    This function writes/drives the "value" on the selected I/O line/pin.

  Precondition:
    Port Initialization must have been done using appropriate Initialize API call.

  Parameters:
    pin       - One of the IO pins from the enum SYS_PORT_PIN
    value     - value to be written on the selected pin:
                true  = set pin to high (1).
                false = clear pin to low (0).

  Returns:
    None.

  Example:
    <code>
    SYS_PORT_PinWrite(SYS_PORT_PIN_PB3, true);
    </code>

  Remarks:
    None.
*/
static inline void SYS_PORT_PinWrite(SYS_PORT_PIN pin, bool value);

// *****************************************************************************
/* Function:
    bool SYS_PORT_PinRead(SYS_PORT_PIN pin)

  Summary:
    Read the selected pin value.

  Description:
    This function reads the selected pin value.
    it reads the value regardless of pin configuration, whether uniquely as an
    input, or driven by the PIO Controller, or driven by peripheral.

  Precondition:
    Reading the I/O line levels requires the clock of the PIO Controller to be
    enabled, otherwise this API reads the levels present on the I/O line at the
    time the clock was disabled.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    Returns the read value of the selected I/O pin.

  Example:
    <code>

    bool value;
    value = SYS_PORT_PinRead(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
       To read the latched value on this pin, SYS_PORT_PinLatchRead API should be used.
*/
static inline bool SYS_PORT_PinRead(SYS_PORT_PIN pin);

// *****************************************************************************
/* Function:
    bool SYS_PORT_PinLatchRead ( SYS_PORT_PIN pin )

  Summary:
    Read the value driven on the selected pin.

  Description:
    This function reads the data driven on the selected I/O line/pin.
    Whatever data is written/driven on I/O line by using any of the PORTS
    APIs, will be read by this API.

  Precondition:
    None.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    Returns the value driven on the selected I/O pin.

  Example:
    <code>

    bool value;
    value = SYS_PORT_PinLatchRead(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
    To read actual pin value, SYS_PORT_PinRead API should be used.
*/
static inline bool SYS_PORT_PinLatchRead(SYS_PORT_PIN pin);

// *****************************************************************************
/* Function:
    void SYS_PORT_PinToggle(SYS_PORT_PIN pin)

  Summary:
    Toggles the selected pin.

  Description:
    This function toggles/inverts the value on the selected I/O line/pin.

  Precondition:
    Port Initialization must have been done using appropriate Initialize API call.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    None.

  Example:
    <code>

    SYS_PORT_PinToggle(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/
static inline void SYS_PORT_PinToggle(SYS_PORT_PIN pin);

// *****************************************************************************
/* Function:
    void SYS_PORT_PinSet(SYS_PORT_PIN pin)

  Summary:
    Sets the selected pin.

  Description:
    This function drives '1' on the selected I/O line/pin.

  Precondition:
    None.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    None.

  Example:
    <code>

    SYS_PORT_PinSet(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/
static inline void SYS_PORT_PinSet(SYS_PORT_PIN pin);

// *****************************************************************************
/* Function:
    void SYS_PORT_PinClear(SYS_PORT_PIN pin)

  Summary:
    Clears the selected pin.

  Description:
    This function drives '0' on the selected I/O line/pin.

  Precondition:
    None.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    None.

  Example:
    <code>

    SYS_PORT_PinClear(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/
static inline void SYS_PORT_PinClear(SYS_PORT_PIN pin);

// *****************************************************************************
/* Function:
    void SYS_PORT_PinInputEnable(SYS_PORT_PIN pin)

  Summary:
    Enables selected IO pin as input.

  Description:
    This function enables selected IO pin as input.

  Precondition:
    None.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    None.

  Example:
    <code>

    SYS_PORT_PinInputEnable(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/
static inline void SYS_PORT_PinInputEnable(SYS_PORT_PIN pin);

// *****************************************************************************
/* Function:
    void SYS_PORT_PinOutputEnable(SYS_PORT_PIN pin)

  Summary:
    Enables selected IO pin as output.

  Description:
    This function enables selected IO pin as output.

  Precondition:
    None.

  Parameters:
    pin - One of the IO pins from the enum SYS_PORT_PIN

  Returns:
    None.

  Example:
    <code>

    SYS_PORT_PinOutputEnable(SYS_PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/
static inline void SYS_PORT_PinOutputEnable(SYS_PORT_PIN pin);


#include "sys_ports_mapping.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // SYS_PORTS_H
