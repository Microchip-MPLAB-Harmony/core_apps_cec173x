#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=null
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=null
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/common/debug/serial.c ../src/config/default/peripheral/ecia/plib_ecia.c ../src/config/default/peripheral/gpio/plib_gpio.c ../src/config/default/peripheral/nvic/plib_nvic.c ../src/config/default/peripheral/pcr/plib_pcr.c ../src/config/default/peripheral/uart/plib_uart0.c ../src/config/default/exceptions.c ../src/config/default/interrupts.c ../src/config/default/initialization.c ../src/hal/gpio/gpio_api.c ../src/hal/interrupt/interrupt_api.c ../src/hal/uart/uart_api.c ../src/kernel/main.c ../src/oem/oem_task1/oem_task1.c ../src/oem/oem_task2/oem_task2.c ../src/oem/oem_task3/oem_task3.c ../src/platform/irqhandler.c ../src/startup/startup_CEC173x.S

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1506558953/serial.o ${OBJECTDIR}/_ext/1865182088/plib_ecia.o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ${OBJECTDIR}/_ext/60177741/plib_pcr.o ${OBJECTDIR}/_ext/1865657120/plib_uart0.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/222663291/gpio_api.o ${OBJECTDIR}/_ext/684516973/interrupt_api.o ${OBJECTDIR}/_ext/222260348/uart_api.o ${OBJECTDIR}/_ext/174097801/main.o ${OBJECTDIR}/_ext/1203168206/oem_task1.o ${OBJECTDIR}/_ext/1203168205/oem_task2.o ${OBJECTDIR}/_ext/1203168204/oem_task3.o ${OBJECTDIR}/_ext/1756295213/irqhandler.o ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1506558953/serial.o.d ${OBJECTDIR}/_ext/1865182088/plib_ecia.o.d ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d ${OBJECTDIR}/_ext/60177741/plib_pcr.o.d ${OBJECTDIR}/_ext/1865657120/plib_uart0.o.d ${OBJECTDIR}/_ext/1171490990/exceptions.o.d ${OBJECTDIR}/_ext/1171490990/interrupts.o.d ${OBJECTDIR}/_ext/1171490990/initialization.o.d ${OBJECTDIR}/_ext/222663291/gpio_api.o.d ${OBJECTDIR}/_ext/684516973/interrupt_api.o.d ${OBJECTDIR}/_ext/222260348/uart_api.o.d ${OBJECTDIR}/_ext/174097801/main.o.d ${OBJECTDIR}/_ext/1203168206/oem_task1.o.d ${OBJECTDIR}/_ext/1203168205/oem_task2.o.d ${OBJECTDIR}/_ext/1203168204/oem_task3.o.d ${OBJECTDIR}/_ext/1756295213/irqhandler.o.d ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1506558953/serial.o ${OBJECTDIR}/_ext/1865182088/plib_ecia.o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ${OBJECTDIR}/_ext/60177741/plib_pcr.o ${OBJECTDIR}/_ext/1865657120/plib_uart0.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/222663291/gpio_api.o ${OBJECTDIR}/_ext/684516973/interrupt_api.o ${OBJECTDIR}/_ext/222260348/uart_api.o ${OBJECTDIR}/_ext/174097801/main.o ${OBJECTDIR}/_ext/1203168206/oem_task1.o ${OBJECTDIR}/_ext/1203168205/oem_task2.o ${OBJECTDIR}/_ext/1203168204/oem_task3.o ${OBJECTDIR}/_ext/1756295213/irqhandler.o ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o

# Source Files
SOURCEFILES=../src/common/debug/serial.c ../src/config/default/peripheral/ecia/plib_ecia.c ../src/config/default/peripheral/gpio/plib_gpio.c ../src/config/default/peripheral/nvic/plib_nvic.c ../src/config/default/peripheral/pcr/plib_pcr.c ../src/config/default/peripheral/uart/plib_uart0.c ../src/config/default/exceptions.c ../src/config/default/interrupts.c ../src/config/default/initialization.c ../src/hal/gpio/gpio_api.c ../src/hal/interrupt/interrupt_api.c ../src/hal/uart/uart_api.c ../src/kernel/main.c ../src/oem/oem_task1/oem_task1.c ../src/oem/oem_task2/oem_task2.c ../src/oem/oem_task3/oem_task3.c ../src/platform/irqhandler.c ../src/startup/startup_CEC173x.S

# Pack Options 
PACK_COMMON_OPTIONS=-I "${CMSIS_DIR}/CMSIS/Core/Include"



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=
MP_LINKER_FILE_OPTION=

# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
