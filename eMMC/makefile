
#/bin/bash
#\bin\bash

## Makefile for building the eMMC library for BeagleBone Black device.


SHELL = cmd.exe




directory_OSAL= C:\Users\Margarita\Desktop\OSAL\\


## SDK PATHS ##

pdk_dir= G:/ti_13/pdk_am335x_1_0_13/packages/	

## TOOLS PATHS ##

compiler=${toolchain_dir}\arm-none-eabi-gcc.exe

linker=${toolchain_dir}\arm-none-eabi-ld.exe

ar=${toolchain_dir}\arm-none-eabi-ar.exe

toolchain_dir=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\bin


## FLAGS ##

compiler_flag= -c 
flags= -mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -marm -Dgcc -D__ARM_ARCH_7A__ -DSOC_AM335x -Dam3359 -mfpu=neon
ar_flags = -rcs


## LIBRARIES PATHS ##

toolchain_libc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\arm-none-eabi\lib
toolchain_lib_gcc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\lib\gcc\arm-none-eabi\6.3.1
library_dir=${directory_OSAL}flash\


## RECEIPES DEFINITIONS ##

##### OBJECTS NAME ######

output_file=eMMC

##### LIBRARY NAMES #####

OSAL_LIB=OSAL_LIBRARY



## RECEIPES PATHS ##

output=${directory_OSAL}output\\



all: eMMC.o
	
eMMC.o:
	-cls

	echo
	echo ************   Compiling eMMC now!!   ****************\n
	echo Building library: "eMMC" 
	echo Invoking: GNU Compiler	

	${compiler} ${directory_OSAL}eMMC\src\\*.c ${compiler_flag} ${flags} -I${directory_OSAL}eMMC\INCLUDE\ -I${pdk_dir}

	echo ************   Compiling eMMC now!!   ****************\n
## LINKING OBJECT FILES ##

	${linker} ${linker_flags} ${directory_OSAL}eMMC\\*.o -L  ${toolchain_libc}\ -lc \
	-L ${toolchain_lib_gcc}\ -lgcc -L ${library_dir} -l${OSAL_LIB} -v -o ${directory_OSAL}eMMC\${output_file}.out

## INVOKE AR ##

	${ar} ${ar_flags} lib${output_file}.a ${directory_OSAL}eMMC\\*.o 

	move lib${output_file}.a ${directory_OSAL}flash
	
## CLEANING UP ##

	move ${directory_OSAL}eMMC\\*.o  ${directory_OSAL}eMMC\\output
	move ${directory_OSAL}eMMC\\*.out ${directory_OSAL}eMMC\\output

clean:

	del ${directory_OSAL}eMMC\\*.o
	del ${directory_OSAL}eMMC\\*.out
	del ${directory_OSAL}eMMC\\output\\*.o
	del ${directory_OSAL}eMMC\\output\\*.out
	del ${directory_OSAL}flash\\lib${output_file}.a
  

	
	