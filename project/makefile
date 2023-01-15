
#/bin/bash


## Makefile for PROJECT.



directory_OSAL= C:\Users\Margarita\Desktop\OSAL\\


## SDK PATHS ##

pdk_dir= G:/ti_13/pdk_am335x_1_0_13/packages/	

## TOOLS PATHS ##

compiler=${toolchain_dir}\arm-none-eabi-gcc.exe

linker=${toolchain_dir}\arm-none-eabi-ld.exe

ar=${toolchain_dir}\arm-none-eabi-ar.exe

toolchain_dir=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\bin

bios_dir=G:\ti_13\bios_6_73_01_01\packages

custom=C:\ti\gcc-arm-none-eabi-7-2018-q2-update\lib\gcc\arm-none-eabi\7.3.1\\

xdc_runtime=G:\ti_13\xdctools_3_50_08_24_core\packages
dump_assembly=${toolchain_dir}\arm-none-eabi-objdump.exe -D ${directory_OSAL}project\${output_file}.out > assembly.txt

## FLAGS ##

compiler_flag= -c 
flags= -mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -marm -Dgcc -D__ARM_ARCH_7A__ -DSOC_AM335x -Dam3359 -mfpu=neon
linker_flags = -specs=nosys.specs -specs=nano.specs --nostdlib
crt0=${toolchain_libc}\\crt0.o 


## LIBRARIES PATHS ##

toolchain_libc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\arm-none-eabi\lib
toolchain_lib_gcc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\lib\gcc\arm-none-eabi\6.3.1
library_dir=${directory_OSAL}flash\


## RECEIPES DEFINITIONS ##

##### OBJECTS NAME ######

output_file=project

##### LIBRARY NAMES #####

OSAL_LIB=OSAL_LIBRARY

eMMC_LIB=eMMC

## RECEIPES PATHS ##

output=${directory_OSAL}output\\

## POST BUILD ##


make_bin=${toolchain_dir}\arm-none-eabi-objcopy.exe -S -O binary ${directory_OSAL}project\${output_file}.out ${directory_OSAL}project\${output_file}.bin 

all: project.o
	
project.o:
	-cls

	echo
	echo ************   Compiling PROJECT now!!   ****************\n
	echo Building library: "eMMC" 
	echo Invoking: GNU Compiler	

	${compiler} ${directory_OSAL}project\\*.c ${compiler_flag} ${flags} -I${directory_OSAL}\INCLUDE\ -I${pdk_dir} 
	${compiler} ${directory_OSAL}project\\*.S ${compiler_flag} ${flags} -I${directory_OSAL}\INCLUDE\ -I${pdk_dir} 
	echo ************   Compiling PROJECT now!!   ****************\n
## LINKING OBJECT FILES ##

	${linker} ${linker_flags} -T ${directory_OSAL}project\\am335x_ddr.lds ${directory_OSAL}\\project\\debug\\*.o  ${custom}crtn.o -L  ${toolchain_libc}\   -lc_nano -lnosys \
	-L ${toolchain_lib_gcc}\  -L ${library_dir} -l${OSAL_LIB} -l${eMMC_LIB}  -v -o ${directory_OSAL}project\${output_file}.out

	${dump_assembly}

	${make_bin}


	
## CLEANING UP ##



clean:

  
	del ${directory_OSAL}project\*.a
	del ${directory_OSAL}project\*.o
	del ${directory_OSAL}project\*.out
	del ${directory_OSAL}project\*.bin