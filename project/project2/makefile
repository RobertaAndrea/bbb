
#/bin/bash


## Makefile for PROJECT.

project_name=project2

directory_OSAL= C:\Users\Margarita\Desktop\OSAL

directory_project=${directory_OSAL}\\project\\${project_name}

## SDK PATHS ##

pdk_dir= G:/ti_13/pdk_am335x_1_0_13/packages/	

## TOOLS PATHS ##

compiler=${toolchain_dir}\arm-none-eabi-gcc.exe

linker=${toolchain_dir}\arm-none-eabi-ld.exe

ar=${toolchain_dir}\arm-none-eabi-ar.exe

toolchain_dir=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\bin

starterware_root=G:\starterware_backup\\

starterware_includes_path +=-I${starterware_root}include\\  -I${starterware_root}include\hw\\\ -I${starterware_root}include\armv7a\\  -I${starterware_root}include\armv7a\am335x\\

custom=C:\ti\gcc-arm-none-eabi-7-2018-q2-update\lib\gcc\arm-none-eabi\7.3.1\\

dump_assembly=${toolchain_dir}\arm-none-eabi-objdump.exe -D ${directory_project}\${project_name}.out > assembly.txt

## FLAGS ##

compiler_flag= -c 
flags= -mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -marm -Dgcc -D__ARM_ARCH_7A__ -DSOC_AM335x -Dam3359 -mfpu=neon
linker_flags = -specs=nosys.specs -specs=nano.specs 



## LIBRARIES PATHS ##

toolchain_libc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\arm-none-eabi\lib
toolchain_lib_gcc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\lib\gcc\arm-none-eabi\6.3.1
library_dir=${directory_OSAL}\\LIBRARY


## RECEIPES DEFINITIONS ##

##### OBJECTS NAME ######

output_file=project

##### LIBRARY NAMES #####

OSAL_LIB=OSAL_LIBRARY

eMMC_LIB=eMMC

## RECEIPES PATHS ##

output=${directory_OSAL}output\\

## POST BUILD ##


make_bin=${toolchain_dir}\arm-none-eabi-objcopy.exe -S -O binary ${directory_project}\\${project_name}.out ${directory_project}\\${project_name}.bin 

all: project.o clean_up
	
project.o:
	-cls

	echo
	echo ************   Compiling ${project_name} now!!   ****************\n
	echo Building ${project_name} 
	echo Invoking: GNU Compiler	

	${compiler} ${directory_project}\\*.c ${compiler_flag} ${flags} -I	${directory_project}\INCLUDE\ ${starterware_includes_path}  
	${compiler} ${directory_project}\\*.S ${compiler_flag} ${flags} -I	${directory_project}\INCLUDE\ ${starterware_includes_path}  


	echo ************   Linking ${project_name} now!!   ****************\n
## LINKING OBJECT FILES ##
	echo Building ${project_name} 
	echo Invoking: arm-none-eabi

	${linker} ${linker_flags} -T ${directory_OSAL}\\project\\debug\\am335x_ddr.lds ${directory_project}\\*.o  ${custom}crtn.o -L${custom} -lgcc  -L${toolchain_libc}\\ -lc_nano -lnosys -L${library_dir}\\  -lOSAL_LIBRARY -v -o ${directory_project}\${project_name}.out


	${dump_assembly}

	${make_bin}


	
## CLEANING UP ##

clean_up:
	move ${directory_project}\\*.o  ${directory_project}\\output
	move ${directory_project}\\*.out ${directory_project}\\flash
	move ${directory_project}\\*.bin ${directory_project}\\flash
	move ${directory_project}\\*.txt ${directory_project}\\flash

	echo
	echo ************   Cleaning ${project_name} done!!   ****************
	
	echo Building ${project_name} COMPLETE
		

clean:

  
	-del ${directory_project}\\*.o
	-del ${directory_project}\\*.out
	-del ${directory_project}\\*.a
	-del ${directory_project}\\*.bin
	-del ${directory_project}\\*.txt
	-del ${directory_project}\\output\\*.o
	-del ${directory_project}\\flash\\*.out
	-del ${directory_project}\\flash\\*.txt
	-del ${directory_project}\\flash\\*.bin
	
	