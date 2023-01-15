#/bin/bash

SHELL = cmd.exe



directory_OSAL= C:\Users\Margarita\Desktop\OSAL\\

pdk_dir= G:/ti_13/pdk_am335x_1_0_13/packages/	

toolchain_dir=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\bin

compiler=${toolchain_dir}\arm-none-eabi-gcc.exe

linker=${toolchain_dir}\arm-none-eabi-ld.exe

ar=${toolchain_dir}\arm-none-eabi-ar.exe

dump_assembly=${toolchain_dir}\arm-none-eabi-objdump.exe -D ${directory_OSAL}flash\OSAL_APP.out > ${directory_OSAL}flash\assembly.txt

make_bin=${toolchain_dir}\arm-none-eabi-objcopy.exe -S -O binary ${directory_OSAL}flash\OSAL_APP.out ${directory_OSAL}flash\APP_IMAGE.bin 

library_name=OSAL_LIBRARY


toolchain_lib=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\arm-none-eabi\lib
toolchain_lib_gcc=G:\ti_13\gcc-arm-none-eabi-6-2017-q1-update\lib\gcc\arm-none-eabi\6.3.1

linker_flags= --entry 0x80000000

output=${directory_OSAL}output\\

compiler_flag= -c
flags= -mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -marm -Dgcc -D__ARM_ARCH_7A__ -DSOC_AM335x -Dam3359 -mfpu=neon

#Source files



all: osal.o cpu.o chip.o make_library clean_up  link
	
osal.o:	
	-cls
	echo
	echo ************   Compiling PROJECT now!!   ****************	
	echo Building library: "osal" 
	echo Invoking: GNU Compiler	

	${compiler} ${directory_OSAL}osal/src/*.c ${compiler_flag} ${flags} -I${directory_OSAL}osal\INCLUDE\ -I${pdk_dir}  
	
	${compiler} ${directory_OSAL}osal/src/*.asm -c ${flags} -I${directory_OSAL}osal\INCLUDE\ -I${pdk_dir}  
	

cpu.o:	

	echo  Building library: "cpu"
	echo  Invoking: GNU Compiler
	 
	${compiler} ${directory_OSAL}cpu/src/*.c ${compiler_flag} ${flags} -I${directory_OSAL}cpu\INCLUDE\ -I${pdk_dir}

	${compiler} ${directory_OSAL}cpu/src/*.S ${compiler_flag} ${flags} -I${directory_OSAL}cpu\INCLUDE\ -I${pdk_dir} 

chip.o:
	echo  Building library: "board"
	echo  Invoking: GNU Compiler
	
	${compiler} ${directory_OSAL}board/src/*.c ${compiler_flag} ${flags} -I${directory_OSAL}board\INCLUDE\ -I${pdk_dir}  

	
	
link:
	echo ************   Linking PROJECT now!!   ****************	;
	${linker} ${linker_flags} ${output}*.o -L  ${toolchain_lib}\ -lc \
	-L ${toolchain_lib_gcc}\ -lgcc -v -o ${output}..\flash\OSAL_APP.out

	
	${make_bin}
	
	${dump_assembly}

	echo ************   End of Build!!   ****************	;


make_library:
	move main.o ${output}
	move app_startup.o ${output}
	move init_code.o ${output}
	
	${ar} -crs ${output}..\flash\lib${library_name}.a  *.o

clean_up:

	move *.o ${output}
	echo ************   Build PROJECT done!!   ****************;
	echo


	
clean: 	
	
	del ${output}*.o ${output}\*.out ${output}*.a
	del ${directory_OSAL}flash\*.out
	del ${directory_OSAL}flash\*.a
	del ${directory_OSAL}flash\*.bin ${directory_OSAL}flash\*.txt
	del assembly.txt