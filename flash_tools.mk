STLINK_GDB_PATH = C:\ST\STM32CubeIDE_1.11.2\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.win32_2.1.100.202310302101\tools\bin\ST-LINK_gdbserver.exe

# flash: Uses OpenOCD to flash the compiled ELF file to the STM32 device, verify it, reset the MCU, and then exit.
flash:
	openocd -f"interface/stlink.cfg" -f"target/stm32f4x.cfg" -c"program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

# jflash: Uses JLink command line tool to flash the MCU using a JLink script file (flash.jlink).
jflash:
	JLink -device STM32F411CC -if SWD -speed 4000 -CommanderScript flash.jlink

# debug_legacy: Starts the ST-Link GDB server with specific parameters to allow debugging via SWD interface.
debug_legacy:
	$(STLINK_GDB_PATH) -p 17053 -cp "C:\ST\STM32CubeIDE_1.11.2\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.1.100.202311100844\tools\bin" --frequency 1000 --swd

# debug: Uses OpenOCD to flash the compiled ELF file and reset the MCU, leaving the device ready for debugging.
debug:
	openocd -f"interface/stlink.cfg" -f"target/stm32f4x.cfg" -c"program $(BUILD_DIR)/$(TARGET).elf reset "

# clean: (commented out) Removes the entire build directory and its contents.
# clean:
# 	rmdir /s /q $(BUILD_DIR)

# run: Builds all targets, then flashes the firmware to the device.
run: all flash
