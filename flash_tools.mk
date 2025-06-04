STLINK_GDB_PATH = C:\ST\STM32CubeIDE_1.11.2\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.win32_2.1.100.202310302101\tools\bin\ST-LINK_gdbserver.exe

flash:
	openocd -f"interface/stlink.cfg" -f"target/stm32f4x.cfg" -c"program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

jflash:
	JLink -device STM32F411CC -if SWD -speed 4000 -CommanderScript flash.jlink

debug_legacy:
	$(STLINK_GDB_PATH) -p 17053 -cp "C:\ST\STM32CubeIDE_1.11.2\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.1.100.202311100844\tools\bin" --frequency 1000 --swd

debug:
	openocd -f"interface/stlink.cfg" -f"target/stm32f4x.cfg" -c"program $(BUILD_DIR)/$(TARGET).elf reset "

clean:
	rmdir /s /q $(BUILD_DIR)

run: all flash