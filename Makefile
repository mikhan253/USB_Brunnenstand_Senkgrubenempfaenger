######################################################
# Auto generated Makefile @08.12.2024 09:54:13
######################################################



PROJECT = USBRF
CHIP = atmega328p
ID = m328p
TARGET = $(PROJECT).elf
TCPATH = /usr/sbin
CC = $(TCPATH)/avr-gcc
CPP = $(TCPATH)/avr-gcc
OL = Os
COMMON = -mmcu=$(CHIP)
CFLAGS = $(COMMON) -Wall -gdwarf-2 -std=gnu99 -$(OL) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=32000000UL -DDEBUG
GENDFLAGS = 
LDFLAGS = $(COMMON) -Wl,-Map=$(PROJECT).map
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .singature
HEX_EEPROM_FLAGS = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 --no-change-warnings
INCLUDES =  -I./Drivers/Includes
OBJECTS =  main.o DrvSYS.o DrvUSART.o DrvADC.o DrvTWI.o HlDrvVL53L0X.o DrvSPI.o HlDrvRFM23.o
SOBJECTS =  start.o
EXTOBJS = 

all: clean $(TARGET) $(PROJECT).hex $(PROJECT).lss
	@echo ""
	@echo build done!

DrvUSART.o : ./Drivers/Sources/DrvUSART.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/DrvUSART.c
DrvSYS.o : ./Drivers/Sources/DrvSYS.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/DrvSYS.c
DrvWDT.o : ./Drivers/Sources/DrvWDT.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/DrvWDT.c
DrvADC.o : ./Drivers/Sources/DrvADC.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/DrvADC.c
DrvSPI.o : ./Drivers/Sources/DrvSPI.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/DrvSPI.c
DrvTWI.o : ./Drivers/Sources/DrvTWI.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/DrvTWI.c
HlDrvRFM23.o : ./Drivers/Sources/HlDrvRFM23.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/HlDrvRFM23.c
HlDrvVL53L0X.o : ./Drivers/Sources/HlDrvRFM23.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./Drivers/Sources/HlDrvVL53L0X.c
main.o : ./main.c
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -c ./main.c

start.o : ./Drivers/Sources/start.s
	$(CC) $(INCLUDES) $(GENDFLAGS) $(CFLAGS) -x assembler-with-cpp -c ./Drivers/Sources/start.s


$(TARGET): $(OBJECTS) $(SOBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(SOBJECTS) $(EXTOBJS) $(LIBDIRS) $(LIBS) -o $(TARGET)

$(PROJECT).hex: $(TARGET)
	$(TCPATH)/avr-objcopy -O ihex $(HEX_FLASH_FLAGS) $(PROJECT).elf $(PROJECT).hex

$(PROJECT).lss: $(TARGET)
	$(TCPATH)/avr-objdump -h -S $(PROJECT).elf > $(PROJECT).lss
	@echo
	@$(TCPATH)/avr-size --mcu=$(CHIP) $(TARGET)

AVRDUDE: $(PROJECT).hex
	@$(TCPATH)/avrdude -c arduino -P /dev/ttyUSB0 -p lgt8f328p -Uflash:w:$(PROJECT).hex:i

clean: 
	@rm -rf *.o *.hex *.lss *.map *.elf
	@echo cleanup done!

