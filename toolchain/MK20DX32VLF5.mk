FIXED_SECTIONS+=	-s 0:.isr_vector
FIXED_SECTIONS+=	-s 0x400:.flash_config

LOADER_SIZE=	3072
LOADER_ADDR=	0
APP_SIZE=	29696
APP_ADDR=	3072

# TARGET=	MK20DX32VLF5
TARGET_FAMILY= MK20
TARGET_FAMILY_SHORT=k20
CPU_FAMILY= cortex-m4
