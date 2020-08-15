.PHONY: all clean flash

PROJECT_NAME := simple-menu

SRCS := $(wildcard *.rs) Cargo.toml

all: $(PROJECT_NAME).bin

$(PROJECT_NAME).bin: target/thumbv7em-none-eabihf/debug/$(PROJECT_NAME)
	arm-none-eabi-objcopy $< -O binary -S -R .comment -R .eh_frame $@

target/thumbv7em-none-eabihf/debug/$(PROJECT_NAME): $(SRCS)
	cargo build

target/thumbv7em-none-eabihf/release/$(PROJECT_NAME): $(SRCS)
	cargo build --release

flash: $(PROJECT_NAME).bin
	bossac -e -w -v -b -o 0x4000 -R $(PROJECT_NAME).bin

clean:
	cargo clean
	-@$(RM) $(PROJECT_NAME).bin