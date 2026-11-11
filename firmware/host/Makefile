# Build the player on a PC (gcc or clang) and print a table's tick stream.
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c99
play_table: play_table.c ../backflip_player.c ../backflip_player.h ../presets/*.h
	$(CC) $(CFLAGS) -I.. -o $@ play_table.c ../backflip_player.c

clean:
	rm -f play_table play_table.exe
