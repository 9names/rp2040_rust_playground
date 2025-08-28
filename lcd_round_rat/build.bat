cargo build --release
picotool.exe uf2 convert target\thumbv6m-none-eabi\release\lcd_round_rat -t elf lcd_round_rat.uf2
copy lcd_round_rat.uf2 d:\