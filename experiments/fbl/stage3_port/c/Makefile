# Host build of the FBL residual module and its test (no firmware tree involved).  make test
TABLE ?= ../../out/stage3/rom_table_v1
CFLAGS ?= -O2 -std=c99 -Wall -Wextra -fno-fast-math -I. -I$(TABLE)
test: test_fbl_residual test_vectors.txt
	./test_fbl_residual
test_vectors.txt: gen_test_vectors.py
	~/.pyenv/versions/flying_robots/bin/python gen_test_vectors.py
test_fbl_residual: test_fbl_residual.c fbl_residual.c $(TABLE)/fbl_rom_table.c
	$(CC) $(CFLAGS) -o $@ test_fbl_residual.c fbl_residual.c $(TABLE)/fbl_rom_table.c -lm
clean:
	rm -f test_fbl_residual test_vectors.txt
