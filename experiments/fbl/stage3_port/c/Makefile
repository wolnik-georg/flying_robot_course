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
	rm -f test_fbl_residual test_vectors.txt test_fbl_law law_vectors_command.txt law_vectors_position.txt

test_law: test_fbl_law law_vectors_command.txt
	./test_fbl_law
law_vectors_command.txt: gen_law_vectors.py
	~/.pyenv/versions/flying_robots/bin/python gen_law_vectors.py
test_fbl_law: test_fbl_law.c fbl_law.c
	$(CC) $(CFLAGS) -o $@ test_fbl_law.c fbl_law.c -lm
