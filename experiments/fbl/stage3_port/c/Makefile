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
	rm -f test_fbl_residual test_vectors.txt test_fbl_law law_vectors_command.txt law_vectors_position.txt test_fbl_law_abg law_vectors_abg.txt test_fbl_nn nn_vectors_sample.txt nn_vectors_h4.txt test_fbl_nn_export fbl_nn_weights_sample.c test_fbl_nn_trained nn_vectors_trained.txt

test_law: test_fbl_law law_vectors_command.txt
	./test_fbl_law
law_vectors_command.txt: gen_law_vectors.py
	~/.pyenv/versions/flying_robots/bin/python gen_law_vectors.py
test_fbl_law: test_fbl_law.c fbl_law.c
	$(CC) $(CFLAGS) -o $@ test_fbl_law.c fbl_law.c -lm

test_abg: test_fbl_law_abg law_vectors_abg.txt
	./test_fbl_law_abg
law_vectors_abg.txt: gen_law_vectors_abg.py
	~/.pyenv/versions/flying_robots/bin/python gen_law_vectors_abg.py
test_fbl_law_abg: test_fbl_law_abg.c fbl_law.c
	$(CC) $(CFLAGS) -o $@ test_fbl_law_abg.c fbl_law.c -lm

test_nn: test_fbl_nn nn_vectors_sample.txt
	./test_fbl_nn
nn_vectors_sample.txt: gen_nn_vectors.py
	~/.pyenv/versions/flying_robots/bin/python gen_nn_vectors.py
test_fbl_nn: test_fbl_nn.c fbl_residual.c $(TABLE)/fbl_rom_table.c
	$(CC) $(CFLAGS) -o $@ test_fbl_nn.c fbl_residual.c $(TABLE)/fbl_rom_table.c -lm

test_export: test_fbl_nn_export nn_vectors_sample.txt
	./test_fbl_nn_export
fbl_nn_weights_sample.c: ../export_fbl_nn_weights.py
	~/.pyenv/versions/flying_robots/bin/python ../export_fbl_nn_weights.py $(HOME)/Desktop/learned_fbl_core/KNODE/Models/sample_model.pth $@ --tag bundle_sample
test_fbl_nn_export: test_fbl_nn_export.c fbl_residual.c fbl_nn_weights_sample.c $(TABLE)/fbl_rom_table.c
	$(CC) $(CFLAGS) -o $@ test_fbl_nn_export.c fbl_residual.c fbl_nn_weights_sample.c $(TABLE)/fbl_rom_table.c -lm

test_trained: test_fbl_nn_trained nn_vectors_sample.txt
	./test_fbl_nn_trained nn_vectors_trained.txt
test_fbl_nn_trained: test_fbl_nn_export.c fbl_residual.c $(HOME)/Desktop/crazyflie-firmware/src/modules/src/fbl_nn_weights.c $(TABLE)/fbl_rom_table.c
	$(CC) $(CFLAGS) -o $@ test_fbl_nn_export.c fbl_residual.c $(HOME)/Desktop/crazyflie-firmware/src/modules/src/fbl_nn_weights.c $(TABLE)/fbl_rom_table.c -lm

bench: bench_step
	./bench_step
bench_step: bench_step.c fbl_residual.c fbl_law.c $(TABLE)/fbl_rom_table.c
	$(CC) $(CFLAGS) -o $@ bench_step.c fbl_residual.c fbl_law.c $(TABLE)/fbl_rom_table.c -lm
