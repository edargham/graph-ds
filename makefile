# Sample Makefile
# test_dense: optvanilladense.o main.o
# 	g++-11 -std=c++17 -O3 optvanilladense.o main.o -o test_dense -lpthread -lblas -lpthread #-lcblas -latlas -lopenblas

# optvanilladense.o: optvanilladense.cpp
# 	g++-11 -std=c++17 -O3 -c optvanilladense.cpp

# main.o: main.cpp
# 	g++-11 -std=c++17 -O3 -c -Wno-format main.cpp

# clean_build:
# 	rm -rf *.o

# clean:
# 	rm -rf *.o test_dense

# profile:
# 	valgrind --tool=massif --time-unit=ms --max-snapshots=100 --stacks=yes --depth=5 ./test_dense 1 100

# clear_profile:
# 	rm -rf massif.*

.PHONY: clean clean_build rebuild

graph_ds: ./build/main.o
	mkdir -p ./build/
	g++ -std=c++17 -O3 ./build/main.o -o graph_ds.out

./build/main.o: ./src/main.cpp
	mkdir -p ./build/
	g++ -std=c++17 -O3 -c ./src/main.cpp -o ./build/main.o

clean_build:
	rm -rf ./build/*.o

clean:
	rm -rf ./build/*.o graph_ds.out

rebuild:
	make clean && make
