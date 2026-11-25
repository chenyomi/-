BUILD := build

.PHONY: all test demo clean

all:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Debug
	cmake --build $(BUILD)

test: all
	ctest --test-dir $(BUILD) --output-on-failure

demo: all
	./$(BUILD)/yoomi-cell

clean:
	rm -rf $(BUILD)
