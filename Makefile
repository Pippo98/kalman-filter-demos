MAKEFLAGS += --no-print-directory
CORES = $(shell nproc)

.PHONY: release debug format clean

define setup_folder
	if [ -d bin ]; then \
		if [ ! -f .last_compilation_mode ] || [ "$$(cat .last_compilation_mode | sed 's/^ *//;s/ *$$//')" != "$$(echo $(1) | sed 's/^ *//;s/ *$$//')" ]; then \
			echo "File content '$$(cat .last_compilation_mode)' differs from '$(1)' (after trimming spaces)"; \
			echo "===> removing binaries\n"; \
			rm -r bin; \
		fi; \
	fi; \
	mkdir -p $(1); \
	echo $(1) > .last_compilation_mode; \
	cd $(1);
endef

debug:
	$(call setup_folder, debug) \
 	cmake .. -DCMAKE_BUILD_TYPE=Debug; \
	make -j$(CORES);

release:
	$(call setup_folder, release) \
	cmake .. -DCMAKE_BUILD_TYPE=Release; \
	make -j$(CORES);

format:
	find ./core/ -name "*.c*" -o -name "*.h*" | xargs clang-format -style=file:.clang-format -i

clean:
	@if [ -d debug ]; then \
		echo "removing debug folder"; \
		rm -r debug;\
	fi;
	@if [ -d release ]; then \
		echo "removing release folder"; \
		rm -r release;\
	fi 
