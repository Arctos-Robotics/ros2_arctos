# Define packages
PACKAGES = arctos_bringup arctos_hardware_interface arctos_motor_driver arctos_description

# ANSI color codes for styling
GREEN = \033[1;32m
YELLOW = \033[1;33m
BLUE = \033[1;34m
RED = \033[1;31m
RESET = \033[0m

# Use bash for proper handling of color codes
SHELL := /bin/bash

# Default target
.PHONY: menu
menu:
	@printf "$(BLUE)=========================================$(RESET)\n"
	@printf "$(GREEN)       ROS2 Package Build Menu$(RESET)\n"
	@printf "$(BLUE)=========================================$(RESET)\n"
	@printf " $(YELLOW)1)$(RESET) Build all packages\n"
	@printf " $(YELLOW)2)$(RESET) Clean build/, install/, logs/\n"
	@printf " $(YELLOW)3)$(RESET) Clean CMake cache\n"
	@printf " $(YELLOW)4)$(RESET) Build a specific package\n"
	@printf " $(YELLOW)5)$(RESET) Exit\n"
	@printf "$(BLUE)=========================================$(RESET)\n"
	@printf "$(GREEN)Choose an option: $(RESET)"
	@read choice; \
	case $$choice in \
		1) $(MAKE) build_all ;; \
		2) $(MAKE) clean ;; \
		3) $(MAKE) clean_cache ;; \
		4) $(MAKE) build_package ;; \
		5) exit 0 ;; \
		*) printf "$(RED)Invalid option. Please try again.$(RESET)\n"; $(MAKE) menu ;; \
	esac

# Build all packages
.PHONY: build_all
build_all:
	@printf "$(GREEN)Building all packages...$(RESET)\n"
	colcon build --packages-select $(PACKAGES)
	@printf "$(GREEN)Build complete!$(RESET)\n"

# Clean build, install, and logs directories
.PHONY: clean
clean:
	@printf "$(YELLOW)Cleaning build/, install/, and log/ directories...$(RESET)\n"
	rm -rf build/ install/ log/
	@printf "$(GREEN)Clean complete!$(RESET)\n"

# Clean CMake cache
.PHONY: clean_cache
clean_cache:
	@printf "$(YELLOW)Clearing CMake cache...$(RESET)\n"
	rm -rf build/CMakeCache.txt build/CMakeFiles
	@printf "$(GREEN)CMake cache cleared!$(RESET)\n"

# Build a specific package
.PHONY: build_package
build_package:
	@printf "$(BLUE)Available packages:$(RESET) $(PACKAGES)\n"
	@printf "$(GREEN)Enter the package name to build: $(RESET)"
	@read package; \
	if echo "$(PACKAGES)" | grep -wq $$package; then \
		printf "$(GREEN)Building $$package...$(RESET)\n"; \
		colcon build --packages-select $$package; \
		printf "$(GREEN)Build for $$package complete!$(RESET)\n"; \
	else \
		printf "$(RED)Invalid package name. Please try again.$(RESET)\n"; \
		$(MAKE) build_package; \
	fi
