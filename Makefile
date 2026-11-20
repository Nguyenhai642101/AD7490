# ---------------------------------------------------------------------------
# AD7490 driver (Xilinx AXI Quad SPI, Cortex-A53) - build, test, coverage
#
#   make                 target library + host unit tests + style check
#   make target          build/a53/libad7490.a  (+ .su stack usage files)
#   make stack           largest stack usage per function (target build)
#   make test            build and run the host unit tests
#   make coverage        tests + line/branch coverage report (needs gcovr)
#   make mcdc            tests rebuilt with GCC >= 14 -fcondition-coverage
#                        + MC/DC condition coverage report (gcovr)
#   make style           rule 0 checks (80 columns, no //, no tabs, ...)
#   make clean           delete the build/ directory
#
# Cross compiler: bare-metal toolchain from Vitis by default. Override:
#   make target CROSS_COMPILE=aarch64-linux-gnu-
# ---------------------------------------------------------------------------

CROSS_COMPILE ?= aarch64-none-elf-
TARGET_CC     := $(CROSS_COMPILE)gcc
TARGET_AR     := $(CROSS_COMPILE)ar
TARGET_ARCH   ?= -mcpu=cortex-a53
HOST_CC       ?= gcc
GCOVR         ?= gcovr
# MC/DC needs GCC >= 14 (-fcondition-coverage) and its matching gcov
MCDC_CC       ?= gcc-14
MCDC_GCOV     ?= gcov-14

A53_DIR  := build/a53
HOST_DIR := build/host
COV_DIR  := build/coverage
MCDC_DIR := build/mcdc

INC := -Ihal -Idriver

# Rule 12: mandatory warnings, qualified optimisation profile (DAL A/B)
WARN := -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wconversion \
        -Wsign-conversion -Wshadow -Wstrict-prototypes \
        -Wmissing-prototypes -Wundef -Wcast-qual -Wcast-align=strict \
        -Wvla -Wstack-usage=256 -fno-common
OPT  := -O1 -fno-inline

TARGET_CFLAGS := $(WARN) $(OPT) $(TARGET_ARCH) -ffreestanding \
                 -fstack-usage $(INC)

# Host tests: a failed ASSERT counts as a test failure (test_common.h)
TEST_ASSERT := -D'ASSERT(expr)=test_assert((expr), (uint32_t)__LINE__)'
TEST_EXTRA  ?=
TEST_CFLAGS := -std=c99 -Wall -Wextra -Werror -O0 -g --coverage \
               $(INC) -Itest -include test/test_common.h $(TEST_ASSERT) \
               $(TEST_EXTRA)

A53_OBJS := $(A53_DIR)/axi_qspi_hal.o $(A53_DIR)/ad7490_driver.o \
            $(A53_DIR)/ad7490_monitor.o
A53_LIB  := $(A53_DIR)/libad7490.a

TEST_NAMES   := test_qspi_hal test_frame_convert test_device \
                test_sequencer test_monitor
TEST_BINS    := $(addprefix $(HOST_DIR)/,$(TEST_NAMES))
TEST_SUPPORT := $(HOST_DIR)/test_common.o $(HOST_DIR)/fake_ad7490.o
HOST_DRIVER  := $(HOST_DIR)/ad7490_driver.o

STYLE_FILES := $(wildcard hal/*.[ch] driver/*.[ch] test/*.[ch] \
                         examples/*.[ch])

.PHONY: all target stack test coverage mcdc style clean

all: target test style

# --------------------------- target (Cortex-A53) ---------------------------

target: $(A53_LIB)

$(A53_LIB): $(A53_OBJS)
	$(TARGET_AR) rcs $@ $^
	@echo "Built $@"

$(A53_DIR)/%.o: hal/%.c hal/axi_qspi_hal.h | $(A53_DIR)
	$(TARGET_CC) $(TARGET_CFLAGS) -c $< -o $@

$(A53_DIR)/%.o: driver/%.c driver/ad7490_driver.h \
                driver/ad7490_monitor.h | $(A53_DIR)
	$(TARGET_CC) $(TARGET_CFLAGS) -c $< -o $@

stack: $(A53_LIB)
	@echo "Stack usage per function (bytes), largest last:"
	@cat $(A53_DIR)/*.su | sort -t"$$(printf '\t')" -k2 -n | tail -10

# ------------------------------ host tests ---------------------------------

test: $(TEST_BINS)
	@find $(HOST_DIR) -name '*.gcda' -exec rm -f {} +
	@status=0; for t in $(TEST_BINS); do ./$$t || status=1; done; \
	 exit $$status

$(HOST_DIR)/test_qspi_hal: $(HOST_DIR)/test_qspi_hal.o \
                           $(HOST_DIR)/test_common.o
	$(HOST_CC) --coverage $^ -o $@

$(HOST_DIR)/test_frame_convert: $(HOST_DIR)/test_frame_convert.o \
                                $(TEST_SUPPORT)
	$(HOST_CC) --coverage $^ -lm -o $@

$(HOST_DIR)/test_device: $(HOST_DIR)/test_device.o $(HOST_DRIVER) \
                         $(TEST_SUPPORT)
	$(HOST_CC) --coverage $^ -o $@

$(HOST_DIR)/test_sequencer: $(HOST_DIR)/test_sequencer.o $(HOST_DRIVER) \
                            $(TEST_SUPPORT)
	$(HOST_CC) --coverage $^ -o $@

$(HOST_DIR)/test_monitor: $(HOST_DIR)/test_monitor.o $(HOST_DRIVER) \
                          $(TEST_SUPPORT)
	$(HOST_CC) --coverage $^ -o $@

# Tests that #include a HAL or driver source also depend on it
$(HOST_DIR)/test_qspi_hal.o: hal/axi_qspi_hal.c
$(HOST_DIR)/test_frame_convert.o: driver/ad7490_driver.c
$(HOST_DIR)/test_monitor.o: driver/ad7490_monitor.c

$(HOST_DIR)/%.o: test/%.c test/test_common.h test/fake_ad7490.h \
                 driver/ad7490_driver.h driver/ad7490_monitor.h | $(HOST_DIR)
	$(HOST_CC) $(TEST_CFLAGS) -c $< -o $@

$(HOST_DIR)/%.o: driver/%.c driver/ad7490_driver.h | $(HOST_DIR)
	$(HOST_CC) $(TEST_CFLAGS) -c $< -o $@

# ------------------------------- coverage ----------------------------------

coverage: test | $(COV_DIR)
	@echo "--- line coverage ---"
	@$(GCOVR) -r . $(HOST_DIR) --filter hal/ --filter driver/ \
	  --object-directory $(HOST_DIR)
	@echo "--- branch coverage (ASSERT failure branches excluded) ---"
	@$(GCOVR) -r . $(HOST_DIR) --filter hal/ --filter driver/ \
	  --object-directory $(HOST_DIR) \
	  --txt-metric branch --exclude-lines-by-pattern '.*ASSERT\(.*'
	@$(GCOVR) -r . $(HOST_DIR) --filter hal/ --filter driver/ \
	  --object-directory $(HOST_DIR) \
	  --exclude-lines-by-pattern '.*ASSERT\(.*' \
	  --html-details $(COV_DIR)/index.html
	@echo "HTML report: $(COV_DIR)/index.html"

# MC/DC (masking) condition coverage: same tests, separate build directory
mcdc:
	@$(MAKE) --no-print-directory test HOST_CC=$(MCDC_CC) \
	  HOST_DIR=$(MCDC_DIR) TEST_EXTRA=-fcondition-coverage
	@echo "--- MC/DC condition coverage ($(MCDC_CC)) ---"
	@$(GCOVR) -r . $(MCDC_DIR) --filter hal/ --filter driver/ \
	  --gcov-executable $(MCDC_GCOV) --txt-metric condition

# -------------------------------- style ------------------------------------

style:
	@for f in $(STYLE_FILES); do \
	  [ -z "$$(tail -c1 $$f)" ] || { echo "$$f: no final newline"; exit 1; }; \
	done
	@awk -f test/check_style.awk $(STYLE_FILES)

# --------------------------------- misc ------------------------------------

$(A53_DIR) $(HOST_DIR) $(COV_DIR):
	@mkdir -p $@

clean:
	rm -rf build
