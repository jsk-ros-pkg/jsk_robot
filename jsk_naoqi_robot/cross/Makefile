TARGET_MACHINE?=arm64v8

all:
	@echo "'make system' for setup System directories"
	@echo "'make user' for setup User directories"
	@echo ""
	@echo "By default, it compile with arm64v8(aarch64) target"
	@echo "To compile with amd64 target"
	@echo "run 'make system TARGET_MACHINE=amd64' or"
	@echo "    'TARGET_MACHINE=amd64 ./build_ros1.sh'"

system:
	./prepare_requirements_ros1.sh
	./build_ros1_dependencies.sh
	./build_ros1.sh

user:
	./build_user.sh

install:
	./install.sh

clean:
	rm -fr ${TARGET_MACHINE}_ws_*

distclean: clean
	rm -fr ${TARGET_MACHINE}_*

