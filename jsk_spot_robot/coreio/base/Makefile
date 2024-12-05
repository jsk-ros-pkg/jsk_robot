HOME=$(shell realpath ~/)

all: $(HOME)/bash.sh
	@echo "make build"

$(HOME)/bash.sh:
	echo "#!/bin/bash\n\n" > $(HOME)/bash.sh
ifeq ($(shell id -u $$USER),1000) ## This is dev(uid=1000) setting
endif
	echo "cd $(CURDIR)\nmake shell ARGS=\$$@\n" >> $(HOME)/bash.sh
	chmod u+x $(HOME)/bash.sh


WS_ROOT=$(abspath $(CURDIR)/../../../../../)

HOSTNAME=$(shell hostname)
IPADDRESS=$(shell dig +short $(HOSTNAME).jsk.imi.i.u-tokyo.ac.jp)
IPADDRESS=$(shell hostname -I | sed 's/ /\n/g' | awk '/133.11/')

ifeq ($(shell id -u $$USER),1000) ## This is dev(uid=1000) setting
DOCKER_USER=spot
DOCKER_TARGET_NAME=spot_dev_env
$(info "This is dev setting, create $(DOCKER_TARGET_NAME)")
ifneq ($(WS_ROOT),$(HOME)/spot_dev_env)
$(info "We assume WS_ROOT to $(HOME)/spot_dev_env but current WS_ROOT is '$(WS_ROOT)'. Please setup as follows")
$(info "$ mkdir -p ~/spot_dev_env/src")
$(info "$ cd ~/spot_dev_env/src")
$(info "$ git clone $(shell git ls-remote --get-url) -b $(shell git branch | grep ^* | sed s/\*\ // )")
$(error "")
endif
else
DOCKER_USER=$(USER)
DOCKER_TARGET_NAME=$(USER)_dev_env
$(info "This is user setting, create $(DOCKER_TARGET_NAME)")
endif

define run_uid_not_equal
ifeq ($(shell id -u $$USER),${1})
$$(info "${2}")
$$(error "Target '$@' need to run except uid ${1} ($(shell id -un ${1})), your current uid is $(shell id -u $$USER) ($(shell echo $$USER))")
endif
endef
define run_uid_equal
ifneq ($(shell id -u $$USER),${1})
$$(info "${2}")
$$(error "Target '$@' need to run with uid ${1} ($(shell id -un ${1})), your current uid is $(shell id -u $$USER) ($(shell echo $$USER))")
endif
endef

define run
docker run --rm --privileged --hostname $(HOSTNAME)-core-io --add-host $(HOSTNAME)-core-io:$(IPADDRESS) --add-host $(HOSTNAME)-core-io.jsk.imi.i.u-tokyo.ac.jp:$(IPADDRESS) --network=host -u=$(shell id -u $$USER):$(shell id -g $$USER) --group-add sudo -v /dev:/dev -v $(shell realpath /home/$(shell id -un 1000))/spot_dev_env:/home/spot/ws -v $(WS_ROOT):/home/$(DOCKER_USER)/ws -v $(HOME)/.ros:/home/$(DOCKER_USER)/.ros -v $(HOME)/.config:/home/$(DOCKER_USER)/.config -w /home/$(DOCKER_USER)/ws $(shell if test -n "$(NAME)"; then echo '--name $$NAME'; fi) -ti $(DOCKER_TARGET_NAME) ${1};
endef

define create_src_tree_tar
$(info "Create source tree tar file")
$(eval TMP_FILE:=$(shell mktemp))
$(eval PKG_FILE:=$(WS_ROOT)/package.tar)
cp Dockerfile $(WS_ROOT); sed -i 's+@@HOME@@+$(HOME)+' $(WS_ROOT)/Dockerfile;  (cd $(WS_ROOT); find src -iname 'package.xml' -o -path '*jsk_spot_robot/requirements.txt' | tar cf $(TMP_FILE) -T -); chmod 644 $(TMP_FILE)
if test "$$(md5sum $(TMP_FILE) | awk '{print $$1}')" = "$$(md5sum $(PKG_FILE) | awk '{print $$1}')" ; then echo "- We have latest $(PKG_FILE)"; else echo "- source tree is different from $(PKG_FILE), create new source tree tar"; cp $(TMP_FILE) $(PKG_FILE); tar -tf $(PKG_FILE); fi
rm $(TMP_FILE)
endef
define delete_src_tree_tar
rm $(WS_ROOT)/Dockerfile;
endef

# setup workspace ( clone source code from jsk_spot_draiver.rosinstall, rosdep install, wstool )
base_build:
	$(eval $(call run_uid_equal,1000,This target requires dev setting with uid 1000))
	# make .ros file
	mkdir -p ~/.ros
	# create tar file to pass package.xml under jsk_spot_robot, so that pip3 install requirements.txt
	$(call create_src_tree_tar)
	# build base file
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build -t spot_dev_env:base_build --target base_build --progress=plain --network=host -t spot_dev_env -f Dockerfile .
	# run rosdep update
	@$(call run, bash -c '[ -e /etc/ros/rosdep/sources.list.d/ ] || rosdep update -y --include-eol-distros')
	# create workspace from jsk_spot_driver.rosinstall
	@if [ ! -e $(WS_ROOT)/src/.rosinstall ]; then $(call run, wstool init src) fi
	@$(call run, wstool merge -t src src/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall)
	@if test $$(find $(WS_ROOT)/src/.rosinstall -mmin +1 -print); then echo ".rosinstall has not been chaned, skip wstool update" ; else $(call run, wstool update -t src) fi
	# delete package.tar
	$(call delete_src_tree_tar)

# run rosdep install
pre_build: base_build
	$(eval $(call run_uid_equal,1000,This target requires dev setting with uid 1000))
	# create tarfile to pass package.xm under $($WS_ROOT), so that rosdep install finds necessary dependency list
	$(call create_src_tree_tar)
	# rosdep install
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build -t spot_dev_env:pre_build --target pre_build --progress=plain --network=host -t spot_dev_env -f Dockerfile .
	# delete package.tar
	$(call delete_src_tree_tar)

# install dev tools, ros_entrypoint.sh
dev_build: pre_build
	$(eval $(call run_uid_equal,1000,This target requires dev setting with uid 1000))
	# dev_build does not use package.xml
	$(call create_src_tree_tar)
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build -t spot_dev_env:dev_build --target dev_build --progress=plain --network=host -t spot_dev_env -f Dockerfile .
	# delete package.tar
	$(call delete_src_tree_tar)

# run user build
user_build:
	$(eval $(call run_uid_not_equal,1000,This target requires user setting, except uid 1000))
	$(call create_src_tree_tar)
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build -t spot_dev_env:user_build --target user_build --progress=plain --network=host -t $(shell echo $$USER)_dev_env --build-arg USER=$$USER --build-arg UID=$(shell id -u $$USER) --build-arg GID=$(shell id -g $$USER) -f Dockerfile .

# run catkin build
catkin_config:
	$(call run, catkin init)
	$(call run, catkin config $(CATKIN_CONFIG) --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so.1)

catkin_build:
	$(call run, catkin build --summarize)

# run all build
build:
ifeq ($(shell id -u $$USER),1000) ## This is dev(uid=1000) setting
	make dev_build
	make catkin_config CATKIN_CONFIG='--install --blacklist test_tf2'
	make catkin_build
else
	make user_build
	make catkin_config
endif
shell:
	echo "$(ARGS)" > ~/.ros/bash-init.sh
	$(call run, bash --init-file /home/$(DOCKER_USER)/.ros/bash-init.sh)

emacs:
	$(call run, emacs -nw)
