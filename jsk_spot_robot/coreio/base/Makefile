all: $(HOME)/bash.sh
	@echo "make build: build docker for spot user"

$(HOME)/bash.sh:
	echo "#!/bin/bash\n\ncd ~/spot_driver_ws/src/jsk_robot/jsk_spot_robot/coreio/base\nmake shell\n" > $(HOME)/bash.sh
	chmod u+x $(HOME)/bash.sh


WS_ROOT=$(abspath $(CURDIR)/../../../../../)

define run
docker run --rm --privileged --hostname strelka --add-host strelka:133.11.216.166 --add-host strelka.jsk.imi.i.u-tokyo.ac.jp:133.11.216.166 --network=host --device=/dev/input -v $(WS_ROOT):/home/spot/ws -v $(HOME)/.ros:/home/spot/.ros -w /home/spot/ws -ti dev_env ${1};
endef
define docker_run
${1}
endef

pre_build:
	# make .ros file
	mkdir -p ~/.ros
	# build base file
	cp Dockerfile $(WS_ROOT)
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build --target base_build --progress=plain --network=host -t dev_env -f Dockerfile .
	$(call run, /bin/ls -al src)
	# copy workspaces
	if [ ! -e $(WS_ROOT)/src/.rosinstall ]; then $(call run, wstool init src) fi
	$(call run, wstool merge -t src src/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall)
	$(call run, wstool update -t src)
	# rosdep install all
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build --target pre_build --progress=plain --network=host -t dev_env -f Dockerfile .
	rm $(WS_ROOT)/Dockerfile

dev_build:
	cp Dockerfile $(WS_ROOT)
	cd $(WS_ROOT); [ -e package.xml.tar ] || find src -iname 'package.xml' | tar cf package.xml.tar -T -
	cd $(WS_ROOT); DOCKER_BUILDKIT=1 docker build --target dev_build --progress=plain --network=host -t dev_env -f Dockerfile .
	rm $(WS_ROOT)/Dockerfile $(WS_ROOT)/package.xml.tar

catkin_build:
	$(call run, catkin init)
	$(call run, catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so.1)
	$(call run, catkin build)


shell:
	$(call run, bash)

emacs:
	$(call run, emacs -nw)
