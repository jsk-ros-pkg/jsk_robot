#!/usr/bin/env python


from catkin_pkg.package import parse_package
import optparse
import logging
import os
from rosdep2.rospack import init_rospack_interface, is_ros_package, is_system_dependency, is_view_empty
from rosdistro import get_cached_distribution, get_index, get_index_url
from rosinstall_generator.generator import generate_rosinstall, sort_rosinstall
import rospkg
import sys
import yaml

_static_rosdep_view = init_rospack_interface()

packages_checked = []
packages_uninstalled = []
def main():
    parser = optparse.OptionParser()
    parser.add_option("--rosdistro", action="store", default="melodic")
    parser.add_option("--exclude", action="append", default=['RPP'], type="str")
    (options, packages) = parser.parse_args()
    distro_name = options.rosdistro
    excludes = options.exclude

    distro = get_cached_distribution(get_index(get_index_url()), distro_name)
    release_packages = distro.release_packages
    r = rospkg.RosPack()

    # pass all logging output to stderr
    logger = logging.getLogger('rosinstall_generator')
    logger.addHandler(logging.StreamHandler(sys.stderr))

    for package in packages:
        logger.warn("# '{}'".format(package))  # start processing
        logger.warn("- Check if '{}' exists".format(package))  # check if package exists in ROS_PACKAGE_PATH
        try:
            # If found, add them to 'packages_checked and' add it's depeneds to 'packages'
            path = r.get_path(package)
            logger.warn("  ... find '{}' at '{}'".format(package, path))
            # depends = r.get_depends(package, implicit=False)  # Get all depends
            p = parse_package(os.path.join(r.get_path(package),"package.xml"))
            depends = filter(lambda name: name in r.list() or not is_system_dependency(_static_rosdep_view, name),
                             [d.name for d in p.build_depends + p.run_depends])
            logger.warn("- Package '{}' depends on {}".format(package, depends))
            # If depends is alredy checked, we skip them
            depends = set(depends) - set(packages_checked) - set(packages)
            packages.extend(depends)  # add depends to packages
            packages_checked.append(package)  # add packages to releaed_packages
        except rospkg.common.ResourceNotFound:
            # If not found, tha packages should be released
            logger.warn("- Check if '{}' released".format(package))  # FIXME: should be unreleased, if released, need to skip
            if  package in release_packages.keys():
                logger.warn("   ... {} is already released in '{}'".format(package, release_packages[package].repository_name))
                # package not found in WS and found in release should be installed
                packages_uninstalled.append(package)
                packages_checked.append(package)
                continue
            else:
                logger.error("'{}' is not releasead, you need add them within your workspace".format(package))
    # run rosinstall generator
    logger.warn("rosinstall_generator {} --rosdistro melodic --deps --exclude {}".format(' '.join(packages_uninstalled), excludes))

    # assuming we have https://github.com/ros-infrastructure/rosinstall_generator/pull/81
    #
    # apt install -y python-pip
    # git clone https://github.com/k-okada/rosinstall_generator /tmp/rosinstall_generator -b add_depend_type
    # pip install /tmp/rosinstall_generator
    try:
        rosinstall_data = generate_rosinstall(distro_name, packages_uninstalled, deps=True, excludes=excludes,
                                              depend_type=['buildtool', 'buildtool_export', 'build', 'build_export', 'run'])
        rosinstall_data = sort_rosinstall(rosinstall_data)
        print(yaml.safe_dump(rosinstall_data, default_flow_style=False))
    except Exception as e:
        if str(e) in ['No packages/stacks left after ignoring unreleased',
                      'No packages/stacks left after applying the exclusions']:
            logger.warn("rosinstall said : {}".format(str(e)))
            logger.warn("                  The packages may have been already installed")
        else:
            raise Exception(e)

if __name__ == "__main__":
    main()
