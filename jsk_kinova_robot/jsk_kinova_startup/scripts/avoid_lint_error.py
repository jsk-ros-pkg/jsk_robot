# This file is to avoid roslint_add_test() failure in CMakeLists.txt.
# TODO: Please remove this file after other python files are created.

# roslint_add_test() failure:
# <?xml version="1.0" encoding="UTF-8"?>
# <testsuite name="roslint" tests="1" failures="1" errors="0">
#   <testcase name="roslint_jsk_kinova_startup" classname="roslint.LintCheck">
#     <failure message="One or more linter errors was reported." type=""><![CDATA[make[4]: Entering directory '/home/leus/catkin_workspace/build/jsk_kinova_startup'
# make[4]: warning: jobserver unavailable: using -j1.  Add '+' to parent make rule.
# make[4]: *** No rule to make target 'roslint_jsk_kinova_startup'.  Stop.
# make[4]: Leaving directory '/home/leus/catkin_workspace/build/jsk_kinova_startup']]></failure>
#   </testcase>
# </testsuite>
