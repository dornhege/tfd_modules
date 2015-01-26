#!/usr/bin/env python
import roslib
from Cheetah.Template import Template
import sys
import os
import string

# UpperCamelCase -> [upper, camel, case]
def split_lower_case(name):
    index = [i for i in range(len(name)) if name[i] in string.uppercase]
    parts = []
    for i in range(len(index)):
        if i == len(index) - 1: # last
            parts.append(name[index[i]:].lower())
        else:
            parts.append(name[index[i]:index[i+1]].lower())
    return parts

# name in UpperCamelCase -> under_scored
def make_underscore(name):
    parts = split_lower_case(name)
    return '_'.join(parts)

# name in UpperCamelCase -> CAPS_NAME
def make_caps(name):
    parts = split_lower_case(name)
    return '_'.join([x.upper() for x in parts])

if __name__=="__main__":
    if len(sys.argv) < 3:
        print "Usage %s action namespace [s]" % sys.argv[0], "(if 's' is given at the end a service action executor is created"
        sys.exit(1)
    action_pkg, action_name = sys.argv[1].split("/")
    namespace = sys.argv[2]
    action_underscored = make_underscore(action_name)
    action_caps = make_caps(action_name)
    #print action_pkg
    #print action_name
    #print action_underscored
    #print action_caps
    #print namespace

    tmpl_dir = roslib.packages.get_pkg_subdir("continual_planning_executive", "scripts", False)
    if not tmpl_dir:
        print "Could not find package://continual_planning_executive/scripts directory"
        sys.exit(1)

    header_tmpl_name = "actionExecutor.h.tmpl"
    cpp_tmpl_name = "actionExecutor.cpp.tmpl"
    if len(sys.argv) >= 4 and sys.argv[3] == "s":
        header_tmpl_name = "actionExecutorService.h.tmpl"
        cpp_tmpl_name = "actionExecutorService.cpp.tmpl"

    header_template = Template(file=os.path.join(tmpl_dir, header_tmpl_name),
            searchList=[{'PACKAGE'  : action_pkg,
                         'ACTION'   : action_name,
                         'ACTION_CAPS'   : action_caps,
                         'ACTION_UNDERSCORED'   : action_underscored,
                         'NAMESPACE' : namespace}])
    f = file("actionExecutor%s.h" % action_name, 'w')
    f.write(str(header_template))
    f.close()

    cpp_template = Template(file=os.path.join(tmpl_dir, cpp_tmpl_name),
            searchList=[{'PACKAGE'  : action_pkg,
                         'ACTION'   : action_name,
                         'ACTION_CAPS'   : action_caps,
                         'ACTION_UNDERSCORED'   : action_underscored,
                         'NAMESPACE' : namespace}])
    f = file("actionExecutor%s.cpp" % action_name, 'w')
    f.write(str(cpp_template))
    f.close()

