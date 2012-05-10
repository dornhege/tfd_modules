#!/usr/bin/env python
from Cheetah.Template import Template
import sys
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
        print "Usage %s action namespace" % sys.argv[0]
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

    header_template = Template(file="actionExecutor.h.tmpl", 
            searchList=[{'PACKAGE'  : action_pkg,
                         'ACTION'   : action_name,
                         'ACTION_CAPS'   : action_caps,
                         'ACTION_UNDERSCORED'   : action_underscored,
                         'NAMESPACE' : namespace}])
    f = file("actionExecutor%s.h" % action_name, 'w')
    f.write(str(header_template))
    f.close()

    cpp_template = Template(file="actionExecutor.cpp.tmpl", 
            searchList=[{'PACKAGE'  : action_pkg,
                         'ACTION'   : action_name,
                         'ACTION_CAPS'   : action_caps,
                         'ACTION_UNDERSCORED'   : action_underscored,
                         'NAMESPACE' : namespace}])
    f = file("actionExecutor%s.cpp" % action_name, 'w')
    f.write(str(cpp_template))
    f.close()

