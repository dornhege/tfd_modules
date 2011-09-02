#!/usr/bin/env python

import os
import collections

def make_dir_list(path):
    """ Create list from path where each directory is one list entry """
    front,back = os.path.split(path)
    if not back and not front:
        return []
    if not back:
        return make_dir_list(front)
    if not front:
        return [ back ]
    frontL = make_dir_list(front)
    backL = make_dir_list(back)
    if not frontL:
        return backL
    if not backL:
        return frontL
    theList = []
    theList.extend(frontL)
    theList.extend(backL)
    return theList

def behaves_like_dict(object):
    """Check if object is dict-like"""
    return isinstance(object, collections.Mapping)
    
# From: http://appdelegateinc.com/blog/2011/01/12/merge-deeply-nested-dicts-in-python/
def merge(a, b):
    """Merge two deep dicts non-destructively
    
    Uses a stack to avoid maximum recursion depth exceptions
    
    >>> a = {'a': 1, 'b': {1: 1, 2: 2}, 'd': 6}
    >>> b = {'c': 3, 'b': {2: 7}, 'd': {'z': [1, 2, 3]}}
    >>> c = merge(a, b)
    >>> from pprint import pprint; pprint(c)
    {'a': 1, 'b': {1: 1, 2: 7}, 'c': 3, 'd': {'z': [1, 2, 3]}}
    """
    assert behaves_like_dict(a), behaves_like_dict(b)
    dst = a.copy()
    
    stack = [(dst, b)]
    while stack:
        current_dst, current_src = stack.pop()
        for key in current_src:
            if key not in current_dst:
                current_dst[key] = current_src[key]
            else:
                if behaves_like_dict(current_src[key]) and behaves_like_dict(current_dst[key]) :
                    stack.append((current_dst[key], current_src[key]))
                else:
                    current_dst[key] = current_src[key]
    return dst

