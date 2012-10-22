#!/usr/bin/env python

import roslib; roslib.load_manifest('opl')
import rospy
import sys

from simpleparse.common import numbers, strings, comments
from simpleparse.parser import Parser
from simpleparse.dispatchprocessor import DispatchProcessor

SCOPE_OPERATOR = '.'
OBJECT_NAME = 'Object'

class Element(object):
  def __init__(self, tag, name, parent):
    self.tag = tag
    self.name = name
    self._parent = parent
    self.children = list()
    self.child_names = set()
    self._is_symbol_definition = False
    self._is_type_definition = False
    self._is_name_definition = True
    
  def evaluate(self):
    self.custom_evaluation()
    for child in self.children:
      child.evaluate()
  
  def custom_evaluation(self):
    pass
  
  def find_this(self):
    if self._parent != None:
      return self._parent.find_this()
    return None
  
  def find_type(self, type_name):
    return self._parent.find_type(type_name)
  def defines_type(self, type_name = None):
    return self._is_type_definition and self._name == type_name
  
  def find_symbol(self, symbol_name):
    for child in self.children:
      if child.defines_symbol(symbol_name):
        return child
    if self._parent != None:
      return self._parent.find_symbol(symbol_name)
    return None
  def defines_symbol(self, symbol_name):
    return self._is_symbol_definition and self._name == symbol_name
  
  def add_child(self, element):
    if element._is_name_definition:
      if element.name in self.child_names:
        raise OPLParsingError('multiple definition of name {0} in element {1}.'.format(element.name, self.name))
      self.child_names.add(element.name)
    self.children.add(element)
  
  def generate_pddl(self):
    pass
  def generate_cpp(self):
    pass
  def __str__(self):
    children_str = ''
    for child in self.children:
      children_str += str(child)
    return '{0} {1} ({2})'.format(self.tag, self.name, children_str)
  
class Domain(Element):
  def __init__(self, tag, name):
    Element.__init__(self, tag, name, parent=None)

  def find_type(self, type_name):
    for child in self.children:
      if child.defines_type(type_name):
        return child
      
class Type(Element):
  def __init__(self, tag, name, parent, super_name):
    super(Type, self).__init__(tag, name, parent)
    self._is_type_definition = True
    self._super_name = super_name
    self._super_type = None
    self._evaluated = False
    
  def custom_evaluation(self):
    self._evaluated = True
    if self._super_name == self._name:
      raise OPLParsingError('type {0} extend itself.'.format(self.name))
    if self._super_name != None or self._super_name != OBJECT_NAME: 
      self._super_type = self._parent.find_type(self._super_name)
    if self._super_type == None:
      raise OPLParsingError('super_type {0} of type {1} not found'.format(self._super_name, self.name))
    
  def find_symbol(self, symbol_name):
    for child in self.children:
      if child.defines_symbol(symbol_name):
        return child
    if self._super_type != None:
      return self._super_type.find_symbol(symbol_name)
    return None
      
      
class OPLParsingError(Exception):
  def __init__(self, *args, **kwargs):
    Exception.__init__(self, *args, **kwargs)
  
if __name__=='__main__':
  #parser = OplParser('../domains/test_domain.opl')
  #parser.read_ebnf_definition('opl_definitions.ebnf')
    definitions = open('opl_definitions.ebnf', 'r').read()
    production = 'file'
    parser = Parser(definitions, production)
    domain_str = open('../domains/empty_domain.opl', 'r').read()
    success, children, nextcharacter = parser.parse(domain_str)
    assert success and nextcharacter==len(domain_str), """Wasn't able to parse input as a %s (%s chars parsed of %s), returned value was %s"""%( production, nextcharacter, len(domain_str), (success, children, nextcharacter))
    print """Successfully parsed input as a %s (%s chars parsed of %s), returned value was %s"""%( production, nextcharacter, len(domain_str), (success, children, nextcharacter))
    
    # domain
    tag, start, stop, sub_tags = children[0]
    name = domain_str[sub_tags[0][1]:sub_tags[0][2]]
    domain = Domain(tag, name)
#    tag, start, stop, sub_tags = children[0]
    
  
#    child = Type()
    print str(domain)
    
    
    
    