// generated from rosidl_generator_cs/resource/_msg_pkg_typesupport_entry_point.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating _<msg_pkg>_s.ep.<typesupport_impl>_c.c files
@#
@# Context:
@#  - package_name
@#  - message_specs (list of rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg files
@#  - service_specs (list of rosidl_parser.ServiceSpecification)
@#    Parsed specification of the .srv files
@#  - typesupport_impl (string identifying the typesupport used)
@#  - convert_camel_case_to_lower_case_underscore (function)
@#######################################################################
@

#include <stdbool.h>
#include <stdint.h>

@{
static_includes = set([
    '#include <rosidl_generator_c/message_type_support_struct.h>',
    '#include <rosidl_generator_c/visibility_control.h>',
])
for spec, subfolder in message_specs:
  if subfolder == 'msg':
    static_includes.add('#include <rosidl_generator_c/message_type_support_struct.h>')
  elif subfolder == 'srv' or subfolder == 'action':
    static_includes.add('#include <rosidl_generator_c/service_type_support_struct.h>')
}@
@[for value in sorted(static_includes)]@
@(value)
@[end for]@

@{
includes = {}
for spec, subfolder in message_specs:
  type_name = spec.base_type.type
  module_name = convert_camel_case_to_lower_case_underscore(type_name)
  key = '%s/%s/%s' % (spec.base_type.pkg_name, subfolder, module_name)
  includes[key + '_support'] = '#include <%s__type_support.h>' % key
  includes[key + '_struct'] = '#include <%s__struct.h>' % key
  includes[key + '_functions'] = '#include <%s__functions.h>' % key

for spec, subfolder in service_specs:
  type_name = convert_camel_case_to_lower_case_underscore(spec.srv_name)
  module_name = convert_camel_case_to_lower_case_underscore(type_name)
  key = '%s/%s/%s' % (spec.pkg_name, subfolder, module_name)
  includes[key] = '#include <%s.h>' % key
}@
@[for v in sorted(includes.values())]@
@(v)
@[end for]@

@[for spec, subfolder in message_specs]@
@{
pkg_name = spec.base_type.pkg_name
type_name = spec.base_type.type
module_name = convert_camel_case_to_lower_case_underscore(type_name)
msg_typename = '%s__%s__%s' % (pkg_name, subfolder, type_name)
}@
@[end for]@

@[for spec, subfolder in message_specs]@
@{
type_name = convert_camel_case_to_lower_case_underscore(spec.base_type.type)
}@

ROSIDL_GENERATOR_C_EXPORT
void * @(pkg_name)__@(subfolder)__@(type_name)__get_type_support()
{
    return (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(@(pkg_name), @(subfolder), @(spec.msg_name));
}
@[end for]@
