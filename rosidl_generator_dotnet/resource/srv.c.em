@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore

type_name = service.namespaced_type.name
msg_typename = '%s__%s' % ('__'.join(service.namespaced_type.namespaces), type_name)
header_filename = "{0}/rcldotnet_{1}.h".format('/'.join(service.namespaced_type.namespaces), convert_camel_case_to_lower_case_underscore(type_name))
}@

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>

#include <@('/'.join(service.namespaced_type.namespaces))/@(convert_camel_case_to_lower_case_underscore(type_name)).h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>


#include "@(header_filename)"

@# Add message support for request and response messages
@{
TEMPLATE(
    'srv.msg.c.em',
    package_name=package_name, message=service.request_message)
}@

@{
TEMPLATE(
    'srv.msg.c.em',
    package_name=package_name, message=service.response_message)
}@

@# Add specific type support for the service ???
const void * @(msg_typename)__get_typesupport() {
  const void * ptr = ROSIDL_GET_SRV_TYPE_SUPPORT(@(package_name), srv, @(type_name));
  return ptr;
}