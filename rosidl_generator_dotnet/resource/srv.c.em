@{
type_name = service.namespaced_type.name
srv_typename = '%s__%s' % ('__'.join(service.namespaced_type.namespaces), type_name)
}@
const void * @(srv_typename)__get_typesupport(void) {
  const void * ptr = ROSIDL_GET_SRV_TYPE_SUPPORT(@(', '.join(service.namespaced_type.namespaced_name())));
  return ptr;
}
