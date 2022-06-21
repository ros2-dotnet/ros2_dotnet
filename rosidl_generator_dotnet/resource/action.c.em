@{
type_name = action.namespaced_type.name
action_typename = '%s__%s' % ('__'.join(action.namespaced_type.namespaces), type_name)
}@
const void * @(action_typename)__get_typesupport(void) {
  const void * ptr = ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_c, @(', '.join(action.namespaced_type.namespaced_name())))();
  return ptr;
}
