perceive action server
======================

Exposes an actionlib server to preceive objects from a location.

1. pipeline:

base_placement -> move arm to look_at_workspace -> triggers perception pipeline

2. input/output

input: location
output: object_list

3. a client is also provided inside this package to call the action server
