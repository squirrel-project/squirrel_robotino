squirrel_common
===============

Repository for message, service and action related SQUIRREL packages.

Compiling the Repository
========================

Compiling the Robotino driver requires the addition of a Debian repository.
Please add the following line to the file /etc/apt/sources.list:
```bash
deb http://doc.openrobotino.org/download/packages/amd64 ./
```

Action template
===============

For the sake of visualisation action messages should follow the following template:
```
# goal definition
---
# result definition
---
# feedback definition
string current_phase
string current_status
int32 percent_completed
```
