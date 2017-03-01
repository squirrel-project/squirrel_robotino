set -e
set -v

while true; do echo "INSTALL IS RUNNING" && sleep 60; done&
echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" >> /etc/apt/sources.list
