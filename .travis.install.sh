set -e
set -v

echo "BEFORE INSTALL IS RUNNING"
sudo echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
sudo apt-key update
sudo apt-get install -qq -y --force-yes robotino-api2
