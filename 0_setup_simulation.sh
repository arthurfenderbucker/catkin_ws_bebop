#reset firmwared
fdc drop_all instances
fdc drop_all firmwares
rm /dev/shm/shd_*
sudo systemctl stop firmwared.service

#sudo rm /usr/share/firmwared/firmwares/*

#start firmwared
sudo systemctl start firmwared.service

echo  "PING firmwared"
fdc ping

