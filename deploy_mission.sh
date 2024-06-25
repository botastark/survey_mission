#!/bin/bash
sudo systemctl stop mission.service
sudo systemctl disable mission.service
sudo systemctl daemon-reload
sudo systemctl enable mission.service
sudo systemctl start mission.service


sudo systemctl stop listen.service
sudo systemctl disable listen.service
sudo systemctl daemon-reload
sudo systemctl enable listen.service
sudo systemctl start listen.service


