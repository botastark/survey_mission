#!/bin/bash
sudo systemctl stop mission.service
sudo systemctl disable mission.service
sudo systemctl daemon-reload
sudo systemctl enable mission.service
sudo systemctl start mission.service


