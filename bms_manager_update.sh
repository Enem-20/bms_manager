#!/bin/sh

git stash
git pull origin master

sudo bash ./rosinstall.sh
