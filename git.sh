#!/bin/bash


read -p "Enter your commit message:" msg

git add . 
git commit -m "$msg"
git push
