#!/bin/bash


read -p "Enter your commit messag:" msg

git add . 
git commit -m "$msg"
git push
