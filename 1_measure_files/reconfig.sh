#!/bin/bash

rm -rf data/
git config --global user.name "Vekki Sakari"
git config --global user.email "sakari.m.vekki@jyu.fi"
git config -l
cd .ssh/
ls
rm id_ed25519*
ls

cp ../softLink/id_ed25519* .ssh/

#sh-keygen -t ed25519 -C "sakari.m.vekki@jyu.fi"
ls
eval "$(ssh-agent -s)"
ssh-add ./id_ed25519

sudo ntpdate -b time1.mikes.fi
date
 
#xclip -sel clip < ./id_ed25519.pub 
#vivaldi
cd ..
#7804  ls -a
#7805  git clone git@gitlab.j:samavekk/data.git
git clone git@gitlab.jyu.fi:samavekk/data.git
#7807  history |tail -n 40 > reconfig.sh 
