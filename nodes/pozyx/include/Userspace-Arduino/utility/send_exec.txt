send_exec
=========

Upload and execute code remotely

Pre Requisites

Set up ssh keys

 cd ~/.ssh
 ssh-keygen -t dsa #set password or you can leave it blank
 ssh-copy-id -i ~/.ssh/id_dsa.pub $USERNAME@$IP # replace id_dsa.pub by your public key file if some other name is given
 ssh $USERNAME@$IP # login and enter password once

Here after, you won't be prompted for a password from this machine
